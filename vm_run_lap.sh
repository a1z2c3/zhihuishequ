#!/usr/bin/env bash
# Run one patrol lap with an independent, retained Gazebo evaluation record.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd -P)"
PKG="$ROOT/catkin_ws/src/smart_community_semifinal"
RUNS_ROOT="${1:-$ROOT/docs/evidence}"
RUN_MODE="${RUN_MODE:-patrol}"
if [ "$RUN_MODE" != patrol ] && [ "$RUN_MODE" != navigation ]; then
  echo 'RUN_MODE must be patrol or navigation.' >&2
  exit 2
fi
export RUN_MODE
: "${ROS_DISTRO:?Source /opt/ros/melodic/setup.bash first}"
if [ "$ROS_DISTRO" != melodic ] || [ ! -f "$ROOT/catkin_ws/devel/setup.bash" ]; then
  echo 'Build the Melodic workspace with setup_and_check.sh first.' >&2
  exit 2
fi
source "$ROOT/catkin_ws/devel/setup.bash"
if [ "$(rospack find smart_community_semifinal)" != "$PKG" ]; then
  echo 'The sourced ROS package is not from this copy of the project.' >&2
  exit 2
fi
if [ -z "${DISPLAY:-}" ]; then
  echo 'Gazebo camera rendering requires DISPLAY (use Xvfb for a headless VM).' >&2
  exit 2
fi
if ! command -v timeout >/dev/null || ! command -v sha256sum >/dev/null; then
  echo 'GNU timeout and sha256sum are required.' >&2
  exit 2
fi
if timeout 3 rosnode list 2>/dev/null |
    grep -Eq '^/(gazebo|gazebo_gui|semifinal_patrol|independent_run_evaluator)$'; then
  echo 'A Gazebo or patrol run is already active on this ROS master.' >&2
  exit 2
fi

mkdir -p "$RUNS_ROOT"
RUN_DIR="$(mktemp -d "$RUNS_ROOT/${RUN_MODE}_$(date +%Y%m%d_%H%M%S)_XXXXXX")"
echo "Run evidence: $RUN_DIR"
(
  cd "$ROOT"
  find README.md setup_and_check.sh vm_run_lap.sh catkin_ws/src/smart_community_semifinal \
    -type f ! -path '*/__pycache__/*' ! -name '*.pyc' -print0 |
    sort -z | xargs -0 sha256sum
) > "$RUN_DIR/source_sha256.txt"

SCENE_PID=''; EVAL_PID=''; PATROL_PID=''
stop_process() {
  local pid="$1"
  [ -n "$pid" ] || return 0
  if kill -0 "$pid" 2>/dev/null; then
    kill -INT "$pid" 2>/dev/null || true
    timeout 20 tail --pid="$pid" -f /dev/null >/dev/null 2>&1 || true
    if kill -0 "$pid" 2>/dev/null; then
      kill -TERM "$pid" 2>/dev/null || true
      timeout 5 tail --pid="$pid" -f /dev/null >/dev/null 2>&1 || true
    fi
    if kill -0 "$pid" 2>/dev/null; then kill -KILL "$pid" 2>/dev/null || true; fi
  fi
  wait "$pid" 2>/dev/null || true
}
cleanup() {
  stop_process "$PATROL_PID"
  stop_process "$EVAL_PID"
  stop_process "$SCENE_PID"
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

if [ "$RUN_MODE" = navigation ]; then
  roslaunch smart_community_semifinal navigation.launch gui:="${GUI:-false}" \
    start_client:=false evidence_dir:="$RUN_DIR/perception" \
    > "$RUN_DIR/scene.log" 2>&1 &
else
  roslaunch smart_community_semifinal patrol.launch gui:="${GUI:-false}" \
    start_task:=false evidence_dir:="$RUN_DIR/perception" \
    > "$RUN_DIR/scene.log" 2>&1 &
fi
SCENE_PID=$!

# Do not start the evaluator until the world and robot are available.
ready=false
for unused in $(seq 1 120); do
  if ! kill -0 "$SCENE_PID" 2>/dev/null; then break; fi
  if timeout 3 rosservice call /gazebo/get_model_state semifinal_bot world \
      2>/dev/null | grep -q 'success: True'; then ready=true; break; fi
  sleep 1
done
if [ "$ready" != true ]; then
  echo 'Gazebo robot did not become ready; see scene.log.' >&2
  exit 1
fi
if ! timeout 185 python - <<'PY'
from __future__ import print_function
import sys, time, rospy, tf

rospy.init_node('semifinal_wait_for_map', anonymous=True, disable_signals=True)
listener = tf.TransformListener()
deadline = time.time() + 180
while time.time() < deadline and not rospy.is_shutdown():
    try:
        listener.lookupTransform('map', 'base_footprint', rospy.Time(0))
        sys.exit(0)
    except tf.Exception:
        time.sleep(.5)
sys.exit(1)
PY
then
  echo 'Map-to-robot TF did not become ready; see scene.log.' >&2
  exit 1
fi

timeout --signal=INT --kill-after=10s 1650s \
  python "$PKG/tools/evaluate_run.py" _output_dir:="$RUN_DIR" \
  > "$RUN_DIR/evaluator.log" 2>&1 &
EVAL_PID=$!
subscribed=false
for unused in $(seq 1 30); do
  if ! kill -0 "$EVAL_PID" 2>/dev/null; then break; fi
  if timeout 3 rostopic info /semifinal/task_status 2>/dev/null |
      grep -q '/independent_run_evaluator'; then subscribed=true; break; fi
  sleep 1
done
if [ "$subscribed" != true ]; then
  echo 'Evaluator did not subscribe before task start; see evaluator.log.' >&2
  exit 1
fi

if [ "$RUN_MODE" = navigation ]; then
  rosrun smart_community_semifinal move_base_waypoint_client.py \
    _layout:="$PKG/config/layout.json" > "$RUN_DIR/task.log" 2>&1 &
else
  rosrun smart_community_semifinal patrol_node.py \
    _layout:="$PKG/config/layout.json" > "$RUN_DIR/task.log" 2>&1 &
fi
PATROL_PID=$!
wait "$EVAL_PID" || true
EVAL_PID=''

# Capture the map while the scene and gmapping are still alive.  The checked-in
# map predates scene changes and must not be treated as current navigation data.
if [ "$RUN_MODE" = patrol ]; then
  if ! timeout 30 rosrun map_server map_saver -f "$RUN_DIR/slam_map" \
      > "$RUN_DIR/map_saver.log" 2>&1; then
    echo 'Map snapshot unavailable; inspect map_saver.log.' >&2
  fi
fi

python - "$RUN_DIR" "$PKG" <<'PY'
from __future__ import print_function
import io, json, os, sys

folder = sys.argv[1]
path = os.path.join(folder, 'run_result.json')
result = {}
read_error = None
try:
    with io.open(path, encoding='utf-8') as stream:
        result = json.load(stream)
except (IOError, ValueError) as exc:
    read_error = str(exc)
task = result.get('task') or {}
crossings = result.get('stop_crossings') or []
violations = result.get('body_violations') or []
warnings = result.get('safety_margin_warnings') or []
red_body = result.get('red_body_violations') or []
summary = result.get('street_summary') or {}
with io.open(os.path.join(sys.argv[2], 'config', 'layout.json'),
             encoding='utf-8') as stream:
    layout = json.load(stream)
population = layout.get('population') or {}
street_population = population.get('by_street') or {}
counts_ok = all((summary.get(street) or {}).get('resident') ==
                (street_population.get(street) or {}).get('resident') and
                (summary.get(street) or {}).get('visitor') ==
                (street_population.get(street) or {}).get('visitor')
                for street in ('A', 'B'))
population_total = sum((summary.get(street) or {}).get('total',0)
                       for street in ('A','B'))
population_ok = (population_total == population.get('total') == 16)
expected_plates = set(target['expected_label'] for target in layout['route']
                      if target.get('expected_category') == 'plate')
expected_lights = set(target['gate'] for target in layout['route']
                      if target.get('gate'))
crossed_lights = set(item.get('light_id') for item in crossings
                     if item.get('pass'))
reference_stamps = {}
ocr_verified_plates = set()
for event in result.get('events') or []:
    stamp = event.get('stamp')
    for detection in event.get('detections') or []:
        if detection.get('commit_state') != 'committed_reference_match':
            continue
        if detection.get('unexpected_label'):
            continue
        label = detection.get('label')
        if label:
            reference_stamps.setdefault(label, set()).add(stamp)
        if detection.get('ocr_status') == 'recognized_and_verified':
            ocr_verified_plates.add(label)
reference_quorum_plates = set(label for label, stamps in reference_stamps.items()
                             if len(stamps) >= 3)
plates_ok = expected_plates <= reference_quorum_plates
passed = (task.get('phase') == 'done' and
          expected_lights and expected_lights <= crossed_lights and
          all(item.get('pass') for item in crossings) and not violations and
          'red_body_violations' in result and not red_body and
          counts_ok and population_ok and expected_plates and plates_ok)
lines = [
    '# %s lap evidence' % os.environ.get('RUN_MODE','patrol'), '',
    'Result: %s' % ('PASS' if passed else 'FAIL / INCOMPLETE'),
    'Task phase: %s' % task.get('phase', 'not reported'),
    'Task error: %s' % task.get('error', 'not reported'),
    'Simulated seconds: %s' % result.get('simulated_elapsed'),
    'Stop-line crossings: %d (%d green)' %
        (len(crossings), sum(bool(item.get('pass')) for item in crossings)),
    'Distinct expected lights crossed: %d/%d' %
        (len(expected_lights & crossed_lights), len(expected_lights)),
    'Body violations: %d' % len(violations),
    'Lane reserve warnings: %d' % len(warnings),
    'Minimum physical lane clearance (m): %s' % result.get('minimum_lane_clearance_m'),
    'Red-light body overlaps: %d' % len(red_body),
    'Street counts match supplied scene (A/B: 7 resident + 1 visitor): %s' % counts_ok,
    'Total detected population matches supplied scene (16): %s' % population_ok,
    'Reference-quorum plates: %d/%d' %
        (len(expected_plates & reference_quorum_plates), len(expected_plates)),
    'OCR-verified plates (independent quality channel): %d/%d' %
        (len(expected_plates & ocr_verified_plates), len(expected_plates)),
    'Camera samples: %d' % len(result.get('camera_samples') or []), '',
    'SLAM map snapshot: %s' % ('saved' if os.path.isfile(os.path.join(folder,'slam_map.pgm'))
                                 else 'not saved'),
    'run_result.json is independent Gazebo evaluation; source_sha256.txt ',
    'records the exact source files. Logs and perception/ retain failure evidence.',
]
if read_error:
    lines.append('Evaluation read error: %s' % read_error)
with io.open(os.path.join(folder, 'INDEX.md'), 'w', encoding='utf-8') as stream:
    stream.write(u'\n'.join(lines) + u'\n')
print('\n'.join(lines))
sys.exit(0 if passed else 1)
PY
