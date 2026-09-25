#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Generate an auditable 4.2 m scene from supplied competition artwork.

60 cm annotated lanes control dimensions. Unannotated coordinates and person
placements are explicitly reconstruction assumptions, never official ground truth.
The white lane markings are visual-only, not fictitious LiDAR walls.
"""
import argparse
import hashlib
import json
import math
from pathlib import Path
import shutil
from xml.sax.saxutils import escape
from PIL import Image, ImageDraw, ImageOps

PKG = Path(__file__).resolve().parents[1]
SIGNAL_CYCLE = {"period_s": 28.0, "red_s": 10.0, "green_s": 15.0,
                "yellow_s": 3.0, "light_2_offset_s": 7.0,
                "offsets_s": {"light_1": 0.0, "light_2": 7.0}}


def write_text_lf(path, text):
    """Write generated text deterministically on Windows and Linux."""
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8", newline="\n") as stream:
        stream.write(text)


def save_json(path, value):
    write_text_lf(path, json.dumps(value, ensure_ascii=False, indent=2) + "\n")


def mesh(path, width, height, texture="texture.png", ground=False):
    # Front lies in X-Z with normal -Y; yaw rotates this normal toward observers.
    if ground:
        positions = "0 0 0 {w} 0 0 {w} {h} 0 0 {h} 0".format(w=width,h=height)
        uv = "0 0 1 0 1 1 0 1"
    else:
        positions = "{a} 0 0 {b} 0 0 {b} 0 {h} {a} 0 {h}".format(a=-width/2,b=width/2,h=height)
        uv = "0 0 1 0 1 1 0 1"
    write_text_lf(path, '''<?xml version="1.0" encoding="utf-8"?>
<COLLADA xmlns="http://www.collada.org/2005/11/COLLADASchema" version="1.4.1">
<asset><unit name="meter" meter="1"/><up_axis>Z_UP</up_axis></asset>
<library_images><image id="image"><init_from>../materials/textures/%s</init_from></image></library_images>
<library_effects><effect id="fx"><profile_COMMON>
<newparam sid="surface"><surface type="2D"><init_from>image</init_from></surface></newparam>
<newparam sid="sampler"><sampler2D><source>surface</source></sampler2D></newparam>
<technique sid="common"><lambert><diffuse><texture texture="sampler" texcoord="UVMap"/></diffuse></lambert></technique>
</profile_COMMON></effect></library_effects>
<library_materials><material id="mat"><instance_effect url="#fx"/></material></library_materials>
<library_geometries><geometry id="plane"><mesh>
<source id="pos"><float_array id="pos-array" count="12">%s</float_array><technique_common><accessor source="#pos-array" count="4" stride="3"><param name="X" type="float"/><param name="Y" type="float"/><param name="Z" type="float"/></accessor></technique_common></source>
<source id="uv"><float_array id="uv-array" count="8">%s</float_array><technique_common><accessor source="#uv-array" count="4" stride="2"><param name="S" type="float"/><param name="T" type="float"/></accessor></technique_common></source>
<vertices id="verts"><input semantic="POSITION" source="#pos"/></vertices>
<triangles count="2" material="material"><input semantic="VERTEX" source="#verts" offset="0"/><input semantic="TEXCOORD" source="#uv" offset="1" set="0"/><p>0 0 1 1 2 2 0 0 2 2 3 3</p></triangles>
</mesh></geometry></library_geometries>
<library_visual_scenes><visual_scene id="scene"><node><instance_geometry url="#plane"><bind_material><technique_common><instance_material symbol="material" target="#mat"><bind_vertex_input semantic="UVMap" input_semantic="TEXCOORD" input_set="0"/></instance_material></technique_common></bind_material></instance_geometry></node></visual_scene></library_visual_scenes>
<scene><instance_visual_scene url="#scene"/></scene></COLLADA>''' % (texture, positions, uv))


def card(name, source, width, height, ground=False):
    root = PKG / "models" / name
    (root / "meshes").mkdir(parents=True, exist_ok=True)
    (root / "materials/textures").mkdir(parents=True, exist_ok=True)
    with Image.open(source) as image:
        # A white backing is deliberate: alpha handling differs between Ogre versions.
        if image.mode == "RGBA":
            backing = Image.new("RGBA", image.size, "white")
            backing.alpha_composite(image)
            image = backing.convert("RGB")
        image.save(root / "materials/textures/texture.png")
    mesh(root / "meshes/card.dae", width, height, ground=ground)
    collision = ""
    if not ground:
        collision = '<collision name="card"><pose>0 0 %f 0 0 0</pose><geometry><box><size>%f 0.005 %f</size></box></geometry></collision>' % (height/2,width,height)
    text = '''<?xml version="1.0"?>
<sdf version="1.6"><model name="%s"><static>true</static><link name="body">
<visual name="artwork"><geometry><mesh><uri>model://%s/meshes/card.dae</uri></mesh></geometry><cast_shadows>false</cast_shadows></visual>%s
</link></model></sdf>''' % (name,name,collision)
    write_text_lf(root / "model.sdf", text)
    write_text_lf(root / "model.config", '<model><name>%s</name><version>1.0</version><sdf version="1.6">model.sdf</sdf><description>Official artwork; dimensions tracked in manifest</description></model>' % name)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--materials", required=True, type=Path)
    args = parser.parse_args()
    assets = PKG / "assets"
    assets.mkdir(parents=True, exist_ok=True)
    recognition = []
    for category, subdir, prefix in [("resident","社区人员","resident"),("visitor","非社区人员","visitor")]:
        for source in sorted((args.materials / "人员" / subdir).glob("*.png")):
            name = prefix + "_" + source.stem
            target = assets / (name + ".png")
            shutil.copy2(str(source),str(target))
            with Image.open(source) as image:
                width = 0.145 * image.width / image.height
            recognition.append({"category": category,"label": name,"file": target.name,
                                "source": str(source.relative_to(args.materials)),
                                "sha256": hashlib.sha256(source.read_bytes()).hexdigest(),
                                "height_m": 0.145,"width_m": width,
                                "dimension_status": "approximate_from_ruler_photo_not_official_numeric_spec"})
            card(name, source, width, 0.145)
    plate_labels = [("一", "苏AB8Q62"),("二", "鄂D7B5Q2"),("三", "苏APL12A")]
    for i, (cn, text) in enumerate(plate_labels, 1):
        source = args.materials / "车辆识别" / ("车牌" + cn + ".png")
        name = "plate_%d" % i
        shutil.copy2(str(source), str(assets / (name+".png")))
        recognition.append({"category":"plate", "label":text,"file":name+".png",
                            "source":str(source.relative_to(args.materials)),
                            "sha256":hashlib.sha256(source.read_bytes()).hexdigest(),
                            "width_m":0.095,"height_m":0.03,
                            "dimension_status":"official_text", "text_status":"visually_transcribed"})
        card(name,source,0.095,0.03)
    card("car_background",args.materials/"车辆识别/车牌背景.png",0.345,0.25)
    for colour,cn in [("red","红"),("yellow","黄"),("green","绿")]:
        for state,label in [("on","亮"),("off","暗")]:
            source = args.materials / "红绿灯/红绿灯图片" / (cn+"灯"+label+".png")
            shutil.copy2(str(source),str(assets/(colour+"_"+state+".png")))
    save_json(assets/"manifest.json", {"source":"User supplied official semifinal materials",
              "recognition_assets":recognition,
              "notes":["Known-reference baseline; plate labels are not OCR output.",
                       "Person height is a provisional reconstruction value; verify before filming.",
                       "No person roles inferred from clothing colour."]})
    # Dimensioned geometry. Coordinates are metres, origin at bottom-left.
    a_polygon = [[0.6,3.6],[2.85,3.6],[2.85,0.6],[2.25,0.6],[2.25,3.0],[0.6,3.0]]
    b_polygon = [[0.6,0.6],[1.65,0.6],[1.65,2.25],[0.6,2.25]]
    route = [
        {"name":"start","xy":[3.78,3.90],"yaw":180},
        {"name":"approach_light_1","xy":[2.78,3.90],"yaw":180,"gate":"light_1"},
        {"name":"past_light_1","xy":[1.45,3.90],"yaw":180},
        {"name":"street_a_north","xy":[0.90,3.90],"yaw":-90,"street":"A","observe":True,"expected_category":"person"},
        {"name":"top_left","xy":[0.30,3.90],"yaw":180},
        # Offset the west observation by 10 cm so the two west-facing cards
        # in the A row are not collinear in the camera projection.
        {"name":"street_a_west","xy":[0.30,3.20],"yaw":0,"street":"A","observe":True,"expected_category":"person"},
        {"name":"left_bottom","xy":[0.30,2.63],"yaw":-90},
        {"name":"street_b_west","xy":[0.96,2.63],"yaw":-90,"street":"B","observe":True,"expected_category":"person"},
        {"name":"street_a_south","xy":[1.36,2.63],"yaw":90,"street":"A","observe":True,"expected_category":"person"},
        {"name":"street_b_east","xy":[1.36,2.63],"yaw":-90,"street":"B","observe":True,"expected_category":"person"},
        {"name":"inner_turn","xy":[1.95,2.63],"yaw":0},
        # A legal lane pose with a more normal view of the east-facing card.
        {"name":"street_b_east_side","xy":[1.95,2.30],"yaw":-110,"street":"B","observe":True,"expected_category":"person"},
        {"name":"approach_light_2","xy":[1.95,1.82],"yaw":-90,"gate":"light_2"},
        {"name":"past_light_2","xy":[1.95,0.78],"yaw":-90},
        {"name":"bottom_turn","xy":[1.95,0.30],"yaw":-90},
        {"name":"right_bottom","xy":[3.15,0.30],"yaw":0},
        {"name":"parking_1","xy":[3.15,0.30],"object_xy":[3.833,0.30],"yaw":0,"observe":True,"expected_category":"plate","expected_label":"苏AB8Q62"},
        {"name":"parking_2","xy":[3.15,0.92],"object_xy":[3.833,0.92],"yaw":0,"observe":True,"expected_category":"plate","expected_label":"鄂D7B5Q2"},
        {"name":"parking_3","xy":[3.15,1.54],"object_xy":[3.833,1.54],"yaw":0,"observe":True,"expected_category":"plate","expected_label":"苏APL12A"},
        {"name":"right_top","xy":[3.15,3.90],"yaw":90},
        {"name":"finish","xy":[3.78,3.90],"yaw":180}]
    lights=[{"id":"light_1","xy":[1.80,3.88],"yaw":90},
            {"id":"light_2","xy":[1.95,.56],"yaw":180}]
    layout = {"schema_version":2,"signal_cycle":SIGNAL_CYCLE,"field_size_m":[4.2,4.2],"lane_width_m":0.6,
              "lane_safety_margin_m":0.02,
              "lane_centerline":[entry["xy"] for entry in route],
              "coordinate_status":"dimension_constrained_reconstruction_pending_official_coordinate_confirmation",
              "a_polygon":a_polygon,"b_polygon":b_polygon,
              "parking_boundary_x":3.45,"parking_open_above_y":3.60,
              "population":{"total":16,"resident":14,"visitor":2,
                             "by_street":{"A":{"total":8,"resident":7,"visitor":1},
                                           "B":{"total":8,"resident":7,"visitor":1}}},
              "person_orientation_policy":{"A":["north","south","west"],
                                             "B":["north","east"]},
              "route":route,
              "lights":lights,
              "stop_lines":[{"id":"light_1","point":[2.44,3.9],"direction":[-1,0]},
                            {"id":"light_2","point":[1.95,1.49],"direction":[0,-1]}],
              "assumptions":["The two diagrams are schematic and differ in proportions.",
                 "60 cm labels override pixel-derived distances.",
                 "Person artwork assignment and exact placements are illustrative, not specified by the diagram.",
                 "A arrows permit north/south/west; B arrows permit north/east. Card fronts and observation poses cover every permitted direction.",
                 "Light dimensions 0.64 x 0.14 m and overall 0.48 m are official; mounting locations are reconstructed."]}
    save_json(PKG/"config/layout.json",layout)
    scale=300
    floor=Image.new("RGB",(1260,1260),(24,27,29)); draw=ImageDraw.Draw(floor)
    def p(point): return (int(point[0]*scale),int((4.2-point[1])*scale))
    for poly in [a_polygon,b_polygon]:
        draw.line([p(x) for x in poly+[poly[0]]],fill="white",width=6)
    draw.rectangle((3,3,1256,1256),outline="white",width=6)
    draw.line([p([3.45,0]),p([3.45,3.6])],fill="white",width=6)
    for y in [.62,1.24,1.86,3.60]: draw.line([p([3.45,y]),p([4.2,y])],fill="white",width=6)
    for y in [3.66,3.77,3.88,3.99,4.10]:
        draw.line([p([2.01,y]),p([2.35,y])],fill="white",width=15)
    for x in [1.72,1.83,1.94,2.05,2.16]:
        draw.line([p([x,1.21]),p([x,1.42])],fill="white",width=15)
    draw.line([p([2.44,3.60]),p([2.44,4.2])],fill="white",width=8)
    draw.line([p([1.65,1.49]),p([2.25,1.49])],fill="white",width=8)
    draw.line([p([.60,1.49]),p([1.65,1.49])],fill="white",width=6)
    floor.save(assets/"dimensioned_floor.png")
    card("official_floor",assets/"dimensioned_floor.png",4.2,4.2,ground=True)
    preview=floor.copy(); draw=ImageDraw.Draw(preview)
    draw.line([p(r["xy"]) for r in route],fill="#66b9e8",width=5)
    for i,r in enumerate(route):
        x,y=p(r["xy"]);draw.ellipse((x-7,y-7,x+7,y+7),fill="#edba55")
        draw.text((x+8,y+8),str(i),fill="#edba55")
    preview.save(assets/"route_preview.png")
    world=['<?xml version="1.0"?><sdf version="1.6"><world name="official_semifinal">',
           '<physics type="ode"><max_step_size>0.001</max_step_size><real_time_update_rate>200</real_time_update_rate></physics>',
           '<include><uri>model://sun</uri></include><include><uri>model://ground_plane</uri></include>',
           '<include><uri>model://official_floor</uri><pose>0 0 0.002 0 0 0</pose></include>']
    # Low static masses inside the no-drive islands provide real lidar returns
    # and map structure while staying clear of lanes, stop lines, and cards.
    for name,x,y,sx,sy in [("building_a",2.60,2.05,.30,.70),
                            ("building_b",.75,.95,.25,.40)]:
        world.append('<model name="%s"><static>true</static><pose>%f %f 0 0 0 0</pose>'%(name,x,y))
        world.append('<link name="body"><visual name="mass"><pose>0 0 .25 0 0 0</pose><geometry><box><size>%f %f .50</size></box></geometry><material><ambient>.35 .38 .42 1</ambient><diffuse>.35 .38 .42 1</diffuse></material></visual>'%(sx,sy))
        world.append('<collision name="mass"><pose>0 0 .25 0 0 0</pose><geometry><box><size>%f %f .50</size></box></geometry></collision></link></model>'%(sx,sy))
    instances=[]
    def include(model,name,x,y,yaw,z=0.003):
        world.append('<include><uri>model://%s</uri><name>%s</name><pose>%f %f %f 0 0 %f</pose></include>'%(model,name,x,y,z,yaw))
        instances.append({"model":model,"name":name,"x":x,"y":y,"z":z,"yaw":yaw})
    # Card front is local -Y. The official arrows are treated as allowed
    # viewing directions: A has north/south/west and B has north/east. Keep
    # the corresponding observation poses in the route so no card is forced
    # to be read from its mirrored back face.
    person_instances=[
        # A (top island): two north-facing, three south-facing and three
        # west-facing cards, matching the three legal directions shown by
        # the official arrow mark.
        ("resident_1",.70,3.18,math.pi),("resident_7",.86,3.18,math.pi),
        ("resident_3",1.02,3.18,-math.pi/2),
        # Keep the full 16-person inventory, but separate this west-facing
        # card from the adjacent row so its only legal view is not occluded.
        ("resident_4",1.08,3.10,-math.radians(140)),
        ("visitor_F1",1.08,3.40,0),("resident_10",1.25,3.40,0),
        ("resident_5",1.42,3.40,-math.pi/2),("resident_16",1.59,3.40,0),
        # B (lower island): five north-facing and three east-facing cards.
        ("resident_2",.76,1.95,math.pi),("resident_6",.96,1.95,math.pi),
        ("resident_9",1.15,1.95,math.pi),("visitor_F2",1.36,1.95,math.pi),
        ("resident_14",1.54,1.95,math.pi),
        ("resident_11",.76,1.70,math.pi),("resident_12",1.15,1.70,math.pi),
        ("resident_13",1.54,1.70,math.pi/2)]
    for i,(model,x,y,yaw) in enumerate(person_instances):
        include(model,"person_%02d"%i,x,y,yaw)
    for i,y in enumerate([.30,.92,1.54],1):
        include("car_background","car_%d"%i,3.84,y,-math.pi/2)
        include("plate_%d"%i,"plate_%d"%i,3.833,y,-math.pi/2,.073)
    # Horizontal three-lamp housings. A renderer plugin sets emissive active colours.
    for lamp in lights:
        name=lamp["id"];x,y=lamp["xy"];yaw=math.radians(lamp["yaw"])
        parts=['<model name="%s"><static>true</static><pose>%f %f 0 0 0 %f</pose><link name="housing">'%(name,x,y,yaw)]
        parts.append('<visual name="box"><pose>0 0 .41 0 0 0</pose><geometry><box><size>.64 .04 .14</size></box></geometry><material><ambient>.02 .02 .02 1</ambient><diffuse>.02 .02 .02 1</diffuse></material></visual>')
        for colour,lx,rgb in [("red",-.22,"1 0 0 1"),("yellow",0,"1 .7 0 1"),("green",.22,"0 1 0 1")]:
            parts.append('<visual name="%s"><pose>%f -.025 .41 1.570796 0 0</pose><geometry><cylinder><radius>.052</radius><length>.012</length></cylinder></geometry><material><ambient>%s</ambient><diffuse>%s</diffuse></material></visual>'%(colour,lx,rgb,rgb))
        for lx in [-.29,.29]:
            parts.append('<visual name="leg_%s"><pose>%f 0 .17 0 0 0</pose><geometry><box><size>.025 .025 .34</size></box></geometry></visual><collision name="leg_%s"><pose>%f 0 .17 0 0 0</pose><geometry><box><size>.025 .025 .34</size></box></geometry></collision>'%(lx,lx,lx,lx))
        offset = float(SIGNAL_CYCLE["offsets_s"][name])
        parts.append('</link><plugin name="signal_cycle" filename="libsemifinal_signal.so"><offset>%s</offset><period>%s</period><red>%s</red><green>%s</green></plugin></model>' % (offset, SIGNAL_CYCLE["period_s"], SIGNAL_CYCLE["red_s"], SIGNAL_CYCLE["green_s"]));world.extend(parts)
    world.append('</world></sdf>')
    (PKG/"worlds").mkdir(exist_ok=True)
    write_text_lf(PKG/"worlds/official_semifinal.world", "\n".join(world) + "\n")
    save_json(PKG/"config/scene_instances_for_evaluation_only.json",instances)
    print("Generated",len(recognition),"reference assets,",len(instances),"scene instances.")


if __name__ == "__main__":
    main()
