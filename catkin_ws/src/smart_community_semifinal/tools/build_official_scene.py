#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""根据素材生成场景；重建坐标为仿真设定。"""
import argparse
import hashlib
import json
import math
from pathlib import Path
import shutil
from xml.sax.saxutils import escape
from PIL import Image, ImageDraw, ImageOps

PKG = Path(__file__).resolve().parents[1]
SIGNAL_CYCLE = {"period_s": 30.0, "red_s": 10.0, "green_s": 15.0,
                "yellow_s": 5.0, "light_2_offset_s": 7.0,
                "offsets_s": {"light_1": 16.0, "light_2": 7.0}}

PERSON_WIDTH_M = 0.05
PERSON_HEIGHT_M = 0.15
PERSON_THICKNESS_M = 0.005
PERSON_BASE_WIDTH_M = 0.05
PERSON_BASE_DEPTH_M = 0.03
PERSON_BASE_HEIGHT_M = 0.012
FIELD_M = 4.2


def write_text_lf(path, text):
    """统一生成文本的换行格式。"""
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8", newline="\n") as stream:
        stream.write(text)


def save_json(path, value):
    write_text_lf(path, json.dumps(value, ensure_ascii=False, indent=2) + "\n")


def artwork_dimensions(image_size, width, height):
    """返回立牌内未变形图案的实际尺寸。"""
    image_width, image_height = image_size
    target = float(width) / float(height)
    if image_width / float(image_height) > target:
        canvas_height = int(round(image_width / target))
        return float(width), float(height) * image_height / float(canvas_height)
    canvas_width = int(round(image_height * target))
    return float(width) * image_width / float(canvas_width), float(height)


def mesh(path, width, height, texture="texture.png", ground=False, thickness=0.0):
    if ground:
        positions = "0 0 0 {w} 0 0 {w} {h} 0 0 {h} 0".format(w=width,h=height)
        uv = "0 0 1 0 1 1 0 1"
    else:
        if thickness:
            t=thickness/2.0; a=-width/2; b=width/2; z0=0.; z1=height
            positions="%s" % " ".join("%g %g %g"%p for p in [
                (a,-t,z0),(b,-t,z0),(b,-t,z1),(a,-t,z1),
                (a,t,z0),(b,t,z0),(b,t,z1),(a,t,z1)])
            uv="0 0 1 0 1 1 0 1 1 0 0 0 0 1 1 1"
            write_text_lf(path, '''<?xml version="1.0" encoding="utf-8"?>
<COLLADA xmlns="http://www.collada.org/2005/11/COLLADASchema" version="1.4.1"><asset><unit name="meter" meter="1"/><up_axis>Z_UP</up_axis></asset>
<library_images><image id="image"><init_from>../materials/textures/%s</init_from></image></library_images>
<library_effects><effect id="fx"><profile_COMMON><newparam sid="surface"><surface type="2D"><init_from>image</init_from></surface></newparam><newparam sid="sampler"><sampler2D><source>surface</source></sampler2D></newparam><technique sid="common"><lambert><diffuse><texture texture="sampler" texcoord="UVMap"/></diffuse></lambert></technique></profile_COMMON></effect></library_effects>
<library_materials><material id="mat"><instance_effect url="#fx"/></material></library_materials>
<library_geometries><geometry id="box"><mesh><source id="pos"><float_array id="pa" count="24">%s</float_array><technique_common><accessor source="#pa" count="8" stride="3"><param name="X" type="float"/><param name="Y" type="float"/><param name="Z" type="float"/></accessor></technique_common></source><source id="uv"><float_array id="ua" count="16">%s</float_array><technique_common><accessor source="#ua" count="8" stride="2"><param name="S" type="float"/><param name="T" type="float"/></accessor></technique_common></source><vertices id="v"><input semantic="POSITION" source="#pos"/></vertices><triangles count="12" material="material"><input semantic="VERTEX" source="#v" offset="0"/><input semantic="TEXCOORD" source="#uv" offset="1" set="0"/><p>0 0 1 1 2 2 0 0 2 2 3 3 4 4 6 6 5 5 4 4 7 7 6 6 0 0 4 4 5 5 0 0 5 5 1 1 1 1 5 5 6 6 1 1 6 6 2 2 2 2 6 6 7 7 2 2 7 7 3 3 3 3 7 7 4 4 3 3 0 0 4 4</p></triangles></mesh></geometry></library_geometries><library_visual_scenes><visual_scene id="scene"><node><instance_geometry url="#box"><bind_material><technique_common><instance_material symbol="material" target="#mat"><bind_vertex_input semantic="UVMap" input_semantic="TEXCOORD" input_set="0"/></instance_material></technique_common></bind_material></instance_geometry></node></visual_scene></library_visual_scenes><scene><instance_visual_scene url="#scene"/></scene></COLLADA>''' % (texture,positions,uv))
            return
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
        if image.mode == "RGBA":
            backing = Image.new("RGBA", image.size, "white")
            backing.alpha_composite(image)
            image = backing.convert("RGB")
        if not ground and width > 0 and height > 0:
            target = float(width) / float(height)
            w, h = image.size
            if w / float(h) > target:
                new_size = (w, int(round(w / target)))
                offset = (0, (new_size[1] - h) // 2)
            else:
                new_size = (int(round(h * target)), h)
                offset = ((new_size[0] - w) // 2, 0)
            if new_size != (w, h):
                canvas = Image.new("RGB", new_size, "white")
                canvas.paste(image, offset)
                image = canvas
        image.save(root / "materials/textures/texture.png")
    mesh(root / "meshes/card.dae", width, height, ground=ground,
         thickness=PERSON_THICKNESS_M if not ground and name.startswith(("resident_","visitor_")) else 0.0)
    collision = ""
    support = ""
    if not ground:
        collision = '<collision name="card"><pose>0 0 %f 0 0 0</pose><geometry><box><size>%f %f %f</size></box></geometry></collision>' % (height/2,width,PERSON_THICKNESS_M,height)
        if name.startswith(("resident_", "visitor_")):
            support = ('<visual name="stand_base"><pose>0 0 %f 0 0 0</pose>'
                       '<geometry><box><size>%f %f %f</size></box></geometry>'
                       '<material><ambient>.16 .18 .20 1</ambient><diffuse>.16 .18 .20 1</diffuse></material></visual>'
                       '<collision name="stand_base"><pose>0 0 %f 0 0 0</pose>'
                       '<geometry><box><size>%f %f %f</size></box></geometry></collision>' %
                       (PERSON_BASE_HEIGHT_M/2, PERSON_BASE_WIDTH_M, PERSON_BASE_DEPTH_M,
                        PERSON_BASE_HEIGHT_M,
                        PERSON_BASE_HEIGHT_M/2, PERSON_BASE_WIDTH_M, PERSON_BASE_DEPTH_M,
                        PERSON_BASE_HEIGHT_M))
    text = '''<?xml version="1.0"?>
<sdf version="1.6"><model name="%s"><static>true</static><link name="body">
<visual name="artwork"><geometry><mesh><uri>model://%s/meshes/card.dae</uri></mesh></geometry><cast_shadows>false</cast_shadows></visual>%s%s
 </link></model></sdf>''' % (name,name,collision,support)
    write_text_lf(root / "model.sdf", text)
    write_text_lf(root / "model.config", '<model><name>%s</name><version>1.0</version><sdf version="1.6">model.sdf</sdf><description>Official artwork; dimensions tracked in manifest</description></model>' % name)


PLATE_WIDTH_M = 0.095
PLATE_HEIGHT_M = 0.03
PLATE_IMAGE_SIZE = (380, 120)
PLATE_LABELS = (("一", "苏AB8Q62", None),
                (None, "苏DB812A", "random_1.png"),
                (None, "鄂DP8522", "random_2.png"))


def plate_artwork(source):
    """将完整车牌图案映射到实际宽高比。"""
    with Image.open(source) as original:
        image = original.convert("RGBA")
        background = Image.new("RGBA", image.size, "white")
        background.alpha_composite(image)
        return background.convert("RGB").resize(PLATE_IMAGE_SIZE, Image.LANCZOS)


def build_plate_assets(materials):
    assets = PKG / "assets"
    recognition = []
    for i, (cn, text, random_file) in enumerate(PLATE_LABELS, 1):
        if random_file:
            source = assets / "random_plates" / random_file
            origin = "assets/random_plates/" + random_file
            txt_status = "randomly_generated"
        else:
            source = materials / "车辆识别" / ("车牌" + cn + ".png")
            origin = str(source.relative_to(materials))
            txt_status = "visually_transcribed"
        name = "plate_%d" % i
        target = assets / (name + ".png")
        plate_artwork(source).save(target)
        recognition.append({"category": "plate", "label": text, "file": target.name,
                            "source": origin,
                            "sha256": hashlib.sha256(source.read_bytes()).hexdigest(),
                            "reference_sha256": hashlib.sha256(target.read_bytes()).hexdigest(),
                            "width_m": PLATE_WIDTH_M, "height_m": PLATE_HEIGHT_M,
                            "artwork_width_m": PLATE_WIDTH_M,
                            "artwork_height_m": PLATE_HEIGHT_M,
                            "dimension_status": "official_text", "text_status": txt_status})
        card(name, target, PLATE_WIDTH_M, PLATE_HEIGHT_M)
    return recognition


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--materials", required=True, type=Path)
    parser.add_argument("--plates-only", action="store_true",
                        help="Refresh plate references, models and manifest only; keep scene/persons/maps unchanged")
    args = parser.parse_args()
    assets = PKG / "assets"
    assets.mkdir(parents=True, exist_ok=True)
    if args.plates_only:
        manifest_path = assets / "manifest.json"
        manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
        plates = build_plate_assets(args.materials)
        manifest["recognition_assets"] = [row for row in manifest["recognition_assets"]
                                          if row["category"] != "plate"] + plates
        save_json(manifest_path, manifest)
        return
    recognition = []
    for category, subdir, prefix in [("resident","社区人员","resident"),("visitor","非社区人员","visitor")]:
        for source in sorted((args.materials / "人员" / subdir).glob("*.png")):
            name = prefix + "_" + source.stem
            target = assets / (name + ".png")
            shutil.copy2(str(source),str(target))
            width = PERSON_WIDTH_M
            with Image.open(source) as image:
                artwork_width, artwork_height = artwork_dimensions(image.size, width, PERSON_HEIGHT_M)
            recognition.append({"category": category,"label": name,"file": target.name,
                                "source": str(source.relative_to(args.materials)),
                                "sha256": hashlib.sha256(source.read_bytes()).hexdigest(),
                                "height_m": PERSON_HEIGHT_M,"width_m": width,
                                "artwork_width_m": artwork_width,
                                "artwork_height_m": artwork_height,
                                "thickness_m": PERSON_THICKNESS_M,
                                "dimension_status": "official_training_sheet"})
            card(name, source, width, PERSON_HEIGHT_M)
    recognition.extend(build_plate_assets(args.materials))
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
    a_polygon = [[0.6,3.6],[2.85,3.6],[2.85,0.6],[2.25,0.6],[2.25,3.0],[0.6,3.0]]
    b_polygon = [[0.6,0.6],[1.65,0.6],[1.65,2.25],[0.6,2.25]]
    route = [
        {"name":"start","xy":[3.78,3.90],"yaw":180},
        {"name":"approach_light_1","xy":[2.78,3.90],"yaw":180,"gate":"light_1"},
        {"name":"past_light_1","xy":[1.45,3.90],"yaw":180},
        {"name":"street_a_north","xy":[0.90,3.90],"yaw":-90,"street":"A","observe":True,"expected_category":"person"},
        {"name":"top_left","xy":[0.30,3.90],"yaw":180},
        {"name":"street_a_west","xy":[0.30,3.07],"yaw":0,"street":"A","observe":True,"expected_category":"person"},
        {"name":"left_bottom","xy":[0.30,2.63],"yaw":-90},
        {"name":"street_b_west","xy":[0.96,2.63],"yaw":-90,"street":"B","observe":True,"expected_category":"person"},
        {"name":"street_a_south","xy":[1.36,2.63],"yaw":90,"street":"A","observe":True,"expected_category":"person"},
        {"name":"street_b_east","xy":[1.36,2.63],"yaw":-90,"street":"B","observe":True,"expected_category":"person"},
        {"name":"inner_turn","xy":[1.95,2.63],"yaw":0},
        {"name":"street_b_east_side","xy":[1.95,2.30],"yaw":-110,"street":"B","observe":True,"expected_category":"person"},
        {"name":"approach_light_2","xy":[1.95,1.82],"yaw":-90,"gate":"light_2"},
        {"name":"past_light_2","xy":[1.95,0.78],"yaw":-90},
        {"name":"bottom_turn","xy":[1.95,0.30],"yaw":-90},
        {"name":"right_bottom","xy":[3.15,0.30],"yaw":0},
        {"name":"parking_1","xy":[3.15,0.30],"object_xy":[3.833,0.30],"yaw":0,"observe":True,"expected_category":"plate","expected_label":"苏AB8Q62"},
        {"name":"parking_2","xy":[3.15,0.92],"object_xy":[3.833,0.92],"yaw":0,"observe":True,"expected_category":"plate","expected_label":"苏DB812A"},
        {"name":"parking_3","xy":[3.15,1.54],"object_xy":[3.833,1.54],"yaw":0,"observe":True,"expected_category":"plate","expected_label":"鄂DP8522"},
        {"name":"right_top","xy":[3.15,3.90],"yaw":90},
        {"name":"finish","xy":[3.78,3.90],"yaw":180}]
    lights=[{"id":"light_1","xy":[1.80,3.88],"yaw":90},
            {"id":"light_2","xy":[1.95,.56],"yaw":180}]
    layout = {"schema_version":2,"signal_cycle":SIGNAL_CYCLE,"field_size_m":[FIELD_M,FIELD_M],"lane_width_m":0.6,
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
                 "Light dimensions 0.64 x 0.14 m and overall 0.48 m are official; mounting locations are reconstructed.",
                 "The dimensioned light photo gives a horizontal 0.64 x 0.14 m housing with its bottom edge at 0.34 m and its top edge at 0.48 m; both signals are modelled in that attitude. The plan diagram draws the north signal as a vertical stack, but a 0.64 m tall housing could not satisfy the official 0.34/0.48 m heights, and at the 2.44 m stop line a lamp raised to z=0.63 m would leave the camera's vertical field of view (limit z=0.50 m at 0.64 m range). Horizontal mounting is therefore the choice that keeps every lamp observable."]}
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
    FIELD_WALL_T = 0.005
    FIELD_WALL_H = 0.50
    FIELD_M_ = 4.2
    FIELD_WALL_INNER = FIELD_M_ + 0.40
    for name,x,y,sx,sy,sz in [("bld_B",1.14,1.155,.64,1.01,.35),
                              ("bld_A_north",2.14,3.265,.42,.29,.35),
                              ("bld_A_east_thin",2.65,1.625,.06,1.11,.35),
                              ("bld_parking_wall",4.045,1.625,.27,3.15,.30),
                              ("field_wall_south",FIELD_M_/2,-FIELD_WALL_INNER+FIELD_WALL_T/2,
                               FIELD_M_+2*FIELD_WALL_INNER,FIELD_WALL_T,FIELD_WALL_H),
                              ("field_wall_north",FIELD_M_/2,FIELD_WALL_INNER-FIELD_WALL_T/2,
                               FIELD_M_+2*FIELD_WALL_INNER,FIELD_WALL_T,FIELD_WALL_H),
                              ("field_wall_west",-FIELD_WALL_INNER+FIELD_WALL_T/2,FIELD_M_/2,
                               FIELD_WALL_T,FIELD_M_+2*FIELD_WALL_INNER,FIELD_WALL_H),
                              ("field_wall_east",FIELD_WALL_INNER-FIELD_WALL_T/2,FIELD_M_/2,
                               FIELD_WALL_T,FIELD_M_+2*FIELD_WALL_INNER,FIELD_WALL_H)]:
        world.append('<model name="%s"><static>true</static><pose>%f %f 0 0 0 0</pose>'%(name,x,y))
        world.append('<link name="body"><visual name="mass"><pose>0 0 %f 0 0 0</pose><geometry><box><size>%f %f %f</size></box></geometry><material><ambient>.35 .38 .42 1</ambient><diffuse>.35 .38 .42 1</diffuse></material></visual>'%(sz/2,sx,sy,sz))
        world.append('<collision name="mass"><pose>0 0 %f 0 0 0</pose><geometry><box><size>%f %f %f</size></box></geometry></collision></link></model>'%(sz/2,sx,sy,sz))
    instances=[]
    def include(model,name,x,y,yaw,z=0.003):
        world.append('<include><uri>model://%s</uri><name>%s</name><pose>%f %f %f 0 0 %f</pose></include>'%(model,name,x,y,z,yaw))
        instances.append({"model":model,"name":name,"x":x,"y":y,"z":z,"yaw":yaw})
    person_instances=[
        ("resident_1",.70,3.18,math.pi),("resident_7",.86,3.18,math.pi),
        ("resident_3",1.02,3.18,-math.pi/2),
        ("resident_4",1.08,3.10,-math.radians(140)),
        ("visitor_F1",1.08,3.40,0),("resident_10",1.25,3.40,0),
        ("resident_5",1.42,3.40,-math.pi/2),("resident_16",1.59,3.40,0),
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
    for lamp in lights:
        name=lamp["id"];x,y=lamp["xy"];yaw=math.radians(lamp["yaw"])
        parts=['<model name="%s"><static>true</static><pose>%f %f 0 0 0 %f</pose><link name="housing">'%(name,x,y,yaw)]
        parts.append('<visual name="box"><pose>0 0 .41 0 0 0</pose><geometry><box><size>.64 .05 .14</size></box></geometry><material><ambient>.02 .02 .02 1</ambient><diffuse>.02 .02 .02 1</diffuse></material></visual>')
        for colour,lx,rgb in [("red",-.22,"1 0 0 1"),("yellow",0,"1 .7 0 1"),("green",.22,"0 1 0 1")]:
            parts.append('<visual name="%s"><pose>%f -.025 .41 1.570796 0 0</pose><geometry><cylinder><radius>.052</radius><length>.012</length></cylinder></geometry><material><ambient>%s</ambient><diffuse>%s</diffuse></material></visual>'%(colour,lx,rgb,rgb))
        for lx in [-.32,.32]:
            parts.append('<visual name="leg_%s"><pose>%f 0 .17 0 0 0</pose><geometry><box><size>.025 .025 .34</size></box></geometry></visual><collision name="leg_%s"><pose>%f 0 .17 0 0 0</pose><geometry><box><size>.025 .025 .34</size></box></geometry></collision>'%(lx,lx,lx,lx))
        offset = float(SIGNAL_CYCLE["offsets_s"][name])
        parts.append('</link><plugin name="signal_cycle" filename="libsemifinal_signal.so"><offset>%s</offset><period>%s</period><red>%s</red><green>%s</green></plugin></model>' % (offset, SIGNAL_CYCLE["period_s"], SIGNAL_CYCLE["red_s"], SIGNAL_CYCLE["green_s"]));world.extend(parts)
    world.append('<gui fullscreen="0"><camera name="user_camera">'
                 '<pose frame="">%f %f %f 0 %f %f</pose>'
                 '<view_controller>orbit</view_controller>'
                 '</camera></gui>' % (FIELD_M / 2.0, -2.600000, 4.200000, -0.730000, math.pi / 2.0))
    world.append('</world></sdf>')
    (PKG/"worlds").mkdir(exist_ok=True)
    write_text_lf(PKG/"worlds/official_semifinal.world", "\n".join(world) + "\n")
    save_json(PKG/"config/scene_instances_for_evaluation_only.json",instances)
    print("Generated",len(recognition),"reference assets,",len(instances),"scene instances.")


if __name__ == "__main__":
    main()
