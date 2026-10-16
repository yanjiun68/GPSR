#!/usr/bin/env python3
"""Create geometric occupancy reference for the *modified* 2025 vision world.
NOT a SLAM Toolbox/robot-generated map. Ray range and visibility effects omitted.
"""
from pathlib import Path
import math
import xml.etree.ElementTree as ET
from PIL import Image,ImageDraw,ImageFont
import yaml
ROOT=Path(__file__).resolve().parents[1]
WORLD=ROOT/'worlds'/'robocup_home_2025_vision.sdf'
MP=ROOT/'maps'
res=.05;ox=-7.;oy=-5.5;wid=280;hei=220
img=Image.new('L',(wid,hei),205);dr=ImageDraw.Draw(img)

def pixel(x,y):return (round((x-ox)/res),hei-1-round((y-oy)/res))
def rect(cx,cy,sx,sy,fill):
    a,b=pixel(cx-sx/2,cy+sy/2);c,d=pixel(cx+sx/2,cy-sy/2)
    dr.rectangle((a,b,c,d),fill=fill)
# Entire arena and modest outside strip known clear; screenshot map is ground-truth geometry.
rect(0,0,8.5,8.0,254)  # Matches the revised smaller floor model; outside remains unknown
root=ET.parse(WORLD)
notes=[]
for md in root.findall('./world/model'):
    if md.get('name')=='floor':continue
    if (md.findtext('static') or '').strip().lower()!='true':continue
    name=md.get('name')
    pose=[float(x) for x in (md.findtext('pose') or '0 0 0 0 0 0').split()]
    box=md.find('.//collision/geometry/box/size')
    if box is None:continue
    sx,sy,h=(float(x) for x in box.text.split())
    # Occupancy grid uses the AABB of yaw-rotated collision primitives.
    # This preserves sofa/chair/appliance footprints at 90/180 degree yaw.
    a,b=abs(math.cos(pose[5])),abs(math.sin(pose[5]))
    rect(pose[0],pose[1],sx*a+sy*b,sx*b+sy*a,0)
    notes.append(name)
img.save(MP/'vision_geometric_reference.pgm')
with (MP/'vision_geometric_reference.yaml').open('w') as f:
    yaml.safe_dump({'image':'vision_geometric_reference.pgm','mode':'trinary','resolution':res,'origin':[ox,oy,0.0],
      'negate':0,'occupied_thresh':0.65,'free_thresh':0.196},f,sort_keys=False)
# Full-color floorplan from occupancy image + object labels
canvas=Image.new('RGB',img.size);canvas.paste(img.convert('RGB'))
colors={'drinks':'#0580bb','food':'#e28320','snacks':'#b82671','tableware':'#13885b','extra':'#823ac7'}
js=__import__('json').loads((ROOT/'config'/'robocup_malaysia_2025_objects.json').read_text())
for item in js['items']:
    x,y,*_=item['world_pose_xyz_yaw'];u,v=pixel(x,y)
    d=ImageDraw.Draw(canvas)
    d.ellipse([u-3,v-3,u+3,v+3],fill=colors[item['category']])
canvas=canvas.resize((1120,880),Image.Resampling.NEAREST)
canvas.save(MP/'vision_geometric_reference_preview.png')
print(f'Created vision reference map with {len(notes)} static world obstacles; {wid}x{hei} at {res} m/cell')
