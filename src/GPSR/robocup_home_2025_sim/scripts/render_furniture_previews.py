#!/usr/bin/env python3
"""Render improved SDF furniture using VTK offline; NOT a Gazebo screenshot."""
import sys
import math
from pathlib import Path
from xml.etree import ElementTree as ET
from PIL import Image, ImageDraw, ImageFont
try:import vtk
except ImportError:sys.exit('Install VTK for preview generation')
ROOT=Path(__file__).resolve().parents[1]
WORLD=ROOT/'worlds'/'robocup_home_2025_vision.sdf'
DEST=ROOT/'docs'

def render_group(names,filename,size=1000):
    world=ET.parse(WORLD).getroot().find('world')
    names=set(names)
    models=[m for m in world.findall('model') if m.get('name') in names]
    if not models:raise RuntimeError('No furniture matches '+str(names))
    centerx=sum(float(m.findtext('pose').split()[0]) for m in models)/len(models)
    centery=sum(float(m.findtext('pose').split()[1]) for m in models)/len(models)
    ren=vtk.vtkRenderer();ren.SetBackground(.92,.94,.965)
    minz=0;maxs=[]
    for m in models:
        modelpose=list(map(float,m.findtext('pose').split()))
        # True component size/poses in model and link are used without alteration.
        for visual in m.findall('.//visual'):
            pose=list(map(float,(visual.findtext('pose') or '0 0 0 0 0 0').split()))
            geom=visual.find('geometry')
            if geom is None:continue
            box=geom.find('box');cyl=geom.find('cylinder');sphere=geom.find('sphere')
            if box is not None:
                sz=list(map(float,box.findtext('size').split()))
                source=vtk.vtkCubeSource();source.SetXLength(sz[0]);source.SetYLength(sz[1]);source.SetZLength(sz[2]); ext=max(sz)
            elif cyl is not None:
                radius=float(cyl.findtext('radius'));length=float(cyl.findtext('length'))
                source=vtk.vtkCylinderSource();source.SetRadius(radius);source.SetHeight(length);source.SetResolution(32);source.CappingOn();ext=length
            elif sphere is not None:
                radius=float(sphere.findtext('radius'));source=vtk.vtkSphereSource();source.SetRadius(radius);source.SetThetaResolution(24);source.SetPhiResolution(24);ext=radius*2
            else:continue
            mapper=vtk.vtkPolyDataMapper();mapper.SetInputConnection(source.GetOutputPort())
            actor=vtk.vtkActor();actor.SetMapper(mapper)
            c=list(map(float,(visual.findtext('material/diffuse') or '.7 .7 .7 1').split()))
            actor.GetProperty().SetColor(*c[:3]);actor.GetProperty().SetAmbient(.25);actor.GetProperty().SetDiffuse(.75)
            actor.GetProperty().SetSpecular(.12);actor.GetProperty().SetSpecularPower(30)
            # VTK cylinders are Y-axis oriented, while SDF cylinders default to Z.
            transform=vtk.vtkTransform();transform.PostMultiply()
            if cyl is not None:transform.RotateX(90)
            transform.RotateX(math.degrees(pose[3]));transform.RotateY(math.degrees(pose[4]));transform.RotateZ(math.degrees(pose[5]))
            transform.Translate(modelpose[0]+pose[0]-centerx,modelpose[1]+pose[1]-centery,modelpose[2]+pose[2])
            actor.SetUserTransform(transform);ren.AddActor(actor)
            maxs.append(ext)
    plane=vtk.vtkCubeSource();plane.SetXLength(5.3 if len(models)>1 else 3.6);plane.SetYLength(4.0 if len(models)>1 else 3.3);plane.SetZLength(.04)
    mp=vtk.vtkPolyDataMapper();mp.SetInputConnection(plane.GetOutputPort())
    act=vtk.vtkActor();act.SetMapper(mp);act.SetPosition(0,0,-.04);act.GetProperty().SetColor(.87,.84,.77);ren.AddActor(act)
    for p,intens in [((-4,-6,8),1.4),((5,3,7),.7)]:
        li=vtk.vtkLight();li.SetPosition(*p);li.SetFocalPoint(0,0,.6);li.SetIntensity(intens);ren.AddLight(li)
    cam=ren.GetActiveCamera();extent=2.1 if len(models)==1 else 2.6
    if 'fridge' in names:
        cam.SetPosition(-2.7*extent,-3.2*extent,2.0*extent)
    elif 'dishwasher' in names:
        cam.SetPosition(2.7*extent,3.2*extent,2.0*extent)
    else:
        cam.SetPosition(2.6*extent,-3.2*extent,2.0*extent)
    cam.SetFocalPoint(0,0,.58);cam.SetViewUp(0,0,1);cam.ParallelProjectionOn()
    cam.SetParallelScale(1.45 if len(models)==1 else 1.68)
    rw=vtk.vtkRenderWindow();rw.SetOffScreenRendering(1);rw.SetSize(size,size);rw.AddRenderer(ren)
    ren.ResetCameraClippingRange();rw.Render()
    capture=vtk.vtkWindowToImageFilter();capture.SetInput(rw);capture.Update()
    dest=DEST/filename
    writer=vtk.vtkPNGWriter();writer.SetFileName(str(dest));writer.SetInputConnection(capture.GetOutputPort());writer.Write()
    return dest

if __name__=='__main__':
    DEST.mkdir(parents=True,exist_ok=True)
    groups=[('bed',['bed']),('dining_furniture',['dining_table','chair_d1','chair_d2']),
            ('refrigerator',['fridge']),('dishwasher',['dishwasher'])]
    shots=[]
    for lbl,items in groups:
        print('Rendering',lbl,flush=True)
        shots.append((lbl,render_group(items,lbl+'_preview.png')))
    cell=800
    collage=Image.new('RGB',(cell*2,cell*2),(242,245,248));draw=ImageDraw.Draw(collage)
    for i,(label,file) in enumerate(shots):
        a=Image.open(file).convert('RGB').resize((cell-20,cell-68))
        x=i%2*cell;y=i//2*cell
        collage.paste(a,(x+10,y+10))
        draw.text((x+cell//2,y+cell-40),label.replace('_',' ').title(),font=ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf',24),fill=(25,43,60),anchor='mm')
    collage.save(DEST/'furniture_preview_sheet.png')


