"""Parametric, fully procedural static furniture for RoboCup@Home SDF worlds.

All dimensions in meters. Pieces use SDF box / cylinder primitives, each with
its own diffuse material. No image projection, external Gazebo meshes or Fuel.
The single conservative collision per original furniture model is retained
for Nav2 footprint compatibility, plus collision for handles on appliances.
"""
from math import pi
import xml.etree.ElementTree as ET

# Coordinates for components are local XY but *absolute* Z above the arena floor.
# The model's world Z-center is subtracted automatically in the Builder.
WOOD=(0.50,0.32,0.19); OAK=(0.67,0.49,0.30)
WALNUT=(0.35,0.23,0.15); WHITE=(0.86,0.88,0.86)
STEEL=(0.63,0.69,0.72); STEEL_DARK=(0.38,0.43,0.47)
TEXTILE=(0.21,0.37,0.51); DUVET=(0.27,0.51,0.59)
CREAM=(0.89,0.86,0.79); CHARCOAL=(0.19,0.23,0.26)

class Builder:
    def __init__(self, model):
        self.model=model;self.link=model.find('link');self.z0=float((model.findtext('pose') or '0 0 0').split()[2]); self.idx=0
        # Preserve the original, known-size furniture collision footprint. The
        # detailed visuals take over rendering; collision is intentionally not a
        # complex union to prevent robot base snagging on chair legs / handles.
        for visual in list(self.link.findall('visual')):self.link.remove(visual)
    def part(self,name,loc,size,rgb,shape='box',rotation=None,solid=False,roughness=.85):
        self.idx+=1
        v=ET.SubElement(self.link,'visual',{'name':f'{name}_{self.idx:03d}'})
        pose=[*loc[:2],loc[2]-self.z0,*((rotation or (0,0,0)))]
        ET.SubElement(v,'pose').text=' '.join(f'{x:.5f}' for x in pose)
        geo=ET.SubElement(v,'geometry')
        if shape=='cylinder':
            c=ET.SubElement(geo,'cylinder');ET.SubElement(c,'radius').text=f'{size[0]:.5f}';ET.SubElement(c,'length').text=f'{size[1]:.5f}'
        elif shape=='sphere':
            c=ET.SubElement(geo,'sphere');ET.SubElement(c,'radius').text=f'{size[0]:.5f}'
        else:ET.SubElement(ET.SubElement(geo,'box'),'size').text=' '.join(f'{x:.5f}' for x in size)
        mat=ET.SubElement(v,'material'); color=' '.join(f'{x:.4f}' for x in (*rgb,1.0))
        ET.SubElement(mat,'ambient').text=color;ET.SubElement(mat,'diffuse').text=color
        specval=0.22 if roughness<.5 else 0.045
        ET.SubElement(mat,'specular').text=f'{specval} {specval} {specval} 1'
        if solid:
            col=ET.SubElement(self.link,'collision',{'name':f'contact_{name}_{self.idx:03d}'})
            ET.SubElement(col,'pose').text=v.findtext('pose'); col.append(ET.fromstring(ET.tostring(geo)))
        return v
    def box(self,name,x,y,z,w,d,h,c,solid=False):return self.part(name,(x,y,z),(w,d,h),c,solid=solid)
    def cyl(self,name,x,y,z,r,length,c,rpy=(0,0,0),solid=False):return self.part(name,(x,y,z),(r,length),c,shape='cylinder',rotation=rpy,solid=solid)
    def round_leg(self,name,x,y,height,r=.025,base=0,c=STEEL_DARK):self.cyl(name,x,y,base+height*.5,r,height,c)
    def table(self,w,d,h,top_color=OAK,legs_color=WALNUT,leg_r=.045,edge=.10):
        self.box('table_top',0,0,h-.020,w,d,.040,top_color)
        # Border/apron under tabletop makes thin-top tables recognizable.
        self.box('front_apron',0,-d*.42,h-.105,w-.16,.035,.13,legs_color)
        self.box('back_apron',0,d*.42,h-.105,w-.16,.035,.13,legs_color)
        for x in (-w*.43,w*.43):
            self.box('side_apron',x,0,h-.105,.035,d-.14,.13,legs_color)
            for y in (-d*.40,d*.40):
                self.box('table_leg',x,y,(h-.05)*.5,leg_r,leg_r,h-.05,legs_color)
    def chair(self,back_sign=1):
        h=.47;w=.45;d=.46
        self.box('chair_seat',0,0,h,w,d,.045,WOOD)
        self.box('seat_cushion',0,-back_sign*.025,h+.032,.405,.37,.034,TEXTILE)
        for x in (-.18,.18):
            for y in (-.18,.18):self.box('chair_leg',x,y,.225,.037,.037,.45,WALNUT)
        for x in (-.186,.186):self.box('back_post',x,back_sign*.20,.65,.04,.042,.51,WALNUT)
        self.box('chair_backrest',0,back_sign*.202,.792,.405,.055,.225,WOOD)
        self.box('chair_back_cushion',0,back_sign*.163,.765,.348,.03,.162,TEXTILE)
    def cabinet(self,w,d,h,drawer_count=2,color=WALNUT,faces=OAK):
        self.box('cabinet_shell',0,0,h*.5,w,d,h,color)
        self.box('top_lip',0,0,h-.016,w+.015,d+.02,.034,faces)
        for i in range(drawer_count):
            z=h*(i+.5)/drawer_count
            self.box('drawer_face',0,-d*.5-.010,z,w*.84,.026,h/ drawer_count*.76,faces)
            self.cyl('drawer_knob',0,-d*.5-.033,z,.016,.024,STEEL,(pi/2,0,0),solid=True)

def upgrade_furniture(world):
    touched=[]
    for model in world.findall('model'):
        name=model.get('name')
        if name not in {
            'bed','bedside_table','bedroom_drawers','bookcase', 'dining_table',
            'chair_d1','chair_d2','chair_office', 'chair_living_west',
            'chair_living_south_1','chair_living_south_2',
            'dining_cabinet','living_sofa', 'coffee_table',
            'tv_stand','coat_rack','trash_living','kitchen_counter','fridge','sink',
            'dishwasher','microwave','kitchen_cupboard','trash_kitchen',
            'office_snack_cabinet','office_desk'
        }:continue
        b=Builder(model);touched.append(name)
        if name=='bed':
            # Bed long axis: x; headboard near the western bedroom wall.
            b.box('bed_frame',0,0,.215,2.05,1.22,.43,WOOD)
            b.box('bed_wood_foot',.995,0,.29,.045,1.28,.58,WALNUT)
            b.box('headboard',-.992,0,.54,.07,1.30,1.06,WALNUT)
            b.box('headboard_inlay',-.956,0,.55,.016,1.17,.53,OAK)
            b.box('mattress',.03,0,.465,1.86,1.10,.105,CREAM)
            b.box('bottom_sheet',.04,0,.515,1.83,1.09,.008,WHITE)
            # Rounded looking pillows using stacked textile forms + piping.
            for y in (-.290,.290):
                b.box('pillow_shadow',-.65,y,.549,.43,.43,.065,(.74,.78,.78))
                b.box('pillow_cover',-.65,y,.58,.37,.39,.071,WHITE)
                b.box('pillow_center',-.65,y,.621,.29,.31,.018,(.95,.94,.91))
                for cy in (-.19,.19):b.box('pillow_piping',-.65,y+cy,.583,.30,.007,.008,(.67,.75,.74))
            b.box('folded_duvet',.31,0,.546,1.17,1.08,.065,DUVET)
            b.box('duvet_cuff',-.31,0,.554,.080,1.075,.065,(.75,.84,.85))
            for y in (-.34,0,.34):b.box('duvet_stitch',.33,y,.58,1.07,.003,.004,(.19,.39,.46))
            b.box('duvet_hem',.855,0,.584,.009,1.08,.008,(.16,.33,.41))
        elif name=='bedside_table':b.cabinet(.55,.50,.62,2,OAK,CREAM)
        elif name=='bedroom_drawers':b.cabinet(1.0,.50,1.0,4,WOOD,OAK)
        elif name=='bookcase':
            b.box('left_side',-.225,0,.76,.04,1.06,1.52,OAK)
            b.box('right_side',.225,0,.76,.04,1.06,1.52,OAK)
            b.box('back',0,.50,.76,.46,.036,1.52,WALNUT)
            for z in (.08,.44,.80,1.16,1.49):b.box('shelf',0,0,z,.48,1.06,.035,OAK)
            for i,c in enumerate((TEXTILE,(.72,.33,.22),(.29,.55,.38),CREAM,(.59,.38,.61),(.72,.65,.31))):
                b.box('book_spine',0,-.42+i*.155,.58,.20,.052,.24,c)
        elif name=='dining_table':b.table(.90,.65,.745,OAK,WALNUT)
        elif name=='chair_d1':b.chair(+1)
        elif name in ('chair_d2','chair_office','chair_living_south_1',
                      'chair_living_south_2'):b.chair(-1)
        elif name=='chair_living_west':b.chair(+1)
        elif name=='dining_cabinet':
            # North-wall open shelving. Open/front side is local -Y (room centre).
            for x in (-.36,.36):b.box('shelf_side',x,0,.65,.04,.55,1.30,WOOD)
            b.box('shelf_back',0,.265,.65,.76,.02,1.30,WALNUT)
            for z in (.03,.41,.82,1.285):b.box('shelf',0,0,z,.76,.55,.028,OAK)
        elif name=='living_sofa':
            # Sofa long axis y; backrest next to left/west wall.
            b.box('sofa_base',0,0,.26,.75,2.,.35,CHARCOAL)
            b.box('back_rest',-.285,0,.67,.21,1.99,.45,TEXTILE)
            for y in (-.85,.85):b.box('sofa_arm',-.01,y,.52,.74,.22,.48,TEXTILE)
            for y in (-.56,0,.56):
                b.box('sofa_seat',.075,y,.444,.49,.53,.20,(.33,.52,.62))
                b.box('seat_piping',.34,y,.445,.006,.51,.03,(.16,.32,.45))
            for y in (-.46,.46):b.box('throw_pillow',-.09,y,.64,.12,.34,.31,CREAM)
            for y in (-.75,.75):b.box('sofa_foot',.28,y,.082,.07,.07,.15,WALNUT)
        elif name=='coffee_table':
            b.table(.77,.54,.47,OAK,STEEL_DARK,.040)
            b.box('lower_shelf',0,0,.16,.63,.40,.025,WOOD)
        elif name=='tv_stand':
            b.cabinet(1.25,.43,.55,2,CHARCOAL,WOOD)
            b.box('television_base',0,.02,.58,.34,.20,.035,STEEL_DARK)
            b.box('television_display',0,.08,.84,.88,.07,.46,(.09,.12,.16))
            b.box('tv_frame_trim',0,.035,.845,.92,.015,.48,CHARCOAL)
        elif name=='coat_rack':
            b.cyl('base',0,0,.025,.19,.05,STEEL_DARK)
            b.cyl('pole',0,0,.86,.033,1.66,WALNUT)
            for z in (1.40,1.61):
                b.box('hooks_horizontal',0,0,z,.38,.045,.035,WALNUT)
                b.box('hooks_cross',0,0,z,.045,.38,.035,WALNUT)
        elif name in ('trash_living','trash_kitchen'):
            h=.45 if name=='trash_living' else .5
            b.cyl('trash_can_body',0,0,h*.49,.155,h*.91,STEEL_DARK)
            b.cyl('open_rim',0,0,h*.97,.18,.035,STEEL,solid=True)
            b.cyl('dark_interior',0,0,h*.998,.137,.009,(.06,.08,.09))
            b.box('foot_pedal',0,-.19,.035,.10,.10,.022,CHARCOAL)
        elif name=='kitchen_counter':
            b.box('counter_base',0,0,.43,1.80,.57,.86,CREAM)
            b.box('countertop',0,0,.866,1.83,.61,.036,(.42,.48,.50))
            b.box('counter_toe_kick',0,.289,.055,1.75,.025,.09,STEEL_DARK)
            for x in (-.58,0,.58):
                b.box('cabinet_door',x,.293,.43,.49,.022,.73,WHITE)
                b.cyl('door_knob',x+.165,.323,.65,.014,.027,STEEL,(pi/2,0,0),solid=True)
        elif name=='fridge':
            # Fridge front faces negative X (center of kitchen).
            b.box('fridge_body',0,0,.87,.80,.80,1.74,WHITE)
            b.box('fridge_freezer_door',-.410,0,1.405,.024,.763,.625,(.79,.83,.84))
            b.box('fridge_lower_door',-.412,0,.545,.025,.767,1.00,(.82,.85,.86))
            b.box('freezer_gasket',-.433,0,1.089,.006,.758,.010,CHARCOAL)
            b.box('main_gasket',-.433,0,1.037,.006,.758,.013,CHARCOAL)
            # Two real protruding metallic bar handles with mounting posts.
            for z in (.79,1.41):
                b.cyl('handle_stainless',-.470,-.265,z,.013,.33,STEEL,solid=True)
                for ez in (-.125,.125):
                    b.cyl('handle_mount',-.451,-.265,z+ez,.012,.042,STEEL,(0,pi/2,0))
            b.box('fridge_base_grille',-.413,0,.075,.024,.67,.085,STEEL_DARK)
        elif name=='dishwasher':
            # Appliance front faces +Y towards central kitchen.
            b.box('washer_body',0,0,.435,.72,.59,.87,STEEL_DARK)
            b.box('washer_front_door',0,.303,.39,.69,.025,.76,(.77,.81,.83))
            b.box('washer_control_panel',0,.319,.779,.675,.020,.097,STEEL)
            b.box('washer_horizontal_handle',0,.351,.721,.38,.023,.020,(.35,.40,.45),solid=True)
            b.cyl('washer_selector_knob',-.228,.338,.779,.023,.018,STEEL_DARK,(pi/2,0,0),solid=True)
            for x in (-.08,.015,.11):b.cyl('washer_program_btn',x,.338,.778,.009,.009,CHARCOAL,(pi/2,0,0))
            b.box('washer_status_light',.28,.341,.788,.013,.009,.009,(.11,.66,.31))
            b.box('washer_toe_panel',0,.310,.065,.69,.021,.12,CHARCOAL)
        elif name=='sink':
            b.box('base_cabinet',0,0,.427,.70,.48,.854,CREAM)
            b.box('sink_countertop',0,0,.866,.72,.52,.036,(.46,.53,.55))
            b.box('basin_rim',0,0,.886,.53,.35,.023,STEEL)
            b.box('basin_dark',0,0,.896,.44,.28,.004,(.20,.27,.31))
            b.cyl('faucet_stem',0,.18,1.018,.016,.26,STEEL)
            b.box('faucet_spout',0,.09,1.143,.030,.20,.021,STEEL)
            b.cyl('tap_knob',.085,.18,.961,.026,.023,(.20,.26,.30))
        elif name=='microwave':
            b.box('micro_cabinet_base',0,0,.425,.58,.43,.85,CREAM)
            b.box('microwave_housing',0,0,.999,.58,.43,.300,STEEL_DARK)
            b.box('glass_window',-.062,.222,1.003,.395,.012,.195,(.09,.17,.21))
            b.box('window_border',-.065,.235,1.11,.41,.016,.014,STEEL)
            b.box('microwave_control_panel',.236,.225,1.003,.068,.01,.25,STEEL)
            for z in (.95,1.035,1.110):b.cyl('microwave_button',.235,.239,z,.013,.014,CHARCOAL,(pi/2,0,0))
            b.box('door_handle',.153,.242,1.001,.015,.018,.17,STEEL)
        elif name=='kitchen_cupboard':b.cabinet(.68,.63,1.4,3,OAK,CREAM)
        elif name=='office_snack_cabinet':
            # Second north-wall open shelf. Top supports 2025 object inventory.
            for x in (-.36,.36):b.box('shelf_side',x,0,.53,.04,.55,1.06,WALNUT)
            b.box('shelf_back',0,.265,.53,.76,.02,1.06,WOOD)
            for z in (.03,.34,.69,1.046):b.box('shelf',0,0,z,.76,.55,.028,OAK)
            b.box('display_top',0,0,1.055,.75,.55,.018,(.64,.49,.31))
        elif name=='office_desk':b.table(1.0,.55,.75,OAK,STEEL_DARK)
    return touched

