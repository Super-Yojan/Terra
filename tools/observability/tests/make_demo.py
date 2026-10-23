"""Generate small synthetic sensor evidence for replay/render acceptance, not a field run."""
import hashlib,io,json,pathlib
import numpy as np
from PIL import Image
ROOT=pathlib.Path(__file__).parent/'fixtures/demo'
ROOT.mkdir(parents=True,exist_ok=True);(ROOT/'attachments').mkdir(exist_ok=True)
w,h=96,72;fx=fy=65.;cx=(w-1)/2;cy=(h-1)/2
v,u=np.mgrid[:h,:w];ground=np.full((h,w),np.nan,dtype=np.float32)
below=v>cy;ground[below]=0.5*fy/(v[below]-cy)
ground[ground>15]=np.nan
# Colocated optical RDF camera points along world +X at 0.5 m above the floor.
side=(u-cx)*2/fx;z=0.5-(v-cy)*2/fy
box=(np.abs(side)<0.3)&(z>=0)&(z<=0.7)
depth=ground.copy();depth[box]=2.0
rgb=np.zeros((h,w,3),dtype=np.uint8);rgb[:]=[118,153,183];rgb[np.isfinite(ground)]=[178,184,176];rgb[box]=[224,151,54]
image=io.BytesIO();Image.fromarray(rgb).save(image,format='PNG')
occupancy=np.full((60,60),-1,dtype=np.int8)
for iy in range(60):
 for ix in range(60):
  x=-1+(ix+.5)*.1;y=-3+(iy+.5)*.1
  if 0<x<4 and abs(y)<x*.65:occupancy[iy,ix]=0
  if 1.9<x<2.2 and abs(y)<.3:occupancy[iy,ix]=100
manifest={'exposure_ns':2_000_000_000,'rgb_exposure_ns':2_000_000_000,'frame_id':'synthetic-floor','depth_intrinsics':{'width':w,'height':h,'fx':fx,'fy':fy,'cx':cx,'cy':cy},'rgb_intrinsics':{'width':w,'height':h,'fx':fx,'fy':fy,'cx':cx,'cy':cy},'camera_to_world':{'position':[0,0,.5],'quaternion_xyzw':[-.5,.5,-.5,.5]},'ground_plane':{'normal':[0,0,1],'offset':0},'clearance':.5,'thresholds':{'min_obstacle_height':.15,'max_obstacle_height':2.,'max_range':15.,'pixel_stride':2},'occupancy':{'width':60,'height':60,'origin_x':-1.,'origin_y':-3.,'resolution':.1},'attachments':[],'integration_result':'accepted','fixture':True}
for name,data,encoding,dimensions,units in [('rgb',image.getvalue(),'png',[w,h],'rgb'),('depth',depth.astype('<f4').tobytes(),'f32le',[w,h],'metres'),('occupancy',occupancy.tobytes(),'i8',[60,60],'occupancy_percent')]:
 digest=hashlib.sha256(data).hexdigest();(ROOT/'attachments'/digest).write_bytes(data)
 manifest['attachments'].append({'name':name,'sha256':digest,'encoding':encoding,'dimensions':dimensions,'units':units,'byte_length':len(data)})
events=[]
def add(kind,at,attrs):
 seq=len(events)+1
 events.append({'event':{'schema_version':1,'id':{'producer_id':'demo-phone','boot_id':'demo-boot','sequence':seq},'input':{'rover_id':'4','session_id':'synthetic-demo','component':'demo','kind':kind,'severity':'warning' if kind=='motion.pause' else 'info','wall_time_ns':1_792_600_000_000_000_000+int(at*1e9),'monotonic_ns':int(at*1e9),'cause_id':None,'attributes':attrs}},'received_ns':1_792_600_000_000_000_000+int((at+.05)*1e9)})
for i in range(20):
 at=1+i*.1;held=i>=10
 add('control.output',at,{'left_effort':0 if held else .2,'right_effort':0 if held else .22,'estimated_forward':0 if held else .12,'target_forward':0 if held else .15,'map_age':.1,'pose_age':.03})
 if i==10:
  add('mapping.evidence',at,manifest)
  add('motion.pause',at,{'reason':'heading_requires_spin_or_replan','fixture':True})
(ROOT/'events.jsonl').write_text(''.join(json.dumps(e)+'\n' for e in events))
(ROOT/'README.md').write_text('Synthetic floor/box and motion-pause evidence for software replay acceptance. Not physical rover data. Regenerate with tests/make_demo.py.\n')
