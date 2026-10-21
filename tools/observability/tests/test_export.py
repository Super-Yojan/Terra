import hashlib, json
from pathlib import Path
import numpy as np
from rerun.chunk import RrdReader
from terra_replay.export import ExportConfig, export_session, classify_depth

def event(seq, kind='control.output', attributes=None, frame='boot'):
 return {'schema_version':1,'id':{'producer_id':'phone','boot_id':frame,'sequence':seq},'input':{'rover_id':'4','session_id':'run','component':'control','kind':kind,'severity':'info','wall_time_ns':1_700_000_000_000_000_000+seq*1_000_000_000,'monotonic_ns':seq*1_000_000_000,'cause_id':None,'attributes':attributes or {'left_effort':0.2,'reason':'active'}}}

def source(tmp_path,events):
 d=tmp_path/'source';d.mkdir(exist_ok=True);(d/'attachments').mkdir(exist_ok=True)
 (d/'events.jsonl').write_text(''.join(json.dumps(e)+'\n' for e in events));return d

def test_recording_preserves_source_receipt_and_reset_boundaries(tmp_path):
 a=event(1);b=event(2,frame='second-boot')
 d=source(tmp_path,[{'event':b,'received_ns':1_700_000_009_000_000_000},{'event':a,'received_ns':1_700_000_010_000_000_000}])
 out=tmp_path/'run.rrd';result=export_session(d,out,ExportConfig())
 summary=RrdReader(out).stream().collect().summary()
 assert result.events==2
 assert 'source_phone_boot' in summary and 'source_phone_second-boot' in summary
 assert 'receipt_time' in summary
 index=json.loads(out.with_suffix('.index.json').read_text());assert {r['id']['boot_id'] for r in index}=={'boot','second-boot'}
 assert json.loads(out.with_suffix('.manifest.json').read_text())['clock_alignment']=='unmeasured; wall-clock alignment is approximate'

def test_late_event_changes_revision_without_rewriting_ids(tmp_path):
 d=source(tmp_path,[event(1)]);out=tmp_path/'one.rrd';a=export_session(d,out,ExportConfig())
 original=json.loads(out.with_suffix('.index.json').read_text())[0]['id']
 source(tmp_path,[event(1),event(2)]);b=export_session(d,tmp_path/'two.rrd',ExportConfig())
 assert a.revision!=b.revision
 assert json.loads((tmp_path/'two.index.json').read_text())[0]['id']==original

def test_missing_image_is_explicit_not_replaced(tmp_path):
 d=source(tmp_path,[event(1,'mapping.evidence',{'exposure_ns':1000,'frame_id':'local','attachments':[{'name':'rgb','sha256':'a'*64,'encoding':'jpeg','dimensions':[2,2],'units':'rgb','byte_length':10}]})])
 out=tmp_path/'missing.rrd';result=export_session(d,out,ExportConfig());assert result.missing_attachments==1
 assert 'missing' in RrdReader(out).stream().collect().summary()

def test_ground_classification_uses_plane_and_camera_pose():
 m={'depth_intrinsics':{'width':1,'height':1,'fx':1,'fy':1,'cx':0,'cy':0},'camera_to_world':{'position':[0,0,0.5],'quaternion_xyzw':[0,0,0,1]},'ground_plane':{'normal':[0,0,1],'offset':0},'thresholds':{'min_obstacle_height':0.15,'max_obstacle_height':2,'max_range':15,'pixel_stride':1}}
 p,c=classify_depth(m,np.array([1],dtype='<f4').tobytes());assert np.allclose(p,[[0,0,1.5]]);assert c.tolist()==[1]
 m['ground_plane']['offset']=1.5;p,c=classify_depth(m,np.array([1],dtype='<f4').tobytes());assert c.tolist()==[0]
 m['ground_plane']['normal']=[0,0.5,1];m['ground_plane']['offset']=1.5;p,c=classify_depth(m,np.array([1],dtype='<f4').tobytes());assert c.tolist()==[0]

def test_duplicate_conflict_rejected(tmp_path):
 a=event(1);b=event(1);b['input']['kind']='different'
 import pytest
 with pytest.raises(ValueError,match='conflicting'):
  export_session(source(tmp_path,[a,b]),tmp_path/'bad.rrd',ExportConfig())

def test_real_images_depth_ground_and_occupancy_are_in_recording(tmp_path):
 from PIL import Image
 import io
 depth=np.array([0.5,0.5,0.2,0.5],dtype='<f4').tobytes()
 image=io.BytesIO();Image.fromarray(np.zeros((2,2,3),dtype=np.uint8)).save(image,format='PNG')
 blobs={'depth':(depth,'f32le','metres'),'rgb':(image.getvalue(),'png','rgb'),'occupancy':(bytes([255,0,100,0]),'i8','occupancy_percent')}
 m={'exposure_ns':1_000_000_000,'rgb_exposure_ns':1_000_000_000,'frame_id':'local-A','depth_intrinsics':{'width':2,'height':2,'fx':2,'fy':2,'cx':0.5,'cy':0.5},'rgb_intrinsics':{'width':2,'height':2,'fx':2,'fy':2,'cx':0.5,'cy':0.5},'camera_to_world':{'position':[0,0,0.5],'quaternion_xyzw':[1,0,0,0]},'ground_plane':{'normal':[0,0,1],'offset':0},'thresholds':{'min_obstacle_height':0.15,'max_obstacle_height':2,'max_range':15,'pixel_stride':1},'occupancy':{'width':2,'height':2,'origin_x':0,'origin_y':0,'resolution':0.1},'attachments':[]}
 d=source(tmp_path,[])
 for name,(blob,encoding,units) in blobs.items():
  digest=hashlib.sha256(blob).hexdigest();(d/'attachments'/digest).write_bytes(blob)
  m['attachments'].append({'name':name,'sha256':digest,'encoding':encoding,'dimensions':[2,2],'units':units,'byte_length':len(blob)})
 source(tmp_path,[event(1,'mapping.evidence',m)]);out=tmp_path/'full.rrd';result=export_session(d,out,ExportConfig());assert result.missing_attachments==0
 summary=RrdReader(out).stream().collect().summary()
 for part in ['classified_endpoints','camera/rgb','camera/depth','occupancy','ground','obstacle_min']:assert part in summary

def test_tilted_camera_ground_and_obstacle_height():
 angle=np.pi/9
 q=[np.cos(angle/2),0,np.sin(angle/2),0]
 # Half-turn around a tilted horizontal axis: a downward-looking ray with a horizontal component.
 from terra_replay.export import _quaternion_matrix
 R=_quaternion_matrix(q);normal=np.array([0.2,0,1.0]);position=np.array([0,0,0.6]);ray=R@np.array([0,0,1]);distance=-(normal@position)/(normal@ray)
 m={'depth_intrinsics':{'width':1,'height':1,'fx':1,'fy':1,'cx':0,'cy':0},'camera_to_world':{'position':position.tolist(),'quaternion_xyzw':q},'ground_plane':{'normal':normal.tolist(),'offset':0},'thresholds':{'min_obstacle_height':0.15,'max_obstacle_height':2,'max_range':15,'pixel_stride':1}}
 _,c=classify_depth(m,np.array([distance],dtype='<f4').tobytes());assert c.tolist()==[0]
 obstacle_distance=(0.3*np.linalg.norm(normal)-normal@position)/(normal@ray)
 _,c=classify_depth(m,np.array([obstacle_distance],dtype='<f4').tobytes());assert c.tolist()==[1]

def test_measured_offset_is_preserved_with_uncertainty(tmp_path):
 clock=event(0,'clock.alignment',{'offset_ns':2_000_000_000,'uncertainty_ns':10_000_000,'measurement_wall_ns':1_700_000_001_000_000_000})
 d=source(tmp_path,[clock,event(1)]);out=tmp_path/'clock.rrd';export_session(d,out,ExportConfig())
 index=json.loads(out.with_suffix('.index.json').read_text());point=next(r for r in index if r['kind']=='control.output')
 assert point['estimated_server_wall_ns']==point['wall_time_ns']+2_000_000_000
 assert point['clock_uncertainty_ns']==10_000_000
 assert 'estimated_server_time' in RrdReader(out).stream().collect().summary()


def test_sqlite_selection_is_applied_before_event_limit(tmp_path):
 import sqlite3
 database=tmp_path/'events.db'
 with sqlite3.connect(database) as db:
  db.execute('CREATE TABLE events(payload TEXT)')
  for i in range(5):
   value=event(i);value['input']['session_id']='old'
   db.execute('INSERT INTO events VALUES (?)',(json.dumps(value),))
  value=event(6);value['input']['session_id']='wanted'
  db.execute('INSERT INTO events VALUES (?)',(json.dumps(value),))
 from terra_replay.export import _read_source
 rows,_=_read_source(database,ExportConfig(session='wanted',max_events=2))
 assert len(rows)==1 and rows[0]['event']['input']['session_id']=='wanted'
