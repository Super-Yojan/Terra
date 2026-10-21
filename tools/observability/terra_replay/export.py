"""Replay source evidence without executing any recorded command."""
from __future__ import annotations
import argparse
from dataclasses import dataclass
import hashlib
import io
import json
import math
from pathlib import Path
import re
import sqlite3
from urllib.parse import quote
import numpy as np
from PIL import Image
import rerun as rr
from .layout import default_blueprint

@dataclass
class ExportConfig:
    session: str | None = None
    rover: str | None = None
    max_events: int = 100_000
    max_source_bytes: int = 256 * 1024 * 1024
    start_ns: int | None = None
    end_ns: int | None = None

@dataclass
class ExportResult:
    path: str
    events: int
    revision: str
    missing_attachments: int


def _read_source(source: Path, config: ExportConfig):
    if source.is_dir():
        events_file = source / 'events.jsonl'
        if events_file.exists():
            if events_file.stat().st_size > config.max_source_bytes:
                raise ValueError('export source too large; choose a narrower time range')
            with events_file.open() as f:
                raw = [json.loads(line) for line in f if line.strip()]
        else:
            candidates = list(source.glob('*.db'))
            if len(candidates) != 1: raise ValueError('provide events.jsonl or one journal database')
            return _read_source(candidates[0], config)
        attachments = source / 'attachments'
    else:
        attachments = source.with_suffix('.evidence')
        # Read-only URI prevents accidental creation or modification of a journal.
        with sqlite3.connect(source.resolve().as_uri() + '?mode=ro', uri=True) as db:
            tables = {r[0] for r in db.execute("SELECT name FROM sqlite_master WHERE type='table'")}
            clauses=[];params=[]
            if config.session is not None:
                clauses.append("(json_extract(payload,'$.input.session_id')=? OR json_extract(payload,'$.input.kind')='clock.alignment')");params.append(config.session)
            if config.rover is not None:
                clauses.append("(json_extract(payload,'$.input.rover_id')=? OR json_extract(payload,'$.input.kind')='clock.alignment')");params.append(config.rover)
            for boundary,operator in [(config.start_ns, '>='),(config.end_ns, '<')]:
                if boundary is not None:
                    clauses.append("json_extract(payload,'$.input.wall_time_ns')"+operator+'?');params.append(boundary)
            where=' WHERE '+ ' AND '.join(clauses) if clauses else ''
            params.append(config.max_events+1)
            if 'observations' in tables:
                attachments = source.with_suffix('.attachments')
                rows = db.execute('SELECT payload, received_ns FROM observations'+where+' ORDER BY cursor LIMIT ?', params)
                raw=[];size=0
                for p,t in rows:
                    size+=len(p.encode())
                    if size>config.max_source_bytes:raise ValueError('export source too large; narrow the time range')
                    raw.append({'event':json.loads(p), 'received_ns':t})
            elif 'events' in tables:
                raw=[];size=0
                for (p,) in db.execute('SELECT payload FROM events'+where+' ORDER BY rowid LIMIT ?', params):
                    size+=len(p.encode())
                    if size>config.max_source_bytes:raise ValueError('export source too large; narrow the time range')
                    raw.append(json.loads(p))
            else: raise ValueError('unrecognized journal schema')
    selected = []
    for row in raw:
        event = row.get('event', row)
        data = event.get('input', {})
        if config.session is not None and data.get('session_id') != config.session and data.get('kind') != 'clock.alignment': continue
        if config.rover is not None and data.get('rover_id') != config.rover and data.get('kind') != 'clock.alignment': continue
        at = data.get('wall_time_ns', 0)
        if config.start_ns is not None and at < config.start_ns: continue
        if config.end_ns is not None and at >= config.end_ns: continue
        selected.append({'event':event, 'received_ns':row.get('received_ns'), 'association':row.get('association')})
        if len(selected) > config.max_events: raise ValueError('too many export events; narrow the time range')
    return selected, attachments


def _segment(value):
    s = quote(str(value), safe='-_')
    return s if s not in ('', '.', '..') else hashlib.sha256(str(value).encode()).hexdigest()


def _quaternion_matrix(q):
    x,y,z,w = np.asarray(q, dtype=float)
    norm = math.sqrt(x*x+y*y+z*z+w*w)
    if not math.isfinite(norm) or norm < 1e-12: raise ValueError('invalid camera quaternion')
    x,y,z,w = x/norm,y/norm,z/norm,w/norm
    return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
                     [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
                     [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])


def classify_depth(manifest, depth_bytes):
    """Classify sampled endpoints using the recorded mapper inputs, not guessed camera height."""
    k = manifest['depth_intrinsics']; w,h = int(k['width']), int(k['height'])
    if w <= 0 or h <= 0 or w*h > 4_194_304 or len(depth_bytes) != w*h*4: raise ValueError('invalid depth dimensions')
    if not all(math.isfinite(k[n]) for n in ('fx','fy','cx','cy')) or k['fx'] <= 0 or k['fy'] <= 0: raise ValueError('invalid intrinsics')
    limits = manifest['thresholds']; stride = int(limits.get('pixel_stride',4))
    if stride < 1: raise ValueError('invalid depth sampling stride')
    depth = np.frombuffer(depth_bytes, dtype='<f4').reshape(h,w)
    v,u = np.mgrid[0:h:stride,0:w:stride]; z = depth[::stride,::stride]
    good = np.isfinite(z) & (z > 0)
    optical = np.column_stack(((u[good]-k['cx'])*z[good]/k['fx'], (v[good]-k['cy'])*z[good]/k['fy'], z[good]))
    pose = manifest['camera_to_world']; rotation = _quaternion_matrix(pose['quaternion_xyzw'])
    position = np.asarray(pose['position'], dtype=float)
    if position.shape != (3,) or not np.isfinite(position).all(): raise ValueError('invalid camera position')
    points = optical @ rotation.T + position
    ground = manifest['ground_plane']; normal = np.asarray(ground['normal'],dtype=float); norm = np.linalg.norm(normal)
    if normal.shape != (3,) or not np.isfinite(normal).all() or norm < 1e-12: raise ValueError('invalid ground plane')
    heights = (points @ normal - float(ground['offset'])) / norm
    classes = (heights >= limits['min_obstacle_height']).astype(np.uint8)
    classes[(heights > limits['max_obstacle_height']) | (np.linalg.norm(optical,axis=1) > limits['max_range'])] = 2
    return points, classes


def _attachment(directory, item):
    digest = item['sha256']
    if not re.fullmatch('[0-9a-f]{64}',digest): raise ValueError('invalid attachment identity')
    path = directory / digest
    if not path.exists(): return None
    if path.resolve().parent != directory.resolve(): raise ValueError('attachment escapes evidence directory')
    if path.stat().st_size > 32*1024*1024: raise ValueError('attachment too large')
    raw = path.read_bytes()
    if len(raw) != item['byte_length'] or hashlib.sha256(raw).hexdigest() != digest: raise ValueError('attachment integrity failure')
    return raw


def _log_evidence(rec, root, manifest, directory):
    frame = _segment(manifest.get('frame_id','unknown'))
    geometry = root + '/frames/' + frame
    missing = 0
    blobs = {}
    metadata = {}
    for item in manifest.get('attachments',[]):
        raw = _attachment(directory,item)
        if raw is None:
            missing += 1
            rec.log(root+'/missing',rr.TextLog('Missing or evicted attachment: '+item.get('name','unknown'),level='WARN'))
        else: blobs[item['name']] = raw; metadata[item['name']] = item
    # Occupancy-only snapshots do not imply a new camera exposure. For a camera
    # evidence bundle, explicitly expire absent channels rather than forward-fill them.
    if manifest.get('evidence_type') != 'occupancy':
        if manifest.get('time_basis') == 'receipt_only':
            if 'rgb' not in blobs: rec.log(root+'/receipt_only/rgb',rr.Clear(recursive=True))
        else:
            if 'rgb' not in blobs:rec.log(geometry+'/camera/rgb',rr.Clear(recursive=True))
            if 'depth' not in blobs:
                for entity in ('camera/depth','classified_endpoints','ground','obstacle_min','obstacle_max'):
                    rec.log(geometry+'/'+entity,rr.Clear(recursive=True))
    if 'camera_to_world' in manifest:
        pose=manifest['camera_to_world']
        rec.log(geometry+'/camera',rr.Transform3D(translation=pose['position'],quaternion=rr.Quaternion(xyzw=pose['quaternion_xyzw'])))
    if 'rgb' in blobs:
        item=metadata['rgb']
        if item['encoding']=='rgba8':
            w,h=item['dimensions'];image=np.frombuffer(blobs['rgb'],dtype=np.uint8).reshape(h,w,4)
        else:
            with Image.open(io.BytesIO(blobs['rgb'])) as im: image=np.asarray(im.convert('RGB'))
        target=root+'/receipt_only/rgb' if manifest.get('time_basis')=='receipt_only' else geometry+'/camera/rgb/image'
        if manifest.get('time_basis') != 'receipt_only' and 'rgb_intrinsics' in manifest:
            k=manifest['rgb_intrinsics']
            rec.log(geometry+'/camera/rgb',rr.Pinhole(focal_length=[k['fx'],k['fy']],principal_point=[k['cx'],k['cy']],width=k['width'],height=k['height'],camera_xyz=rr.ViewCoordinates.RDF))
        rec.log(target,rr.Image(image))
    if 'depth' in blobs:
        k=manifest['depth_intrinsics'];pose=manifest['camera_to_world']
        rec.log(geometry+'/camera',rr.Transform3D(translation=pose['position'],quaternion=rr.Quaternion(xyzw=pose['quaternion_xyzw'])))
        rec.log(geometry+'/camera/depth',rr.Pinhole(focal_length=[k['fx'],k['fy']],principal_point=[k['cx'],k['cy']],width=k['width'],height=k['height'],camera_xyz=rr.ViewCoordinates.RDF))
        depth=np.frombuffer(blobs['depth'],dtype='<f4').reshape(k['height'],k['width'])
        rec.log(geometry+'/camera/depth/image',rr.DepthImage(depth,meter=1.0))
        points,classes=classify_depth(manifest,blobs['depth'])
        colors=np.array([[130,210,170],[240,90,60],[180,140,230]],dtype=np.uint8)[classes]
        rec.log(geometry+'/classified_endpoints',rr.Points3D(points,colors=colors,radii=0.025))
        ground=manifest['ground_plane'];normal=np.asarray(ground['normal'],dtype=float);norm=np.linalg.norm(normal);normal/=norm
        if normal[2]>0.01:
            center=np.asarray(pose['position'],dtype=float);xy=np.array([[-2,-2],[2,-2],[2,2],[-2,2],[-2,-2]])+center[:2]
            for label,above in [('ground',0),('obstacle_min',manifest['thresholds']['min_obstacle_height']),('obstacle_max',manifest['thresholds']['max_obstacle_height'])]:
                z=(ground['offset']/norm+above-xy@normal[:2])/normal[2]
                rec.log(geometry+'/'+label,rr.LineStrips3D([np.column_stack([xy,z])]))
    if 'occupancy' in blobs and 'occupancy' in manifest:
        grid=manifest['occupancy'];w,h=grid['width'],grid['height']
        values=np.frombuffer(blobs['occupancy'],dtype=np.int8).reshape(h,w)
        shades=np.where(values<0,120,255-np.clip(values,0,100)*2.55).astype(np.uint8)
        # Image rows are flipped for a top-down display; world grid coordinates remain unchanged.
        rec.log(geometry+'/occupancy_image',rr.Image(np.flipud(shades).copy()))
        rows,cols=np.nonzero(values>=0)
        xy=np.column_stack((grid['origin_x']+(cols+0.5)*grid['resolution'],grid['origin_y']+(rows+0.5)*grid['resolution']))
        points=np.column_stack((xy,np.zeros(len(xy))))
        colors=np.repeat(shades[rows,cols,None],3,axis=1)
        rec.log(geometry+'/occupancy',rr.Points3D(points,colors=colors,radii=grid['resolution']/2))
    return missing


def export_session(source_dir: Path, output_rrd: Path, config: ExportConfig) -> ExportResult:
    rows, directory = _read_source(Path(source_dir),config)
    unique = {}; selected=[]
    for row in rows:
        e=row['event'];identity=json.dumps(e['id'],sort_keys=True);body=json.dumps(e,sort_keys=True,allow_nan=False)
        digest=hashlib.sha256(body.encode()).hexdigest()
        if identity in unique:
            if unique[identity]!=digest: raise ValueError('conflicting event identity')
            continue
        unique[identity]=digest;selected.append(row)
    if not selected: raise ValueError('no events in selected session/range')
    selected.sort(key=lambda r:(r['event']['input']['wall_time_ns'],r['event']['id']['producer_id'],r['event']['id']['boot_id'],r['event']['id']['sequence']))
    revision=hashlib.sha256(json.dumps({'exporter_schema':1,'sdk_version':rr.__version__,'events':selected},sort_keys=True).encode()).hexdigest()
    output_rrd=Path(output_rrd);output_rrd.parent.mkdir(parents=True,exist_ok=True)
    index=[];missing=0;last_frames={}
    alignments={}
    for row in selected:
        e=row['event'];a=e['input'].get('attributes',{})
        if e['input']['kind']=='clock.alignment' and isinstance(a.get('offset_ns'),int):
            alignments.setdefault((e['id']['producer_id'],e['id']['boot_id']),[]).append(a)

    rec=rr.RecordingStream('terra-observability-v1',recording_id='terra-'+revision)
    roots_and_frames={}
    unpaired_roots=set()
    for row in selected:
        e=row['event'];data=e['input'];root='/rovers/'+_segment(data['rover_id'])+'/sources/'+_segment(e['id']['producer_id'])
        frames=roots_and_frames.setdefault(root,[])
        if data.get('attributes',{}).get('time_basis')=='receipt_only':
            unpaired_roots.add(root)
        elif 'attachments' in data.get('attributes',{}):
            frame=_segment(data['attributes'].get('frame_id','unknown'))
            if frame not in frames: frames.append(frame)
    rec.save(output_rrd,default_blueprint=default_blueprint(roots_and_frames,unpaired_roots))
    first=selected[0]['event']['input']['wall_time_ns']
    try:
        rec.log('/rovers',rr.ViewCoordinates.RIGHT_HAND_Z_UP,static=True)
        rec.log('/alignment',rr.TextLog('Wall-clock alignment is approximate. RTT estimates are included where measured; clock drift is unmeasured. Source timelines preserve per-producer order.',level='WARN'),static=True)
        for row in selected:
            e=row['event'];data=e['input'];identity=e['id'];attrs=data.get('attributes',{})
            root='/rovers/'+_segment(data['rover_id'])+'/sources/'+_segment(identity['producer_id'])
            rec.reset_time()
            rec.set_time('wall_clock_approx',duration=(data['wall_time_ns']-first)/1e9)
            if data['monotonic_ns'] > 0:
                rec.set_time('source_'+_segment(identity['producer_id'])+'_'+_segment(identity['boot_id']),duration=data['monotonic_ns']/1e9)
            rec.set_time('sequence_'+_segment(identity['producer_id'])+'_'+_segment(identity['boot_id']),sequence=identity['sequence'])
            if row['received_ns'] is not None:rec.set_time('receipt_time',duration=(row['received_ns']-first)/1e9)
            clock=None
            samples=alignments.get((identity['producer_id'],identity['boot_id']),[])
            if samples:
                clock=min(samples,key=lambda a:abs(a['measurement_wall_ns']-data['wall_time_ns']))
                if abs(clock['measurement_wall_ns']-data['wall_time_ns'])>60_000_000_000: clock=None
            if clock:
                rec.set_time('estimated_server_time',timestamp=np.datetime64(data['wall_time_ns']+clock['offset_ns'],'ns'))
                rec.log(root+'/metrics/clock_rtt_uncertainty_seconds',rr.Scalars(clock['uncertainty_ns']/1e9))
            event_text=(('['+row['association']+'] ') if row.get('association') else '')+data['kind']+' '+json.dumps(attrs,sort_keys=True,ensure_ascii=False)[:2000]
            rec.log(root+'/events/'+_segment(data['component']),rr.TextLog(event_text,level={'warning':'WARN','error':'ERROR','debug':'DEBUG'}.get(data['severity'],'INFO')))
            index.append({'id':identity,'kind':data['kind'],'association':row.get('association'),'wall_time_ns':data['wall_time_ns'],'monotonic_ns':data['monotonic_ns'],'received_ns':row['received_ns'],'frame_id':attrs.get('frame_id'),'estimated_server_wall_ns':data['wall_time_ns']+clock['offset_ns'] if clock else None,'clock_uncertainty_ns':clock['uncertainty_ns'] if clock else None})
            for metric in ['left_effort','right_effort','estimated_forward','estimated_yaw_rate','target_forward','target_yaw_rate','pose_age','map_age','imu_age','map_timeout','pose_timeout','command_age_ms']:
                value=attrs.get(metric)
                if isinstance(value,(int,float)) and math.isfinite(value):rec.log(root+'/metrics/'+metric,rr.Scalars(value))
            state=attrs.get('state',attrs)
            status=state.get('status',{})
            route=status.get('global_route')
            if route:rec.log(root+'/route',rr.LineStrips3D([[[p[0],p[1],0] for p in route]]))
            pose=attrs.get('pose')
            if isinstance(pose,dict) and 'x' in pose and 'y' in pose:rec.log(root+'/pose',rr.Points3D([[pose['x'],pose['y'],0]],radii=0.15))
            if 'attachments' in attrs and attrs.get('time_basis') != 'receipt_only':
                frame=_segment(attrs.get('frame_id','unknown'))
                previous=last_frames.get(root)
                if previous is not None and previous!=frame:
                    rec.log(root+'/frames/'+previous,rr.Clear(recursive=True))
                last_frames[root]=frame
            if 'attachments' in attrs:
                missing+=_log_evidence(rec,root,attrs,directory)
        rec.flush()
    finally:
        rec.disconnect()
    output_rrd.with_suffix('.events.jsonl').write_text(''.join(json.dumps(r,sort_keys=True)+'\n' for r in selected))
    output_rrd.with_suffix('.index.json').write_text(json.dumps(index,indent=2))
    output_rrd.with_suffix('.manifest.json').write_text(json.dumps({'schema_version':1,'rerun_sdk':rr.__version__,'events':len(selected),'revision':revision,'recording_id':'terra-'+revision,'missing_attachments':missing,'clock_alignment':('RTT estimates available; source timelines remain authoritative and drift is unmeasured' if alignments else 'unmeasured; wall-clock alignment is approximate'),'gaps':sum(r['event']['input']['kind']=='observability.gap' for r in selected)},indent=2))
    return ExportResult(str(output_rrd),len(selected),revision,missing)


def main():
    parser=argparse.ArgumentParser(description='Export operational evidence to a Rerun timeline (never executes commands).')
    sub=parser.add_subparsers(dest='action',required=True)
    export=sub.add_parser('export');export.add_argument('input',type=Path);export.add_argument('--output',type=Path,required=True);export.add_argument('--session');export.add_argument('--rover')
    fetch=sub.add_parser('fetch');fetch.add_argument('endpoint');fetch.add_argument('--token-file',type=Path,required=True);fetch.add_argument('--rover',required=True);fetch.add_argument('--session',required=True);fetch.add_argument('--output-dir',type=Path,required=True)
    args=parser.parse_args()
    if args.action=='fetch':
        from .client import fetch_recording
        print(fetch_recording(args.endpoint,args.token_file,args.rover,args.session,args.output_dir))
        return
    result=export_session(args.input,args.output,ExportConfig(session=args.session,rover=args.rover))
    print(json.dumps(result.__dict__))

if __name__=='__main__':main()
