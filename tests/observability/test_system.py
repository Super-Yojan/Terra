"""Real loopback collector, Pi backlog retry and Rerun export; no hardware or GCP."""
import hashlib,json,os,pathlib,secrets,socket,subprocess,sys,time,urllib.request,urllib.error
import pytest
ROOT=pathlib.Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'hardware/raspberry-pi'))
from terra_rover.observability import ObservationJournal, ObservationUploader
from rerun.chunk import RrdReader

@pytest.fixture
def collector(tmp_path):
 tokens={name:secrets.token_urlsafe(24) for name in ['phone','pi','dashboard','reader']}
 credentials=[{'token_sha256':hashlib.sha256(token.encode()).hexdigest(),'scope':{'producer_id':name,'rovers':['4','_producer'] if name=='reader' else ['4'],'read':name=='reader'}} for name,token in tokens.items()]
 path=tmp_path/'credentials.json';path.write_text(json.dumps(credentials));path.chmod(0o600)
 with socket.socket() as sock:sock.bind(('127.0.0.1',0));port=sock.getsockname()[1]
 env=dict(os.environ,TERRA_OBSERVABILITY_BIND=f'127.0.0.1:{port}',TERRA_OBSERVABILITY_DB=str(tmp_path/'collector.db'),TERRA_OBSERVABILITY_CREDENTIALS=str(path),TERRA_OBSERVABILITY_EXPORTER_PYTHON=str(ROOT/'tools/observability/.venv/bin/python'))
 log=(tmp_path/'collector.log').open('w')
 proc=subprocess.Popen([str(ROOT/'target/debug/terra-observability-collector')],env=env,stdout=log,stderr=log)
 endpoint=f'http://127.0.0.1:{port}'
 try:
  for _ in range(100):
   if proc.poll() is not None:raise RuntimeError('collector exited before readiness')
   try:
    with socket.create_connection(('127.0.0.1',port),timeout=.1):break
   except OSError:time.sleep(.05)
  else:raise TimeoutError('collector startup')
  yield endpoint,tokens
 finally:
  proc.terminate()
  try:proc.wait(timeout=10)
  except subprocess.TimeoutExpired:proc.kill();proc.wait()
  log.close()

def request(endpoint,token,path,payload=None):
 raw=None if payload is None else json.dumps(payload).encode()
 req=urllib.request.Request(endpoint+path,data=raw,headers={'authorization':'Bearer '+token,'content-type':'application/json'})
 with urllib.request.urlopen(req,timeout=30) as response:return json.loads(response.read())

def event(producer,seq,kind,base,attrs=None,rover='4'):
 return {'schema_version':1,'id':{'producer_id':producer,'boot_id':'test-boot','sequence':seq},'input':{'rover_id':rover,'session_id':'run' if rover!='_producer' else None,'component':producer,'kind':kind,'severity':'info','wall_time_ns':base+seq*10_000_000,'monotonic_ns':seq*10_000_000,'cause_id':None,'attributes':attrs or {}}}

def test_disconnect_retry_timeline_and_scope(collector,tmp_path):
 endpoint,tokens=collector;base=time.time_ns()
 clock=event('phone',0,'clock.alignment',base,{'offset_ns':100_000_000,'uncertainty_ns':5_000_000,'measurement_wall_ns':base},rover='_producer')
 pause=event('phone',1,'motion.pause',base,{'reason':'heading_requires_spin_or_replan'})
 for e in [clock,pause,pause,event('dashboard',1,'command.sent',base,{'sequence':55})]:
  request(endpoint,tokens[e['id']['producer_id']],'/v1/events',{'events':[e]})
 j=ObservationJournal(tmp_path/'pi.db','pi','4');j.record({'component':'bluetooth','kind':'connection.state','attributes':{'ready':True},'wall_time_ns':base});j.flush()
 # A real failed connection leaves the durable queue intact, then a new uploader replays it.
 uploader=ObservationUploader.__new__(ObservationUploader);uploader.journal=j;uploader.token=tokens['pi'];uploader.endpoint='http://127.0.0.1:1'
 try:
  with pytest.raises(Exception):uploader.once()
  assert len(j.pending())==1
  uploader.endpoint=endpoint;assert uploader.once()==1;assert not j.pending()
 finally:j.close()
 rows=request(endpoint,tokens['reader'],'/v1/events')['events']
 assert sum(e['event']['input']['kind']=='motion.pause' for e in rows)==1
 with pytest.raises(urllib.error.HTTPError):request(endpoint,tokens['phone'],'/v1/sessions')
 forbidden=event('phone',2,'state',base,rover='99')
 with pytest.raises(urllib.error.HTTPError):request(endpoint,tokens['phone'],'/v1/events',{'events':[forbidden]})
 job=request(endpoint,tokens['reader'],'/v1/exports',{'rover_id':'4','session_id':'run'})
 for _ in range(200):
  status=request(endpoint,tokens['reader'],'/v1/exports/'+job['id'])
  if status['status']!='running':break
  time.sleep(.05)
 assert status['status']=='complete',status
 def download(name):
  req=urllib.request.Request(endpoint+'/v1/exports/'+job['id']+'/'+name,headers={'authorization':'Bearer '+tokens['reader']})
  with urllib.request.urlopen(req,timeout=30) as r:return r.read()
 out=tmp_path/'run.rrd';out.write_bytes(download('recording.rrd'))
 assert 'events' in RrdReader(out).stream().collect().summary()
 index=json.loads(download('recording.index.json'))
 point=next(e for e in index if e['kind']=='motion.pause')
 assert point['estimated_server_wall_ns']==pause['input']['wall_time_ns']+100_000_000
 assert point['clock_uncertainty_ns']==5_000_000
 assert any(e['kind']=='connection.state' for e in index), 'unassociated Pi context should remain visible without inventing a mission ID'
 from terra_replay.client import fetch_recording
 token_file=tmp_path/'reader.token';token_file.write_text(tokens['reader'])
 fetched=fetch_recording(endpoint,token_file,'4','run',tmp_path/'download')
 assert RrdReader(fetched).recordings()
