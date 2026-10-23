"""Authenticated collector downloads. Tokens stay out of URLs, commands and output."""
import json
from pathlib import Path
import re
import time
import urllib.request
from urllib.parse import urlparse

class NoRedirect(urllib.request.HTTPRedirectHandler):
    def redirect_request(self, req, fp, code, msg, headers, newurl):
        return None


def fetch_recording(endpoint: str, token_file: Path, rover: str, session: str, directory: Path, timeout: float = 240) -> Path:
    parsed=urlparse(endpoint)
    if not (parsed.scheme=='https' or (parsed.scheme=='http' and parsed.hostname in ('127.0.0.1','localhost','::1'))):raise ValueError('HTTPS endpoint required')
    if parsed.username or parsed.password or parsed.query or parsed.fragment:raise ValueError('invalid collector endpoint')
    token=Path(token_file).read_text().strip()
    if not token:raise ValueError('empty reader credential')
    opener=urllib.request.build_opener(NoRedirect)
    endpoint=endpoint.rstrip('/')
    def request(path, payload=None):
        raw=None if payload is None else json.dumps(payload).encode()
        req=urllib.request.Request(endpoint+path,data=raw,headers={'authorization':'Bearer '+token,'content-type':'application/json'})
        with opener.open(req,timeout=30) as r:return json.loads(r.read(1024*1024))
    job=request('/v1/exports',{'rover_id':str(rover),'session_id':session})
    identity=job['id']
    if not re.fullmatch('[0-9a-f]{32}',identity):raise ValueError('invalid export identity')
    deadline=time.monotonic()+timeout
    while True:
        status=request('/v1/exports/'+identity)
        if status['status']=='complete':break
        if status['status']=='failed':raise RuntimeError('Collector export failed; inspect exporter installation and source limits')
        if time.monotonic()>=deadline:raise TimeoutError('collector export timeout')
        time.sleep(.25)
    directory=Path(directory);directory.mkdir(parents=True,exist_ok=True)
    limits={'recording.rrd':2*1024*1024*1024,'recording.events.jsonl':256*1024*1024,'recording.index.json':64*1024*1024,'recording.manifest.json':1024*1024}
    for name,limit in limits.items():
        req=urllib.request.Request(endpoint+'/v1/exports/'+identity+'/'+name,headers={'authorization':'Bearer '+token})
        temporary=directory/(name+'.part')
        try:
            with opener.open(req,timeout=60) as response, temporary.open('wb') as file:
                size=0
                while chunk:=response.read(64*1024):
                    size+=len(chunk)
                    if size>limit:raise ValueError('export artifact exceeds download limit')
                    file.write(chunk)
            temporary.replace(directory/name)
        finally:
            temporary.unlink(missing_ok=True)
    return directory/'recording.rrd'
