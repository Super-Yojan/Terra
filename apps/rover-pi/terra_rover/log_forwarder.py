"""Independent journal reader/uploader. No imports of actuator or Bluetooth control."""
import hashlib,json,os,pathlib,re,subprocess,time
from .observability import ObservationJournal,ObservationUploader

UNITS=('terra-rover.service','bluetooth.service','terra-network-setup.service',
       'hostapd.service','systemd-networkd.service','wpa_supplicant@wlan0.service',
       'NetworkManager.service','systemd-timesyncd.service')

def project_journal(entry):
    unit=next((entry.get(field) for field in ('_SYSTEMD_UNIT','UNIT','OBJECT_SYSTEMD_UNIT') if entry.get(field) in UNITS),None)
    if entry.get('_TRANSPORT')=='kernel':unit='kernel'
    cursor=entry.get('__CURSOR')
    if unit is None or not cursor:return None
    text=entry.get('MESSAGE','')
    if not isinstance(text,str):text='[non-text journal message]'
    try:
        envelope=json.loads(text)
        if isinstance(envelope,dict) and envelope.get('schema_version')==1 and isinstance(envelope.get('id'),dict):
            return {'envelope':envelope,'source_cursor':cursor}
    except (ValueError,TypeError):pass
    if (any(word in text.lower() for word in ('password','passphrase','bearer ','private key','credential','authorization:'))
            or re.search(r"(?i)\b(?:wpa[_-])?psk\b['\"]?\s*(?:[=:]|value\b)",text)):
        text='[sensitive journal message omitted]'
    try:
        wall=int(entry.get('__REALTIME_TIMESTAMP',0))*1000
        mono=int(entry.get('__MONOTONIC_TIMESTAMP',0))*1000
    except (ValueError,TypeError):return None
    return {'source_cursor':cursor,'component':unit,'kind':'service.journal','wall_time_ns':wall,
            'monotonic_ns':mono,'attributes':{'message':text[:32768],'cursor_hash':hashlib.sha256(cursor.encode()).hexdigest()}}


def main():
    root=pathlib.Path(os.environ.get('TERRA_OBSERVABILITY_STATE','/var/lib/terra-observability-pi'))
    root.mkdir(parents=True,exist_ok=True)
    journal=ObservationJournal(root/'events.db',os.environ['TERRA_OBSERVABILITY_PRODUCER'],os.environ['TERRA_OBSERVABILITY_ROVER'], max_bytes=int(os.getenv('TERRA_OBSERVABILITY_EVENT_BYTES',512*1024*1024)), retention_seconds=int(os.getenv('TERRA_OBSERVABILITY_RETENTION_SECONDS',7*86400)))
    token=pathlib.Path(os.environ['TERRA_OBSERVABILITY_UPLOAD_TOKEN_FILE']).read_text().strip()
    ObservationUploader(journal,os.environ['TERRA_OBSERVABILITY_ENDPOINT'],token)
    cursor_file=root/'journal.cursor'
    while True:
        command=['journalctl','--follow','--output=json','--no-pager']
        for unit in UNITS:
            for field in ('_SYSTEMD_UNIT','UNIT','OBJECT_SYSTEMD_UNIT'):
                if len(command)>4:command.append('+')
                command.append(f'{field}={unit}')
        command+=['+','_TRANSPORT=kernel']
        if cursor_file.exists():command+=['--after-cursor='+cursor_file.read_text().strip()]
        else:command+=['--since=-24h','--lines=5000']
        process=subprocess.Popen(command,stdout=subprocess.PIPE,stderr=subprocess.DEVNULL)
        try:
            while True:
                line=process.stdout.readline(1024*1024)
                if not line:break
                if not line.endswith(b'\n'):
                    journal.record({'component':'observability','kind':'observability.gap','attributes':{'reason':'oversized_journal_line'}})
                    break
                try:entry=json.loads(line)
                except ValueError:continue
                event=project_journal(entry)
                if event is None:continue
                if 'envelope' in event:
                    envelope=event['envelope']
                    if envelope['id'].get('producer_id')!=journal.producer:continue
                    accepted=journal.record_envelope(envelope)
                else:accepted=journal.record(event)
                if not accepted:break
                journal.flush()
                identity=event.get('envelope',{}).get('id') or {'producer_id':journal.producer,'boot_id':hashlib.sha256(event['source_cursor'].encode()).hexdigest(),'sequence':0}
                # Advance only when the original identity exists durably in the queue.
                with journal._db() as db:
                    present=db.execute('SELECT 1 FROM events WHERE id=?',(json.dumps(identity,sort_keys=True),)).fetchone()
                if not present:break
                temporary=cursor_file.with_suffix('.tmp');temporary.write_text(event['source_cursor']);temporary.replace(cursor_file)
        finally:
            process.terminate()
            try:process.wait(timeout=5)
            except subprocess.TimeoutExpired:process.kill();process.wait()
        journal.record({'component':'observability','kind':'service.journal_reconnect','attributes':{'reason':'journal_stream_ended'}})
        time.sleep(3)

if __name__=='__main__':main()
