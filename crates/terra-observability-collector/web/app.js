const $ = id => document.getElementById(id);
let token = '', sessions = [], recording = null, controller = new AbortController(), epoch = 0;
function assertCurrent(operation) { if (operation !== epoch || !token) throw new DOMException('Reader access changed', 'AbortError'); }
function setBusy(busy) { for (const id of ['rover','session','start','end','open','refresh']) $(id).disabled = busy; }
const status = message => { $('status').textContent = message; };
function clearRecording() {
  $('viewer').removeAttribute('src'); $('playback').hidden = true;
  if (recording) URL.revokeObjectURL(recording);
  recording = null; $('download').disabled = true;
}
function signOut() {
  epoch++; controller.abort(); controller = new AbortController(); token = ''; sessions = [];
  $('token').value = ''; $('controls').hidden = true; $('access').hidden = false;
  clearRecording(); setBusy(false); status('Signed out. Reader access was cleared from this tab.');
}
async function api(path, options = {}) {
  const response = await fetch('/observability' + path, {
    ...options, signal: controller.signal, cache: 'no-store', credentials: 'omit',
    headers: { Authorization: 'Bearer ' + token, ...(options.body ? {'Content-Type': 'application/json'} : {}) }
  });
  if (!response.ok) {
    if (response.status === 401 || response.status === 403) throw new Error('Reader access was denied. Sign out and enter an authorized reader token.');
    if (response.status === 429) throw new Error('The collector is busy. Wait a moment and try again.');
    throw new Error(`The request failed (${response.status}). Try a smaller time range or refresh the runs.`);
  }
  return response;
}
const option = (value, label) => new Option(label, value);
function chooseRover() {
  clearRecording(); $('session').replaceChildren();
  for (const [index, row] of sessions.entries()) {
    if (row.rover_id !== $('rover').value) continue;
    $('session').add(option(String(index), `${row.session_id ?? 'Device events without a mission'} · ${row.count.toLocaleString()} events`));
  }
  $('open').disabled = !$('session').options.length;
}
async function refresh() {
  const operation = epoch;
  const rows = await (await api('/v1/sessions')).json();
  assertCurrent(operation); sessions = rows.filter(row => row.rover_id !== '_producer');
  sessions.sort((a,b) => b.last_received_ns - a.last_received_ns);
  const previous = $('rover').value;
  $('rover').replaceChildren(...[...new Set(sessions.map(s => s.rover_id))].sort().map(id => option(id, `Rover ${id}`)));
  if ([...$('rover').options].some(o => o.value === previous)) $('rover').value = previous;
  chooseRover(); status(sessions.length ? 'Choose a run and time range, then open its timeline.' : 'No recordings are available for this reader.');
}
function localTime(date) { return new Date(date.getTime() - date.getTimezoneOffset()*60000).toISOString().slice(0,16); }
$('start').value = localTime(new Date(Date.now()-3600000));
$('end').value = localTime(new Date(Date.now()+60000));
$('login').addEventListener('submit', async event => {
  event.preventDefault(); epoch++; controller.abort(); controller = new AbortController(); token = $('token').value.trim(); status('Loading recordings…');
  try { await refresh(); $('token').value = ''; $('access').hidden = true; $('controls').hidden = false; }
  catch (error) { if (error.name !== 'AbortError') status(error.message); }
});
$('logout').addEventListener('click', signOut);
$('rover').addEventListener('change', chooseRover);
$('session').addEventListener('change', clearRecording);
$('refresh').addEventListener('click', async () => { const operation = epoch; setBusy(true); try { await refresh(); } catch(error) { if (error.name !== 'AbortError') status(error.message); } finally { if(operation === epoch) { setBusy(false); $('open').disabled = !sessions.length; } } });
$('open').addEventListener('click', async () => {
  const operation = epoch;
  const row = sessions[Number($('session').value)];
  if (!row) return;
  const start = new Date($('start').value).getTime(), end = new Date($('end').value).getTime();
  if (!Number.isFinite(start) || !Number.isFinite(end) || start >= end) { status('Choose a valid start and end time.'); return; }
  if (row.session_id === null && end-start > 86400000) { status('Choose a window of 24 hours or less for device events.'); return; }
  setBusy(true); clearRecording();
  try {
    status('Preparing the recording. Captured evidence remains read-only.');
    let job = await (await api('/v1/exports', {method:'POST',body:JSON.stringify({rover_id:row.rover_id,session_id:row.session_id,start_ns:start*1e6,end_ns:end*1e6})})).json();
    const deadline = Date.now()+180000;
    while (job.status === 'running' && Date.now() < deadline) {
      await new Promise(resolve => setTimeout(resolve,1500));
      assertCurrent(operation);
      job = await (await api('/v1/exports/'+encodeURIComponent(job.id))).json();
    }
    assertCurrent(operation);
    if (job.status !== 'complete') throw new Error(job.error || 'Recording preparation timed out. Refresh and try again.');
    status('Loading the Rerun timeline…');
    const response = await api('/v1/exports/'+encodeURIComponent(job.id)+'/recording.rrd');
    const bytes = await response.blob();
    assertCurrent(operation);
    recording = URL.createObjectURL(bytes);
    $('viewer').src = '/rerun/viewer/?url='+encodeURIComponent(recording)+'&theme=light&persist=false';
    $('playback').hidden = false; $('download').disabled = false;
    status('Recording ready. Scrub the timeline below, or download it for the Rerun desktop viewer.');
  } catch(error) { if (error.name !== 'AbortError') status(error.message); }
  finally { if (operation === epoch) { setBusy(false); $('open').disabled = !token; } }
});
$('download').addEventListener('click', () => {
  if (!recording) return;
  const link = document.createElement('a'); link.href = recording; link.download = 'recording.rrd'; link.click();
});
window.addEventListener('pagehide', () => { controller.abort(); token = ''; clearRecording(); });
