import unittest, tempfile, pathlib
from terra_rover.observability import ObservationJournal, set_journal
from terra_rover.safety import SafetyController
class ObservabilityTests(unittest.TestCase):
 def test_emergency_command_records_identity_and_resulting_safety(self):
  from terra_rover.protocol import CommandFrame, CommandKind
  with tempfile.TemporaryDirectory() as d:
   j=ObservationJournal(pathlib.Path(d)/'events.db','pi','0');set_journal(j)
   try:
    s=SafetyController();s.session=7;s.layout={'revision':2,'actuators':[]};s.last_time=1
    frame=CommandFrame(CommandKind.EMERGENCY_STOP,7,2,14,())
    self.assertTrue(s.accept(frame,1.01,1.01));j.flush()
    event=next(e for e in j.pending() if e['input']['kind']=='command.accepted')
    a=event['input']['attributes']
    self.assertEqual(a['session'],7);self.assertEqual(a['revision'],2)
    self.assertEqual(a['sequence'],14);self.assertTrue(a['emergency_stop'])
    self.assertFalse(a['armed']);self.assertEqual(a['stop_reason'],'emergency_stop')
   finally:set_journal(None);j.close()
 def test_pi_backlog_survives_without_phone(self):
  with tempfile.TemporaryDirectory() as d:
   path=pathlib.Path(d)/'events.db'
   j=ObservationJournal(path,'pi','4')
   self.assertTrue(j.record({'component':'pi','kind':'lifecycle.boot','attributes':{}}));j.flush();j.close()
   j=ObservationJournal(path,'pi','4');self.assertEqual(len(j.pending()),1);j.close()
 def test_disarm_transition_logged_once(self):
  with tempfile.TemporaryDirectory() as d:
   j=ObservationJournal(pathlib.Path(d)/'events.db','pi','4');set_journal(j)
   try:
    s=SafetyController();s.armed=True;s.disarm('disconnect',1);s.disarm('disconnect',2);j.flush()
    events=[e for e in j.pending() if e['input']['kind']=='motion.disarm' and e['input']['attributes']['reason']=='disconnect']
    self.assertEqual(len(events),1);self.assertTrue(events[0]['input']['attributes']['previous_armed'])
   finally:set_journal(None);j.close()
 def test_observer_failure_does_not_change_outputs(self):
  class Broken:
   def record(self,event):raise OSError('disk failed')
  s=SafetyController();set_journal(Broken())
  try:s.disarm('operator',1);self.assertFalse(s.armed);self.assertEqual(s.reason,'operator')
  finally:set_journal(None)
if __name__=='__main__':unittest.main()

class JournalProjectionTests(unittest.TestCase):
 def test_cursor_identity_is_stable_and_units_are_allowlisted(self):
  from terra_rover.log_forwarder import project_journal
  value={'_SYSTEMD_UNIT':'bluetooth.service','__CURSOR':'stable-cursor','__REALTIME_TIMESTAMP':'123','__MONOTONIC_TIMESTAMP':'50','MESSAGE':'disconnected'}
  a=project_journal(value);b=project_journal(value)
  self.assertEqual(a['source_cursor'],b['source_cursor']);self.assertEqual(a['wall_time_ns'],123000)
  value['_SYSTEMD_UNIT']='unrelated.service';self.assertIsNone(project_journal(value))

class ObservationAdmissionTests(unittest.TestCase):
 def test_invalid_frame_stays_rejected_without_observer_exception(self):
  self.assertFalse(SafetyController().accept(object(),1,1))
