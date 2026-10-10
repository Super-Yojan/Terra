import json
from pathlib import Path
import tempfile
import unittest
from terra_rover.backend import MockBackend
from terra_rover.configuration import ConfigurationStore
from terra_rover.safety import SafetyController
from terra_rover.protocol import FragmentAssembler, ProtocolError, fragment_message, encode_control, encode_reply


def actuator(identifier=0, port='P0'):
    return dict(id=identifier, name='ESC', port=port, kind='bidirectional_esc', inverted=False,
                limits=dict(min=-1, max=1), calibration=dict(type='bidirectional_esc', reverse_us=1000,
                neutral_us=1500, forward_us=2000, arming_duration_ms=2000), route=dict(type='left_effort'), safe=dict(type='zero'))

class PaginationTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(); self.addCleanup(self.temp.cleanup)
        self.backend = MockBackend(('P0', 'P1')); self.safety = SafetyController(self.backend.capabilities())
        self.store = ConfigurationStore(Path(self.temp.name)/'layout.json', self.safety, self.backend)
        self.store.load(0); self.store.begin_connection(123, 0); self.request = 0
    def call(self, operation, **payload):
        self.request += 1
        return self.store.handle(dict(schema_version=1, request_id=self.request, operation=operation, payload=payload), .01)
    def begin(self):
        reply = self.call('begin_layout_edit', expected_revision=self.store.active_revision, mode='replace')
        self.assertEqual(reply['result'], 'ok', reply); return reply['payload']
    def stage(self, draft, entry):
        reply = self.call('stage_actuator', edit_token=draft['edit_token'], edit_version=draft['edit_version'], actuator=entry)
        self.assertEqual(reply['result'], 'ok', reply); draft.update(reply['payload']); return draft
    def commit(self, draft):
        self.assertEqual(self.call('validate_layout_edit', edit_token=draft['edit_token'], edit_version=draft['edit_version'])['result'], 'ok')
        return self.call('commit_layout_edit', **{key:draft[key] for key in ('edit_token','edit_version','base_revision')})
    def test_lazy_reads_and_replayed_atomic_commit(self):
        self.assertEqual(self.call('layout_index')['payload']['actuators'], [])
        self.assertIn('actuator_pagination_v1', self.call('capabilities')['payload']['features'])
        draft = self.stage(self.begin(), actuator()); reply = self.commit(draft)
        self.assertEqual(reply['payload'], {'revision':1}); self.assertEqual(json.loads(self.store.path.read_text())['revision'], 1)
        envelope = dict(schema_version=1,request_id=self.request,operation='commit_layout_edit',payload={key:draft[key] for key in ('edit_token','edit_version','base_revision')})
        self.assertEqual(self.store.handle(envelope,.02), reply)
        index = self.call('layout_index')['payload']; self.assertEqual(index['actuators'], [dict(id=0,name='ESC',kind='bidirectional_esc')])
        self.assertEqual(self.call('read_actuator',expected_revision=1,actuator_id=0)['payload']['actuator'], actuator())
        profile = self.call('read_drive_profile',expected_revision=1,actuator_id=0)['payload']['actuator']
        self.assertEqual(set(profile), {'id','kind','limits','route','safe'})
        self.assertEqual(self.call('read_actuator',expected_revision=0,actuator_id=0)['errors'][0]['code'],'stale_revision')
    def test_conflicts_versions_and_disconnect(self):
        draft = self.stage(self.begin(), actuator()); stale = dict(draft)
        draft = self.stage(draft, actuator(1))
        self.assertEqual(self.call('validate_layout_edit',edit_token=stale['edit_token'],edit_version=stale['edit_version'])['result'],'error')
        reply = self.call('validate_layout_edit',edit_token=draft['edit_token'],edit_version=draft['edit_version'])
        self.assertIn('resource_conflict', [e['code'] for e in reply['errors']]); self.assertIsNone(self.store.read())
        draft.update(self.call('remove_actuator',edit_token=draft['edit_token'],edit_version=draft['edit_version'],actuator_id=1)['payload'])
        self.store.end_connection(.02); self.store.begin_connection(124,.02)
        self.assertEqual(self.call('validate_layout_edit',edit_token=draft['edit_token'],edit_version=draft['edit_version'])['result'],'error')
    def test_validation_required_and_mutation_invalidates_ack(self):
        draft = self.stage(self.begin(),actuator())
        self.assertEqual(self.call('commit_layout_edit',**{key:draft[key] for key in ('edit_token','edit_version','base_revision')})['result'],'error')
    def test_legacy_stage_invalidates_draft_and_begin_invalidates_legacy(self):
        draft = self.stage(self.begin(), actuator())
        layout = dict(schema_version=1, revision=0, actuators=[actuator()])
        self.assertEqual(self.call('stage_layout', layout=layout)['result'], 'ok')
        stage_id = self.request
        self.assertEqual(self.call('validate_layout_edit', edit_token=draft['edit_token'], edit_version=draft['edit_version'])['result'], 'error')
        self.begin()
        self.assertEqual(self.call('commit_layout', staged_request_id=stage_id, staged_revision=0)['result'], 'error')
    def test_failed_stage_preserves_validated_version_and_armed_commit_rejects(self):
        draft = self.stage(self.begin(), actuator())
        self.call('validate_layout_edit', edit_token=draft['edit_token'], edit_version=draft['edit_version'])
        invalid = actuator(); invalid['limits']['min'] = 2
        self.assertEqual(self.call('stage_actuator', edit_token=draft['edit_token'], edit_version=draft['edit_version'], actuator=invalid)['result'], 'error')
        self.safety.armed = True
        payload = {key:draft[key] for key in ('edit_token','edit_version','base_revision')}
        self.assertEqual(self.call('commit_layout_edit', **payload)['errors'][0]['code'], 'configuration_armed')
        self.safety.armed = False
        self.assertEqual(self.call('commit_layout_edit', **payload)['payload'], {'revision':1})
    def test_persistence_failure_consumes_draft_and_preserves_active(self):
        self.commit(self.stage(self.begin(), actuator()))
        draft = self.stage(self.begin(), actuator(1,'P1'))
        self.call('validate_layout_edit', edit_token=draft['edit_token'], edit_version=draft['edit_version'])
        def fail(candidate): raise OSError('disk failure')
        self.store._persist = fail
        reply = self.call('commit_layout_edit', **{key:draft[key] for key in ('edit_token','edit_version','base_revision')})
        self.assertEqual(reply['result'], 'error'); self.assertIsNone(reply['payload'])
        self.assertEqual(self.store.active_revision, 1); self.assertEqual(self.store.read()['actuators'][0]['id'],0)
        self.assertEqual(self.call('validate_layout_edit', edit_token=draft['edit_token'], edit_version=draft['edit_version'])['result'], 'error')
    def test_ble_routes_use_configuration_deadline(self):
        from terra_rover.ble import BlePeripheral
        peripheral = BlePeripheral()
        data = encode_control(dict(schema_version=1, request_id=1, operation='layout_index', payload={}))
        fragments = fragment_message(data,1,20,4096)
        for i, fragment in enumerate(fragments): peripheral._handle_write('control',fragment,i*.15)
        self.assertEqual(peripheral.events[-1][1], 'json')
        peripheral._handle_write('control',fragments[0],1)
        with self.assertLogs('terra_rover.ble', level='WARNING') as logs:
            with self.assertRaises(ProtocolError): peripheral._handle_write('control',fragments[1],3.01)
        self.assertIn('message_id=1 route=json elapsed=2.010', logs.output[0])
        self.assertNotIn(1,peripheral.routes)
    def test_clock_regression_rejects_even_new_first_fragment(self):
        assembler = FragmentAssembler(4096, timeout=2)
        fragments = fragment_message(b'x'*30, 1, 20,4096)
        assembler.push(fragments[0], 1)
        with self.assertRaises(ProtocolError): assembler.push(fragments[0], .9)
        self.assertIsNone(assembler.active)
    def test_mutation_invalidates_validation_ack(self):
        draft = self.stage(self.begin(), actuator())
        self.call('validate_layout_edit', edit_token=draft['edit_token'], edit_version=draft['edit_version'])
        self.stage(draft, actuator(1,'P1'))
        reply = self.call('commit_layout_edit', **{key:draft[key] for key in ('edit_token','edit_version','base_revision')})
        self.assertEqual(reply['errors'][0]['code'],'edit_not_validated')
    def test_conservative_att_bound_rejects_oversized_request(self):
        entry = actuator()
        envelope = dict(schema_version=1,request_id=1,operation='stage_actuator',payload=dict(edit_token='x',edit_version=0,actuator=entry))
        entry['name'] = 'x' * (4081-len(json.dumps(envelope,separators=(',',':')).encode())+len(entry['name']))
        self.assertEqual(len(json.dumps(envelope,separators=(',',':')).encode()),4081)
        with self.assertRaises(ProtocolError): encode_control(envelope)
    def test_json_deadline_and_bounds(self):
        data = encode_control(dict(schema_version=1, request_id=1, operation='layout_index', payload={}))
        fragments = fragment_message(data,1,20,4096); assembler=FragmentAssembler(4096, timeout=2)
        for i, fragment in enumerate(fragments): result=assembler.push(fragment,i*.1)
        self.assertEqual(result,data)
        assembler=FragmentAssembler(); assembler.push(fragments[0],0)
        with self.assertRaises(ProtocolError): assembler.push(fragments[1],.101)
