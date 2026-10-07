"""Exercise installation into an image root without changing the host."""
import hashlib
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[3]
INSTALLER = ROOT / 'packaging/rover/install.sh'


class InstallationTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.directory = Path(self.temporary.name)
        self.bundle = self.directory / 'release'
        self.bundle.mkdir()
        # ELF64, little endian, executable, AArch64. Staging never executes it.
        self.binary = b'\x7fELF\x02\x01\x01' + bytes(9) + b'\x02\x00\xb7\x00' + bytes(44)
        (self.bundle / 'terra-rover').write_bytes(self.binary)
        for filename in ('terra-rover.service', 'terra-rover.env', 'terra-rover.conf'):
            (self.bundle / filename).write_text('release fixture\n')
        self.checksums()
        self.image = self.directory / 'image'

    def checksums(self):
        records = []
        for filename in ('terra-rover', 'terra-rover.service', 'terra-rover.env', 'terra-rover.conf'):
            digest = hashlib.sha256((self.bundle / filename).read_bytes()).hexdigest()
            records.append(f'{digest}  {filename}\n')
        (self.bundle / 'SHA256SUMS').write_text(''.join(records))

    def install(self):
        self.assertTrue(INSTALLER.is_file(), 'release installer has not been implemented')
        shutil.copy2(INSTALLER, self.bundle / 'install.sh')
        return subprocess.run(
            ['sh', str(self.bundle / 'install.sh'), '--root', str(self.image)],
            capture_output=True, text=True, timeout=15)

    def test_installs_executable_and_service_without_starting_host_service(self):
        result = self.install()
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        self.assertEqual((self.image / 'usr/local/bin/terra-rover').read_bytes(), self.binary)
        self.assertEqual((self.image / 'usr/local/bin/terra-rover').stat().st_mode & 0o777, 0o755)
        self.assertTrue((self.image / 'etc/systemd/system/terra-rover.service').is_file())
        self.assertTrue((self.image / 'etc/dbus-1/system.d/terra-rover.conf').is_file())
        self.assertEqual((self.image / 'var/lib/terra-rover').stat().st_mode & 0o777, 0o700)

    def test_upgrade_preserves_configuration_and_owner(self):
        first = self.install()
        self.assertEqual(first.returncode, 0, first.stdout + first.stderr)
        config = self.image / 'etc/terra-rover/rover.env'
        config.write_text('TERRA_PWM_PORTS=P0,P1\n')
        owner = self.image / 'var/lib/terra-rover/owner.json'
        owner.write_text('{"address":"AA:BB:CC:DD:EE:FF"}')
        name = self.image / 'var/lib/terra-rover/device-name.json'
        name.write_text('{"name":"terra-ABC123"}')
        layout = self.image / 'var/lib/terra-rover/layout.json'
        layout.write_text('{"revision":17}')
        result = self.install()
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        self.assertEqual(config.read_text(), 'TERRA_PWM_PORTS=P0,P1\n')
        self.assertEqual(owner.read_text(), '{"address":"AA:BB:CC:DD:EE:FF"}')
        self.assertEqual(name.read_text(), '{"name":"terra-ABC123"}')
        self.assertEqual(layout.read_text(), '{"revision":17}')

    def test_corrupt_binary_is_rejected_before_writing_destination(self):
        (self.bundle / 'terra-rover').write_bytes(self.binary + b'tampered')
        result = self.install()
        self.assertNotEqual(result.returncode, 0)
        self.assertFalse(self.image.exists())

    def test_wrong_architecture_is_rejected_even_with_valid_checksums(self):
        binary = bytearray(self.binary)
        binary[18:20] = b'\x3e\x00'  # x86-64
        (self.bundle / 'terra-rover').write_bytes(binary)
        self.checksums()
        result = self.install()
        self.assertNotEqual(result.returncode, 0)
        self.assertFalse(self.image.exists())

    def test_missing_service_is_rejected_before_writing_destination(self):
        (self.bundle / 'terra-rover.service').unlink()
        result = self.install()
        self.assertNotEqual(result.returncode, 0)
        self.assertFalse(self.image.exists())


if __name__ == '__main__':
    unittest.main()
