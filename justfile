# Build and deploy the Raspberry Pi rover (hardware/raspberry-pi).
# Override the target with `just pi=pi@rover2.local deploy` or TERRA_PI=...
# The SSH key defaults to TerraPhone's pi-admin key; override with TERRA_PI_KEY.

set shell := ["bash", "-euo", "pipefail", "-c"]

pi := env("TERRA_PI", "pi@192.168.77.1")
ssh_key := env("TERRA_PI_KEY", env("HOME", "") + "/Library/Application Support/Terra/pi-admin/id_ed25519")
ssh_opts := if path_exists(ssh_key) == "true" { "-i '" + ssh_key + "'" } else { "" }
version := `sed -n 's/^version = "\(.*\)"/\1/p' hardware/raspberry-pi/pyproject.toml`
release := "terra-rover-" + version + "-linux-arm64"
dist := "hardware/raspberry-pi/dist"
remote_dir := "/tmp/terra-rover-deploy"

default:
    @just --list

# Run the rover's Python tests on this machine.
test:
    cd hardware/raspberry-pi && uv run --with pytest python -m pytest tests

# Compile the ARM64 release in Docker (same as CI).
build:
    docker buildx build --platform linux/arm64 \
        -f packaging/rover/Dockerfile \
        --output type=local,dest={{dist}} .
    cd {{dist}} && shasum -a 256 -c {{release}}.tar.gz.sha256

# Copy the release to the Pi, stop the service, and run its installer.
[confirm("Isolate actuator power before the rover service is stopped. Continue? (y/N)")]
deploy:
    test -f {{dist}}/{{release}}.tar.gz || { echo "Missing {{release}}; run 'just build' first" >&2; exit 1; }
    ssh {{ssh_opts}} {{pi}} 'rm -rf {{remote_dir}} && mkdir -p {{remote_dir}}'
    scp {{ssh_opts}} {{dist}}/{{release}}.tar.gz {{dist}}/{{release}}.tar.gz.sha256 {{pi}}:{{remote_dir}}/
    ssh {{ssh_opts}} -t {{pi}} 'set -e; cd {{remote_dir}} \
        && sha256sum -c {{release}}.tar.gz.sha256 \
        && tar -xzf {{release}}.tar.gz \
        && sudo systemctl stop terra-rover.service \
        && sudo ./{{release}}/install.sh \
        && override=/etc/systemd/system/terra-rover.service.d/source-runtime.conf \
        && if [ -f $override ]; then sudo mv $override $override.disabled && sudo systemctl daemon-reload; fi \
        && sync \
        && terra-rover --check-bundle'
    @echo "Installed {{release}} on {{pi}}. The service is stopped; run 'just start' when ready."

# Build, then deploy.
ship: build deploy

# Start the rover service (does not arm).
start:
    ssh {{ssh_opts}} -t {{pi}} 'sudo systemctl start terra-rover.service && systemctl --no-pager status terra-rover.service'

# Stop the rover service. Isolate actuator power first.
stop:
    ssh {{ssh_opts}} -t {{pi}} 'sudo systemctl stop terra-rover.service'

status:
    ssh {{ssh_opts}} {{pi}} 'systemctl --no-pager status terra-rover.service' || true

# Follow the rover service logs.
logs:
    ssh {{ssh_opts}} -t {{pi}} 'journalctl -u terra-rover.service -f'
