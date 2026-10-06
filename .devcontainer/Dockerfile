FROM mcr.microsoft.com/devcontainers/base:ubuntu-24.04

USER root
ENV DEBIAN_FRONTEND=noninteractive

# Bevy/winit system libraries, Mesa's software Vulkan driver (lavapipe),
# and the Python toolchain for simulator/tools/zenoh_client.py.
RUN apt-get update \
    && apt-get install -y --no-install-recommends \
        pkg-config \
        build-essential \
        clang \
        lld \
        cmake \
        git \
        git-lfs \
        ca-certificates \
        curl \
        xz-utils \
        python3 \
        python3-venv \
        python3-dev \
        libx11-dev \
        libx11-xcb-dev \
        libxcb1-dev \
        libxcursor-dev \
        libxrandr-dev \
        libxi-dev \
        libxkbcommon-dev \
        libxkbcommon-x11-dev \
        libwayland-dev \
        libvulkan-dev \
        libasound2-dev \
        libudev-dev \
        mesa-vulkan-drivers \
        mesa-utils \
        libgl1-mesa-dri \
        vulkan-tools \
        x11-utils \
        fonts-dejavu-core \
    && rm -rf /var/lib/apt/lists/*

# Point the Vulkan loader at lavapipe only. A relative library_path inside a
# symlinked ICD manifest is resolved from the symlink directory, so rewrite it
# to an absolute path at a stable location.
RUN python3 - <<'PY'
import json
import pathlib

matches = sorted(pathlib.Path("/usr/share/vulkan/icd.d").glob("lvp_icd*.json"))
if len(matches) != 1:
    raise SystemExit(f"expected one lavapipe ICD, found {matches}")
data = json.loads(matches[0].read_text())
library = data["ICD"]["library_path"]
if not library.startswith("/"):
    found = None
    for base in ("/usr/lib/x86_64-linux-gnu", "/usr/lib", "/lib/x86_64-linux-gnu"):
        candidate = pathlib.Path(base) / library
        if candidate.exists():
            found = str(candidate)
            break
    if found is None:
        raise SystemExit(f"cannot resolve lavapipe library {library}")
    data["ICD"]["library_path"] = found
out = pathlib.Path("/etc/vulkan/terra-lvp.json")
out.parent.mkdir(parents=True, exist_ok=True)
out.write_text(json.dumps(data, indent=2) + "\n")
print(f"lavapipe ICD {matches[0]} -> {out}")
print(out.read_text())
PY

# Bevy opens the default ALSA device at startup. A null PCM keeps that from
# failing on a container with no sound card.
RUN printf '%s\n' \
        'pcm.!default {' \
        '    type null' \
        '}' \
        'ctl.!default {' \
        '    type null' \
        '}' \
        > /etc/asound.conf

COPY simulator/tools/requirements.txt /tmp/terra-requirements.txt
RUN python3 -m venv /opt/terra-venv \
    && /opt/terra-venv/bin/pip install --upgrade pip \
    && /opt/terra-venv/bin/pip install --no-cache-dir -r /tmp/terra-requirements.txt \
    && ln -sfn /opt/terra-venv/bin/python /usr/local/bin/python \
    && ln -sfn /opt/terra-venv/bin/python /usr/local/bin/python3 \
    && ln -sfn /opt/terra-venv/bin/pip /usr/local/bin/pip \
    && ln -sfn /opt/terra-venv/bin/pip /usr/local/bin/pip3 \
    && chown -R vscode:vscode /opt/terra-venv

ENV PATH="/opt/terra-venv/bin:${PATH}" \
    WGPU_BACKEND=vulkan \
    WGPU_POWER_PREFERENCE=low_power \
    VK_ICD_FILENAMES=/etc/vulkan/terra-lvp.json \
    VK_DRIVER_FILES=/etc/vulkan/terra-lvp.json \
    LIBGL_ALWAYS_SOFTWARE=1 \
    GALLIUM_DRIVER=llvmpipe

RUN vulkaninfo --summary
