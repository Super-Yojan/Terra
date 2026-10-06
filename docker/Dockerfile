FROM rust:1-bookworm AS build

RUN apt-get update && apt-get install -y --no-install-recommends \
    pkg-config build-essential cmake git ca-certificates \
    libwayland-dev libxkbcommon-dev libvulkan-dev libxcursor-dev \
    libxrandr-dev libxi-dev libx11-xcb-dev libasound2-dev libudev-dev \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /workspace
COPY . .
RUN cargo build --release --manifest-path simulator/Cargo.toml

FROM debian:bookworm-slim AS runtime

ENV DEBIAN_FRONTEND=noninteractive \
    DISPLAY=:0 \
    TERRA_DISPLAY_MODE=1600x900x24 \
    TERRA_SOFTWARE_RENDERING=1 \
    PATH=/opt/terra-venv/bin:$PATH

RUN apt-get update && apt-get install -y --no-install-recommends \
    ca-certificates python3 python3-venv python3-pip \
    libwayland-client0 libxkbcommon0 libx11-xcb1 libxcursor1 \
    libxrandr2 libxi6 libx11-6 libxext6 libxfixes3 libxrender1 \
    libxcb1 libxkbcommon-x11-0 libasound2 libudev1 \
    libvulkan1 mesa-vulkan-drivers \
    xvfb x11vnc novnc websockify x11-utils \
    && rm -rf /var/lib/apt/lists/*

RUN python3 -m venv /opt/terra-venv
COPY simulator/tools/requirements.txt /tmp/terra-requirements.txt
RUN /opt/terra-venv/bin/pip install --no-cache-dir -r /tmp/terra-requirements.txt \
    && rm /tmp/terra-requirements.txt

WORKDIR /workspace/simulator
COPY --from=build /workspace/simulator/target/release/simulator /usr/local/bin/terra-simulator
COPY --from=build /workspace/simulator/assets /usr/local/bin/assets
COPY simulator/tools ./tools

COPY docker/simulator-entrypoint.sh /usr/local/bin/terra-entrypoint
RUN chmod +x /usr/local/bin/terra-entrypoint

EXPOSE 6080 7447
ENTRYPOINT ["/usr/local/bin/terra-entrypoint"]
