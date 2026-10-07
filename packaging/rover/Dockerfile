# Bookworm's glibc baseline supports Raspberry Pi OS Bookworm and newer.
FROM python:3.11-slim-bookworm AS build
RUN apt-get update && apt-get install -y --no-install-recommends \
    gcc g++ patchelf curl ca-certificates && rm -rf /var/lib/apt/lists/*
WORKDIR /src
COPY hardware/raspberry-pi/ hardware/raspberry-pi/
COPY scripts/build-rover.sh scripts/build-rover.sh
COPY packaging/ packaging/
COPY docs/hardware/INSTALL.md docs/hardware/INSTALL.md
RUN sh scripts/build-rover.sh /release

# Prove the executable and staged install work without a Python installation.
FROM debian:bookworm-slim AS verify
COPY --from=build /release/ /release/
RUN ! command -v python3 && \
    for bundle in /release/terra-rover-*-linux-arm64; do \
      "$bundle/terra-rover" --help && \
      "$bundle/terra-rover" --check-bundle && \
      sh "$bundle/install.sh" --root /image && \
      /image/usr/local/bin/terra-rover --check-bundle || exit 1; \
      mkdir -p /tmp/customer; printf '0' > /tmp/customer/button; touch /tmp/customer/led; \
      "$bundle/terra-rover" --button-pairing --owner /tmp/customer/owner.json \
        --device-name-file /tmp/customer/name.json --button-file /tmp/customer/button \
        --led-file /tmp/customer/led > /tmp/customer/output 2>&1 & \
      customer=$!; sleep 2; kill -TERM "$customer"; wait "$customer" || exit 1; \
      test -f /tmp/customer/name.json && test ! -e /tmp/customer/owner.json && \
      test "$(cat /tmp/customer/led)" = 0 || exit 1; \
    done

# Export only the installable artifacts; no container is needed on the Pi.
FROM scratch AS release
COPY --from=verify /release/ /
