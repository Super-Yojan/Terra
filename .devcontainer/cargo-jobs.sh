# Pick a rustc job cap when the machine is smaller than the Codespaces
# 8-core / 32 GB host requirement. Source this file; do not execute it.
if [[ -z "${CARGO_BUILD_JOBS:-}" ]] && [[ -r /proc/meminfo ]]; then
  mem_kb="$(awk '/MemTotal:/ { print $2 }' /proc/meminfo)"
  if (( mem_kb < 20000000 )); then
    export CARGO_BUILD_JOBS=1
  elif (( mem_kb < 28000000 )); then
    export CARGO_BUILD_JOBS=2
  fi
fi
