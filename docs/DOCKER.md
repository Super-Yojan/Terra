# Simulator image

!!! tip "TL;DR"
    The Bevy container moved to [Zorvane](https://super-yojan.dev/Zorvane/).
    This repo’s container builds the Pi executable.

```mermaid
flowchart LR
  Old[Old simulator image] --> Z[Zorvane]
  Here[packaging/rover/Dockerfile] --> Pi[terra-rover ARM64]
```

*Prefix is still `terra/rover`. `TERRA_*` variables still apply in Zorvane.*

![Pi stack this image installs](assets/pi-stack.svg)

*Install steps: [compiled Pi executable](hardware/INSTALL.md).*

```sh
cargo run -p zorvane
```

Run that from a Zorvane checkout, not from this tree.
