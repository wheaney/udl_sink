# udl_sink

Sink-side parser/decoder for the UDL command stream emitted by the Linux kernel driver.

## Current scope

The first implementation pass focuses on the packet formats that show up in the current kernel damage path:

- `WRITEREG` register writes
- `WRITERAW8` raw sidecar writes for the 8bpp plane
- `WRITERL8` repeated sidecar writes for the 8bpp plane
- `WRITECOPY8` device-side copies within the 8bpp plane
- `WRITERLX8` extended run-length packets for the 8bpp plane
- `WRITERAW16` raw RGB565 damage writes
- `WRITERL16` repeated RGB565 damage writes
- `WRITECOPY16` device-side framebuffer copies
- `WRITERLX16` extended run-length packets from `udl_transfer.c`

The sink now decodes into internal 16bpp and 8bpp device planes, applies the base-address registers for each plane, and composes the visible image from that state.

- The existing caller-provided RGB565 framebuffer is still supported as a compatibility output.
- An optional XRGB8888 output can be attached with `udl_sink_attach_xrgb8888_output()` to preserve full 24bpp reconstruction when the stream uses the 8bpp sidecar plane.
- `udl_transport` now owns the reusable bulk-stream reassembly path so callers can feed transport-sized reads directly into the library.

The current kernel DRM driver in `udl_ref` still initializes only the 16bpp path, but the sink can now decode 24bpp-style command streams on the receiving side.

## Build

```sh
cmake -S . -B build
cmake --build build
ctest --test-dir build --output-on-failure
```
