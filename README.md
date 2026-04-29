# udl_sink

Sink-side parser/decoder for the UDL command stream emitted by the Linux kernel driver.

## Current scope

The first implementation pass focuses on the packet formats that show up in the current kernel damage path:

- `WRITEREG` register writes
- `WRITERAW16` raw RGB565 damage writes
- `WRITERL16` repeated RGB565 damage writes
- `WRITECOPY16` device-side framebuffer copies
- `WRITERLX16` extended run-length packets from `udl_transfer.c`

The library currently treats the device framebuffer as a linear RGB565 surface and maps that into a caller-provided width/height/stride. USB transport and 8-bit command decoding are intentionally left out of this first slice.

## Build

```sh
cmake -S . -B build
cmake --build build
ctest --test-dir build --output-on-failure
```
