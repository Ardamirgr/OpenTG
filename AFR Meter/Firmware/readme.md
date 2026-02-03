# Firmware Versions (v1.0 / v1.1)

This folder contains firmware code for **two boards** in the system:

- **OpenAFR** = mainboard  
- **Gauge** = gauge/display board

Both boards have **v1.0** and **v1.1** code included.

> ⚠️ **v1.1 is an experimental test version and is currently not working.**  
> Use **v1.0** if you need a working build.

## Folder layout

- `Libraries/` — Shared libraries used by the projects. Heavily modified for flash space. (WILL NOT WORK IN OTHER PROJECTS) 
- **Mainboard (OpenAFR)**
  - `OpenAFR/` — **v1.0 (working)**
  - `OpenAFR_v1.1/` — **v1.1 (test / not working)**
- **Gauge**
  - `gaugev1.0/` — **v1.0 (working)**
  - `gaugev1.1/` — **v1.1 (test / not working)**

## Recommended use

- Start from **v1.0** folders for development, builds, or reference.
- Treat **v1.1** folders as WIP/experimental and expect breaking changes.

## Contributing

If you’re fixing or improving things:
- Please target **v1.1** for experiments.
- Keep **v1.0** stable unless there’s a clear bugfix.

## License

See the repository root for license information.

