# Firmware Versions (v1.0 / v1.1)

This folder contains firmware code for **two boards** in the system:

- **OpenAFR** = mainboard  
- **Gauge** = gauge/display board

Both boards have **v1.0** and **v1.1** code included.

> **v1.1 is the current best working version.**  
> **v1.0** is kept for version history and easy backtracking if needed.

## Folder layout

- `Libraries/` — Shared libraries used by the projects. Heavily modified for flash space. (WILL NOT WORK IN OTHER PROJECTS)
- **Mainboard (OpenAFR)**
  - `OpenAFR/` — **v1.0 (old)**
  - `OpenAFR_v1.1/` — **v1.1 (current / recommended)**
- **Gauge**
  - `gaugev1.0/` — **v1.0 (old)**
  - `gaugev1.1/` — **v1.1 (current / working)**

## Recommended use

- Start from the **v1.1** folders for development, builds, and reference.
- Use **v1.0** folders for version history, comparison, or fallback if needed.

## Contributing

If you’re fixing or improving things:
- Please target **v1.1** as the main development branch/version.
- Keep **v1.0** unchanged unless there’s a specific reason to patch the older version.

## License

See the repository root for license information.
