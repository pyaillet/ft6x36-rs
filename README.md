<p align="center">
    <a href="https://github.com/pyaillet/ft6x36-rs/actions/workflows/ci.yml"><img src="https://github.com/pyaillet/ft6x36-rs/actions/workflows/ci.yml/badge.svg?branch=main" alt="Build status" /></a>
    <a href="https://crates.io/crates/ft6x36"><img src="https://img.shields.io/crates/v/ft6x36.svg" alt="Crates.io"></a>
    <a href="https://docs.rs/ft6x36"><img src="https://docs.rs/ft6x36/badge.svg" alt="Docs.rs"></a>
</p>

# FT6x36 Rust driver

Minimal FT6x36 implementation.

What's working:
- Reporting touch event statuses by querying the device

What's missing:
- Detecting gestures within the device. I found reports noticing the same problem
  with C/C++ drivers, so I guess we are missing some information from the datasheet here

Interrupt service handler setup is not provided, as it depends on your platform

