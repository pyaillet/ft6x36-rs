# FT6x36 Rust driver

Minimal FT6x36 implementation.

What's working:
- Reporting touch event statuses by querying the device

What's missing:
- Detecting gestures within the device. I found reports noticing the same problem
  with C/C++ drivers, so I guess we are missing some information from the datasheet here

Interrupt service handler setup is not provided, as it depends on your platform

