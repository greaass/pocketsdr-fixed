# Changes in 0.21-fixed

- Decode GPS CNAV and CNAV-2, and BeiDou B-CNAV1, B-CNAV2 and B-CNAV3 ephemerides.
- Check CRCs, satellite identifiers, message versions and reference times before assembling ephemerides.
- Keep modern and legacy navigation sources separate when selecting ephemerides for positioning.
- Use signal-specific observation slots, carrier frequencies and code-bias corrections.
- Add BDGIM broadcast-parameter decoding and delay calculation.
- Connect the new decoders to receiver tracking and positioning.
- Set the default acquisition code-length limit to 20 ms.

The navigation regression tests cover malformed messages, week rollover, ephemeris-source selection and nonzero code biases. The positioning test covers 13 modern and legacy signal cases.

The new modules follow the existing receiver interfaces. `modern_nav.h` defines shared types and identifiers; `cnav.c`, `bcnav.c`, `bcnv3.c`, `bdgim.c` and `pnt_signal.c` handle decoding and observation corrections; `sdr_modern.c` and `sdr_bcnv3.c` assemble messages at channel level. Existing receiver and positioning functions call these modules.
