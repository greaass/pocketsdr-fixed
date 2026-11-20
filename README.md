# pocketsdr-fixed

A modified version of [PocketSDR](https://github.com/tomojitakasu/PocketSDR) 0.21, with navigation decoding and positioning fixes for GPS L1C, L2C and L5, and BeiDou B1C, B2a and B2b.

The changes add modern ephemeris decoding, keep navigation messages tied to their signal source, and apply the corresponding frequency and code-bias corrections during positioning.

## Files

- `src/` and `python/`: receiver code and Python tools.
- `lib/`: RTKLIB, PocketFFT and library makefiles.
- `app/`, `html/` and `conf/`: command-line tools, web interface and configuration examples.
- `test/`: receiver tests, including the navigation and positioning regression tests.

## Build

See [build instructions](docs/build.md) for dependencies and commands. The Linux build uses GCC, GNU Make and libusb. The Python tools use NumPy, SciPy, Matplotlib and Tkinter.

Source hashes and the upstream commit are recorded in [UPSTREAM.md](UPSTREAM.md). The [change summary](docs/changes.md) lists the main fixes.

## License

PocketSDR and RTKLIB are by T. Takasu and use the BSD 2-clause license. See [LICENSE.txt](LICENSE.txt) and [RTKLIB's license](lib/RTKLIB/readme.txt). PocketFFT's license is in [lib/pocketfft/LICENSE.md](lib/pocketfft/LICENSE.md). Original copyright notices are retained.
