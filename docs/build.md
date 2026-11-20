# Build and test

## Linux

Install GCC, G++, GNU Make and the libusb-1.0 development headers. PocketFFT and the RTKLIB sources are included.

From the repository root:

```sh
mkdir -p lib/linux bin
make -C lib/build USE_SOAPY=0
make -C lib/build install USE_SOAPY=0
make -C app/pocket_trk USE_SOAPY=0
```

The makefiles also accept `USE_FFTW=1` for FFTW3 and `USE_SOAPY=1` for SoapySDR. Install the corresponding development packages before enabling these options. The x86 build enables AVX2 by default; pass `USE_AVX2=0` for a CPU without AVX2 support.

## Windows

Use MSYS2 UCRT64 with GCC, G++ and GNU Make. Create `lib/win32` and `bin` before installing the libraries. The Windows USB interface uses the Cypress CyAPI SDK: place its source and headers in `lib/cyusb`, and build `CyAPI.a` there using the SDK's license terms and the upstream [PocketSDR build instructions](https://github.com/tomojitakasu/PocketSDR).

Build the receiver libraries from `lib/build` with the supplied makefiles. `USE_SOAPY=0` selects the build without SoapySDR. With `USE_SOAPY=1`, install the SoapySDR headers and import library under `lib/SoapySDR` as expected by `libsdr.mk`.

## Python tools

Install NumPy, SciPy and Matplotlib in a Python environment with Tkinter. After building and installing the libraries, launch the receiver with:

```sh
python python/pocket_sdr.py
```

## Navigation regression tests

After building `librtk.a`, compile and run these tests from the repository root:

```sh
mkdir -p build
for name in test_bcnv3 test_modern test_signal; do
    gcc -O2 -Ilib/RTKLIB/src "test/$name.c" lib/build/librtk.a -lm -o "build/$name"
    "build/$name"
done
```

In MSYS2 UCRT64, add `-DWIN32` and replace `-lm` with `-lwsock32 -lwinmm` in the test link command. The tests check navigation-message fields, rejection of inconsistent messages, observation selection and positioning.
