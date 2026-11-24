# Source checks

Checked on 2026-11-20 using MSYS2 UCRT64 on Windows.

- RTKLIB static and shared libraries built from the sources in this repository.
- `test_bcnv3`: navigation protocol and observation checks passed.
- `test_modern`: ICD fields, CRC, satellite identifiers, issue-of-data fields, reference times and week rollover checks passed.
- `test_signal`: 13 positioning cases passed, covering signal-source selection, nonzero biases, frequency slots and legacy signals.
- All 21 Python files passed syntax checks.
