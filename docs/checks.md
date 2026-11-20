# Source checks

Checked on 2026-11-20 using MSYS2 UCRT64 on Windows.

- RTKLIB static and shared libraries built from the sources in this repository.
- `test_bcnv3`: navigation protocol and observation checks passed.
- `test_modern`: ICD fields, CRC, satellite identifiers, issue-of-data fields, reference times and week rollover checks passed.
- `test_signal`: 13 positioning cases passed, covering signal-source selection, nonzero biases, frequency slots and legacy signals.
- All 21 Python files passed syntax checks.
- All files in the source manifest matched the local 0.21-fixed snapshot byte for byte.

The 13 modules added since the upstream commit passed the file-structure check. The comment checker produced 26 flags for C preprocessor directives and pointer assignments; all were reviewed. Existing source files retain their original structure and comments.
