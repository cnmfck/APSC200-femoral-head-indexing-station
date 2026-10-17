# Package validation

- All 11 supplied Arduino sketches are retained and match their original bytes.
- `anti_stall_encoder_sign_flip` is the main program, as explicitly confirmed by Shenghan Gao.
- Arduino sketch filenames match their containing folders.
- Key saved constants were checked against the controller parameter table.
- Local Markdown/image links were checked.
- Both included images were visually inspected; their original bytes are preserved.
- The finished ZIP passed its integrity check.

No source code was modified. The Arduino compiler and board libraries were not available for a new build; no firmware upload or physical test was performed during packaging. Hardware-success attribution refers to the author's report of the original run.
