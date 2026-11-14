# Verification

The original video, workbook, report, and probability grid are project artifacts. The video shows the original prototype interface. The figures labeled synthetic come from a deterministic offline run.

Run `python -m unittest discover -s tests -v` to check the map and HTTP client. Run `python -m robotic_car review` to inspect the original grid and `python -m robotic_car demo` to exercise the maintained mapper on eight fixed observations. Both support `--image` to save a plot.

The maintained firmware still needs on-car validation of wiring, stop timing, motor direction and sensor calibration. The original recorded demonstration does not establish the behavior of the revised firmware.

## Local checks

- Firmware compiled with Arduino CLI 1.6.0 for `esp8266:esp8266:nodemcuv2`, ESP8266 core 3.1.2, and I2Cdevlib at revision a98805775f5c14b618387742d5ac6ac9c14e215e. The compiler reported 31,808 bytes of global/static RAM, 61,147 bytes of instruction RAM including the reserved cache, and 283,860 bytes of flash code.
- Nine Python tests passed. The saved-map and synthetic-map commands produced plots, and a mocked range scan wrote the expected grid without issuing a motor command. An invalid reading preserved the existing output.
- Edge passed the site checks at 1440 × 900, 820 × 1180 and 390 × 844 in both themes. Checks included menu links, gallery layout, search, categories, zoom, keyboard navigation, touch swipes, horizontal scrolling, downloads and video metadata.
- All 24 full figures and 24 previews decoded at their specified dimensions. The assets also loaded under a Pages-style subdirectory.
## Workbench and gallery

The desktop workbench opened the synthetic grid, applied a valid observation, rejected an invalid range without altering the grid, and exported and reopened a NumPy file. Edge passed updated navigation, theme, and gallery checks at widths of 320, 390, 820 and 1440 pixels in both themes. The README and webpage use the same 24 linked figures in the same order.

The square video player started the optimized MP4 in Edge at desktop, tablet and mobile widths in both themes. The optimized file is 4,835,153 bytes and places playback metadata before the media payload. The original recording remains available as a download.
