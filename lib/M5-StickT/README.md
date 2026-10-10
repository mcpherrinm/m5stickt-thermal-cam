M5Stack's M5-StickT library (MIT), vendored from
https://github.com/m5stack/M5-StickT at a502cb40e0837bccd55a3e09389f3cbfc155cea8
(examples removed), with these changes to build and run on Arduino core 3.x:

- `src/M5StickT.h`: ASCII quotes in the `#error` line (GCC 14 rejects curly ones).
- `src/utility/In_eSPI.h`: include `soc/gpio_struct.h` for direct GPIO register access.
- `src/utility/In_eSPI.cpp`: set the display's CS, DC and RST pins as outputs before
  writing them (core 3.x ignores writes to pins that aren't outputs; RST was never set).
- `src/utility/Button.cpp`: `INPUT` instead of `INPUT_PULLUP` (the buttons are on
  input-only pins with external pull-ups).
