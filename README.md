# omnia-mcutool

This program interacts with the microcontroller on Turris Omnia boards via the
I2C bus.

It can:

- display version (git commit hashes) of the MCU firmware - both for the
  bootloader and application part, and various other information about the
  firmware running on the MCU (use the `-v` / `--firmware-version` option)
- upgrade MCU firmware to the newest version via the `--upgrade` option,
  or flash MCU firmware with a specific binary via the `--firmware` option,
- configure wake up time (`-w` / `--wakeup` option) if the board is brought
  down to low power mode (by kernel, or by the not recommended `--poweroff`
  option),
- enable / disable MCU watchdog (via `--watchdog`), configure its timeout
  (`--watchdog-timeout`) and display its status (`--watchdog-status`),
- enable / disable USB port power (`--usb-port-0` and `--usb-port-1`) and report
  USB power status (`--usb-status`),
- configure brightness of the front LEDs panel (`--leds-brightness`),
  enable / disable LEDs gamma correction (`--leds-gamma`), display LED
  configuration status (`--leds-status`) and do a simple LED controller stress
  test (`--leds-stress-test`),
- configure WAN interface endpoint (to WAN ethernet port or SFP cage, via the
  `--wan-mode` option) and display the current configuration (`--wan-status`,
  `--get-wan-mode`),
- configure front button press event mode (whether it changes LED panel
  brightness or is handled by the system) via the `--button-mode` option
  and display the current configuration (`--button-status`,
  `--get-button-mode`),
- display status of MCU GPIO pins (`--gpio-status`) and configure GPIO pins
  (`--gpio-set`, `--gpio-clear`),
- and finally show the selected factory reset level that the user selected by
  holding the rear reset button (`--reset-selector`).

## MCU firmware binaries

You can find the MCU firmware binaries at
https://gitlab.nic.cz/turris/hw/omnia_hw_ctrl/-/releases
when flashing specific firmware with the `--firmware` option.

The `--upgrade` option tries to automatically find the newest firmware in the
`/usr/share/omnia-mcu-firmware` directory. It may also need the `v2.99` version
of the application firmware when upgrading from older firmware - these should be
in the `/usr/share/omnia-mcu-firmware/v2.99` directory.

## Build

The program requires building with GCC. Clang is not tested.

Simply run `make` with the appropriate `CC` and `CFLAGS` settings.

## License

This program is licensed under GNU GPL v2 or later.