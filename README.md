# hammerhead-platform-controller

Install podman and podman-compose to run pre-commit, builds, tests
`podman-compose run --build pre-commit`
`podman-compose run --build build-debug`
`podman-compose run --build build-release`

For interactive debugging, install cortex-debug VSCode extension and the following dependencies
`sudo apt install build-essential cmake openocd binutils-multiarch gdb-multiarch gcc-arm-none-eabi`

TODO
- Add IMU
- Add fault LED
- Add external i2c pullups
- Remove flow control and power-in pins from UART and SWDIO headers
- Decrease MC buck regulator powerup thresholds to 12V
- Change VIN and VMIN voltage dividers to support full voltage ranges
- Add bottom-side testpoints

- Add mission AO with i2c comms
- Add unit tests
