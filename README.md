# hammerhead-platform-controller

Install podman and podman-compose to run pre-commit, builds, tests
`podman-compose run --build pre-commit`
`podman-compose run --build build-debug`
`podman-compose run --build build-release`

For interactive debugging, install cortex-debug VSCode extension and the following dependencies
`sudo apt install build-essential cmake openocd binutils-multiarch gdb-multiarch gcc-arm-none-eabi`

TODO
- Add motor drivers
- (DROPPED) Add EEPROM driver and parameter system
- Add flash parameter system
- Add hall current sensors
- Add fault LED
- Add external i2c pullups
