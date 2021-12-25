# Nintendo Switch Pro Controller userspace driver

This repository contains a Nintendo Switch Pro Controller userspace driver for Linux.

  * Supports any number of connected controllers at the same time.
  * Supports connecting the controller through USB.
  * Supports force feedback/rumble.
  * Automatic analog stick calibration with carefully configured deadzone, range scaling and overshoot filtering.
  * Will appropriately set the controller's LEDs to let you know which controller is which, if you have multiple connected.
  * Compliant with the Linux Gamepad Specification.
  * Supported by SDL2 out-of-box as a gamepad.

Bluetooth and gyroscope are currently not supported. Normal joycons are also not supported.

## License

Licensed under either of

  * Apache License, Version 2.0, ([LICENSE-APACHE](LICENSE-APACHE) or http://www.apache.org/licenses/LICENSE-2.0)
  * MIT license ([LICENSE-MIT](LICENSE-MIT) or http://opensource.org/licenses/MIT)

at your option.

### Contribution

Unless you explicitly state otherwise, any contribution intentionally submitted
for inclusion in the work by you, as defined in the Apache-2.0 license, shall be
dual licensed as above, without any additional terms or conditions.
