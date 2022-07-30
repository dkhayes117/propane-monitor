# `propane-monitor`

nrf9160 based project for monitoring propane tanks via cellular comms


## Notes
- includes a pre-compiled `zephyr.bin` (secure partition manager)

## Pre-Reqs
- Install [Rustup]
- Add the target with the command below
  ``` console
  $ rustup target add thumbv8m.main-none-eabihf
  ```
- install [probe-run]. On Linux, you might have to install libudev and libusb
  from your package manager before installing `probe-run`
    ```console
    $ cargo install probe-run
    ```
  probe-run is built on the [probe-rs] library and supports `CMSIS-DAP`, `ST-Link`,
  and `Segger J-Link` out of the box.  `J-Link` is recommended for this repo.  For
  Linux, [udev rules] can be added for user access without root privileges.
## Running a Binary
- run example `hello.rs` as a debug build
  ```console
  $ cargo run --example hello
  ```
- or run with [alias commands](.cargo/config.toml) defined in .cargo/config.toml
  ```console
  $ cargo rex hello
  ```
- run example as a release build (production build)
  ```console
  $ cargo rrex hello
  ```
- run main.rs file
  ```console
  $ cargo run
  ```
  ```console
  $ cargo run --release
  ```
## License

Licensed under either of

- Apache License, Version 2.0 ([LICENSE-APACHE](LICENSE-APACHE) or
  http://www.apache.org/licenses/LICENSE-2.0)

- MIT license ([LICENSE-MIT](LICENSE-MIT) or http://opensource.org/licenses/MIT)

at your option.

### Contribution

Unless you explicitly state otherwise, any contribution intentionally submitted
for inclusion in the work by you, as defined in the Apache-2.0 license, shall be
licensed as above, without any additional terms or conditions.

[Rustup]: https://www.rust-lang.org/learn/get-started
[probe-run]: https://crates.io/crates/probe-run
[probe-rs]: https://probe.rs/
[udev rules]: https://probe.rs/docs/getting-started/probe-setup/
