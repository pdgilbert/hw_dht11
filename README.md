# Rust DHT-11 sensor on bluepill/blackpill

##  Contents
- [Summary](#summary)
- [Building](#building)
- [Loading](#loading)

## Summary

This is derived from https://github.com/pdgilbert/rust-integration-testing/examples/rtic/dht_rtic.
It is intended for a specific hardware setup and attempts to use released crates for stability.

Relative to https://github.com/pdgilbert/rust-integration-testing/examples/rtic/dht_rtic:
-there is no attempt to generalized for many MCU devices or check git versions of crates
-code is in scr/lib rather than examples/


## Building

Set one of these lines:
```
               environment variables for cargo                       openocd         embed        test board and processor
  _____________________________________________________________     _____________  _____________   ___________________________
  export HAL=stm32f1xx MCU=stm32f103   TARGET=thumbv7m-none-eabi    PROC=stm32f1x  CHIP=STM32F103C8  # bluepill         Cortex-M3
  export HAL=stm32f4xx MCU=stm32f401   TARGET=thumbv7em-none-eabihf PROC=stm32f4x  CHIP=STM32F4x  # blackpill-stm32f401 Cortex-M4
  export HAL=stm32f4xx MCU=stm32f411   TARGET=thumbv7em-none-eabihf PROC=stm32f4x  CHIP=STM32F4x  # blackpill-stm32f411 Cortex-M4
```
then to build
```
cargo build --no-default-features --target $TARGET --features $MCU,$HAL  [ --release ]
```
The `--release` will be needed if code is too big for memory.

??--no-default-features ??

## Loading

Briefly:

If `openocd`, `gdb`, `.cargo/config` with needed runners, and an appropriate probe are 
in place then in one window run
```
openocd -f interface/$INTERFACE.cfg -f target/$PROC.cfg 
```
where INTERFACE is set for your probe, for example, `export INTERFACE=stlink-v2` for a typical cheap dongle
or `export INTERFACE=stlink-v2-1` for a slightly newer version.
In another window do
```
cargo  run --target $TARGET --features $HAL,$MCU  [ --release]
```

## License

Licensed under either of

 * Apache License, Version 2.0 ([LICENSE-APACHE](LICENSE-APACHE) or
   http://www.apache.org/licenses/LICENSE-2.0)
 * MIT license ([LICENSE-MIT](LICENSE-MIT) or
   http://opensource.org/licenses/MIT)

at your option.

### Contributing

Unless you explicitly state otherwise, any contribution intentionally submitted
for inclusion in the work by you, as defined in the Apache-2.0 license, shall
be dual licensed as above, without any additional terms or conditions.
