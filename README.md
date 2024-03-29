# Rust DHT-11 sensor on bluepill/blackpill

##  Contents
- [Summary](#summary)
- [Building](#building)
- [Loading](#loading)
- [Hardware](#hardware)
- [Status](#status)
- [License](#license)

## Summary

This code reads from a DHT-11 sensor and displays the temperature and humidity on
an 0.91" LCD OLED display (SSD 1306 128x32).
The code is derived from https://github.com/pdgilbert/rust-integration-testing/examples/rtic/dht_rtic.
It is intended for a specific hardware setup and for a sensor which has been
installed in a wall and is not easily changed.

Relative to https://github.com/pdgilbert/rust-integration-testing/examples/rtic/dht_rtic:
- there is no attempt to generalized for many MCU devices.
- released versions of crates are used, rather than git versions.
- code is in scr/main.rs rather than examples/.

In my on-going projects the DHT-11 sensor has been abandoned in favour of other sensors such as AHT-10.
(See https://github.com/pdgilbert/rust-integration-testing directory examples/rtic/.)
This was primiarly because of difficulty making a reliable physical wire connection to the DHT sensor.
The sensor is designed to be soldered on a circuit board. The AHT and others are available in modules on
small circuit boards that simplify a better physical connect to wires.
Also, the `I2C` interface for those sensors is widely used and thus probably more stable.


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
cargo build  --target $TARGET --features $MCU,$HAL --release
```
The `--release` is needed to run on bluepill. Otherwise the code is too big for memory.


## Loading

Briefly:

Assuming `openocd`, `gdb`, `.cargo/config` with needed runners and an appropriate probe are 
in place, in one window run
```
openocd -f interface/$INTERFACE.cfg -f target/$PROC.cfg 
```
where INTERFACE is set for your probe, for example, `export INTERFACE=stlink-v2` for a typical cheap dongle
or `export INTERFACE=stlink-v2-1` for a slightly newer version.
In another window do
```
cargo  run --target $TARGET --features $HAL,$MCU  --release
```


## Hardware

`VCC` for both the DHT-11 and SSD OLED display use `3.3`v from the bluepill/blackpill board.
This works when the unit is powered by 5v from the programming dongle and when it is powered 
by 5v (lithium ion) battery to a `5v` pin on the bluepill/blackpill board.
It also works when powered by 3.2v LiFePO4 battery to a `3v3` pin on the bluepill/blackpill board.
Note that 5v power uses the bluepill/blackpill on board regulator to power the 3.3v DHT and OLED.
Adding anything else may stress the regulator. A 3.2v supply does not use the regulator.

The DHT data pin is connected to pin `A8` on the bluepill/blackpill and has a `10K` pull up resistor.
The DHT `GND` pin is connect to `G` on the bluepill/blackpill. 
(Only 3 of 4 pins on the DHT are used.)

The display `I2C` interface uses the bluepill/blackpill I2C1 interface. 
On both bluepill and blackpill the DHT `SCK` is connected to pin `B8` and the DHT `SDA` to pin `B9`.


## Status

August 25, 2023 
  - builds and runs on bluepill with dongle, 5v battery and 3.2v battery.

  - builds and runs on blackpill stm32f401 with dongle, 5v battery and 3.2v battery. 
    After loading the `gdb` session stalls reading the DHT, but it works if the dongle is 
    disconnected and reconnected (but not re-programmed), so the dongle is just supplying
    power. 
    Also, the build gives a few `variable does not need to be mutable` warnings. This is
    because of a small difference between bluepill and blackpill hals.

August 26, 2023 
  - Power stability on blackpill stm32f401 became a problem after awhile. Not sure if
    this is a bad breadboard connection or damage to the blackpill, possibly caused by
    switching power connections and 3v vs 5v and a low battery during testing. Dongle 
    power and 3v3 power remain good (at least with properly charged battery).
    The problem is when 5v battery power is connected to a 5v pin. The symptom is the 
    blackpill and/or the DHT does not get enough power. Then either the blackpill does
    not boot or stalls waiting for data from the DHT. 
    I may have blown the onboard 3.3v regulator.

  - Humidity readings with 3.2v power seem to be less accurate than with 5v power.    
    Testing of this on the blackpill has not been possible because of the above problem.
    Bluepill measurements are better on 5v power.
    Blackpill on 3.2v power gives measurements close to those on bluepill 3.2v power.


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
