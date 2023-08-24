//! Measure the temperature and humidity from a DHT11  on data pin (A8) and display on OLED with i2c.
//! Blink (onboard) LED with short pulse evry read.
//! On startup the LED is set on for about (at least) 5 seconds in the init process.
//! One main processe is scheduled. It reads the dht and spawns itself to run after a delay.
//! It also spawns a `blink` process that turns the led on and schedules another process to turn it off.
//!
//!  A good reference on performance of humidity sensors is
//!     https://www.kandrsmith.org/RJS/Misc/hygrometers.html
//!
//! See README for build instructions.

#![deny(unsafe_code)]
#![no_std]
#![no_main]

#[cfg(debug_assertions)]
use panic_semihosting as _;

#[cfg(not(debug_assertions))]
use panic_halt as _;


use rtic::app;

#[cfg_attr(feature = "stm32f1xx", app(device = stm32f1xx_hal::pac,   dispatchers = [TIM2, TIM3]))]
#[cfg_attr(feature = "stm32f4xx", app(device = stm32f4xx_hal::pac,   dispatchers = [TIM2, TIM3]))]

mod app {

    const START_TEXT: &str = "hw_dht11\n 2025/08/24"; 

    #[cfg(feature = "stm32f1xx")]
    const MONOCLOCK: u32 = 8_000_000; 

    #[cfg(feature = "stm32f4xx")]
    const MONOCLOCK: u32 = 16_000_000; 

    const MONOTICK:  u32 = 100;
    const READ_INTERVAL: u64 = 10;   // used as seconds
    const BLINK_DURATION: u64 = 20;  // used as milliseconds

    // Note that hprintln is for debugging with usb probe and semihosting. 
    //   It causes battery operation to stall.
    //use cortex_m_semihosting::{debug, hprintln};
    //use cortex_m_semihosting::{hprintln};

    //https://github.com/michaelbeaumont/dht-sensor
    use dht_sensor::*;
    #[cfg(not(feature = "dht22"))]
    use dht_sensor::dht11::Reading;
    #[cfg(feature = "dht22")]
    use dht_sensor::dht11::Reading;

    use ssd1306::{
        mode::BufferedGraphicsMode, 
        prelude::*, 
        I2CDisplayInterface, 
        size::DisplaySize128x32 as DISPLAYSIZE,
        Ssd1306
    };

    use shared_bus::{I2cProxy};
    use core::cell::RefCell;
    use cortex_m::interrupt::Mutex;
    use core::fmt::Write;
    use systick_monotonic::*;
    use cortex_m::prelude::_embedded_hal_blocking_delay_DelayMs;

    // secs() and millis() methods from https://docs.rs/fugit/latest/fugit/trait.ExtU32.html#tymethod.secs
    use fugit::TimerDuration;   


    #[cfg(feature = "stm32f1xx")]
    pub use stm32f1xx_hal as hal;

    #[cfg(feature = "stm32f4xx")]
    pub use stm32f4xx_hal as hal;
     
    use hal::{
        gpio::{gpioa::PA8, Output, OpenDrain},  // pin for dht data
        gpio::{gpioc::PC13,  PushPull},
        timer::Delay,
        pac::I2C1,
        pac::{TIM2},
        gpio::GpioExt,
        prelude::_fugit_RateExtU32,
        prelude::_stm32f4xx_hal_timer_TimerExt,   //weird, for trait missing causing  no method named `delay_us` found for struct `TIM2
    };

    #[cfg(feature = "stm32f1xx")]
    use stm32f1xx_hal::{
        i2c::{BlockingI2c, DutyCycle, Mode},
        flash::FlashExt,
        gpio::{gpiob::{PB8, PB9}, Alternate},
        afio::AfioExt,
        prelude::_stm32_hal_rcc_RccExt,
    };

    
    #[cfg(feature = "stm32f4xx")]
    use stm32f4xx_hal::{
        i2c::{I2c, },
        //gpio::{Alternate, OpenDrain, gpiob::{PB8, PB9,  PB10, PB3, Parts as PartsB}},
        rcc::RccExt,
    };

    
    #[cfg(feature = "stm32f1xx")]
    type I2c1Type = BlockingI2c<I2C1, (PB8<Alternate<OpenDrain>>, PB9<Alternate<OpenDrain>>)>;   
    
    #[cfg(feature = "stm32f4xx")]
    type I2c1Type = I2c<I2C1>;
    
    type LedType = PC13<Output<PushPull>>;
    type DhtType = PA8<Output<OpenDrain>>;
    type Delay1Type = Delay<TIM2, 1000000_u32>;

    // See https://docs.rs/embedded-graphics/0.7.1/embedded_graphics/mono_font/index.html
    // DisplaySize128x32:
    //    &FONT_6X10 128 pixels/ 6 per font = 21.3 characters wide.  32/10 = 3.2 characters high
    //    &FONT_5X8  128 pixels/ 5 per font = 25.6 characters wide.  32/8  =  4  characters high
    //    FONT_8X13  128 pixels/ 8 per font = 16   characters wide.  32/13 = 2.5 characters high
    //    FONT_9X15  128 pixels/ 9 per font = 14.2 characters wide.  32/15 = 2.  characters high
    //    FONT_9X18  128 pixels/ 9 per font = 14.2 characters wide.  32/18 = 1.7 characters high
    //    FONT_10X20 128 pixels/10 per font = 12.8 characters wide.  32/20 = 1.6 characters high
    
    use embedded_graphics::{
        //mono_font::{ascii::FONT_10X20, MonoTextStyleBuilder, MonoTextStyle}, 
        mono_font::{iso_8859_1::FONT_10X20 as FONT, MonoTextStyleBuilder}, 
        pixelcolor::BinaryColor,
        prelude::*,
        text::{Baseline, Text},
    };
    
    
    trait LED {
        // depending on board wiring, on may be set_high or set_low, with off also reversed
        // implementation should deal with this difference
        fn on(&mut self) -> ();
        fn off(&mut self) -> ();
    }


    fn show_display<S>(
        temperature: i8,
        relative_humidity: u8,
        //text_style: MonoTextStyle<BinaryColor>,
        disp: &mut Ssd1306<impl WriteOnlyDataCommand, S, BufferedGraphicsMode<S>>,
    ) -> ()
    where
        S: DisplaySize,
    {
       
       // workaround. build here because text_style cannot be shared
       let text_style = MonoTextStyleBuilder::new().font(&FONT).text_color(BinaryColor::On).build();
    
       let mut lines: [heapless::String<32>; 1] = [
           heapless::String::new(),
       ];
    
       // Many SSD1306 modules have a yellow strip at the top of the display, so first line may be yellow.
       // It is possible to use \n in place of separate writes, with one line rather than vector.
    
       // UTF-8 text is 2 bytes (2 ascii characters) in strings like the next. Cutting an odd number of character from
       // the next test_text can result in a build error message  `stream did not contain valid UTF-8` even with
       // the line commented out!! The test_txt is taken from 
       //      https://github.com/embedded-graphics/examples/blob/main/eg-0.7/examples/text-extended-characters.rs
       
       //let test_text  = "¡¢£¤¥¦§¨©ª«¬­®¯°±²³´µ¶·¸¹º»¼½¾¿ÀÁÂÃÄÅÆÇÈÉÊËÌÍÎÏÐÑÒÓÔÕÖ×ØÙÚÛÜÝÞßàáâãäåæçèéêëìíîïðñòóôõö÷øùúûüýþÿ";
       //   degree symbol "°" is about                  ^^ here 
       
       write!(lines[0], "{:3}°C{:3}%RH", temperature, relative_humidity).unwrap();

       disp.clear_buffer();
       for i in 0..lines.len() {
           // start from 0 requires that the top is used for font baseline
           Text::with_baseline(
               &lines[i],
               Point::new(0, i as i32 * 12), //with font 6x10, 12 = 10 high + 2 space
               text_style,
               Baseline::Top,
               )
               .draw(&mut *disp)
               .unwrap();
       }
       disp.flush().unwrap();
       ()
    }



    #[monotonic(binds = SysTick, default = true)]
    type MyMono = Systick<MONOTICK>;

    #[init]
    fn init(cx: init::Context) -> (Shared, Local, init::Monotonics) {

        let rcc = cx.device.RCC.constrain();
 
        #[cfg(feature = "stm32f1xx")]
        let clocks = rcc.cfgr.freeze(&mut cx.device.FLASH.constrain().acr);
 
        #[cfg(feature = "stm32f4xx")]
        let clocks = rcc.cfgr.freeze();

        let mut delay = cx.device.TIM2.delay_us(&clocks);

        let mut gpioc = cx.device.GPIOC.split();

        #[cfg(feature = "stm32f1xx")]
        let mut led = gpioc.pc13.into_push_pull_output(&mut gpioc.crh);

        #[cfg(feature = "stm32f4xx")]
        let mut led = gpioc.pc13.into_push_pull_output();

        impl LED for LedType {
            fn on(&mut self) -> () {
                self.set_low()
            }
            fn off(&mut self) -> () {
                self.set_high()
            }
        }

        let mut gpioa = cx.device.GPIOA.split();

        #[cfg(feature = "stm32f1xx")]
        let mut dht = gpioa.pa8.into_open_drain_output(&mut gpioa.crh);

        #[cfg(feature = "stm32f4xx")]
        let mut dht = gpioa.pa8.into_open_drain_output();

        let mut gpiob = cx.device.GPIOB.split();

        #[cfg(feature = "stm32f1xx")]
        let scl = gpiob.pb8.into_alternate_open_drain(&mut gpiob.crh);

        #[cfg(feature = "stm32f1xx")]
        let sda = gpiob.pb9.into_alternate_open_drain(&mut gpiob.crh);

        #[cfg(feature = "stm32f1xx")]
        let mut afio = cx.device.AFIO.constrain();

        #[cfg(feature = "stm32f1xx")]
        let i2c = BlockingI2c::i2c1(
            cx.device.I2C1,
            (scl, sda),
            &mut afio.mapr,
                Mode::Fast {
                frequency: 100_000_u32.Hz(),
                duty_cycle: DutyCycle::Ratio2to1,
            },
            clocks,
            1000,
            10,
            1000,
            1000,
        );

        #[cfg(feature = "stm32f4xx")]
        let scl = gpiob.pb8.into_alternate_open_drain::<4u8>(); // _af4   PB8<Alternate<4u8, OpenDrain>>

        #[cfg(feature = "stm32f4xx")]
        let sda = gpiob.pb9.into_alternate_open_drain::<4u8>(); // _af4

        #[cfg(feature = "stm32f4xx")]
        let i2c = I2c::new(cx.device.I2C1, (scl, sda), 400.kHz(), &clocks);

        led.on();
        delay.delay_ms(1000u32);  
        led.off();

        dht.set_high(); // Pull high to avoid confusing the sensor when initializing.
        delay.delay_ms(2000_u32); //  2 second delay for dhtsensor initialization

        let manager: &'static _ = shared_bus::new_cortexm!(I2c1Type = i2c).unwrap();

        let interface = I2CDisplayInterface::new(manager.acquire_i2c());

        let text_style = MonoTextStyleBuilder::new().font(&FONT).text_color(BinaryColor::On).build();

        let mut display = Ssd1306::new(interface, DISPLAYSIZE, DisplayRotation::Rotate0)
            .into_buffered_graphics_mode();

        display.init().unwrap();

        Text::with_baseline(START_TEXT, Point::zero(), text_style, Baseline::Top, )
          .draw(&mut display).unwrap();
        display.flush().unwrap();
        delay.delay_ms(2000u32);    

        let mono = Systick::new(cx.core.SYST,  MONOCLOCK);

        // next turn LED for a period of time that can be used to calibrate the delay timer.
        // Ensure that nothing is spawned above. This relies on delay blocking.
        led.on();
        delay.delay_ms(5000u32);  
        led.off();
        delay.delay_ms(1000u32);  
        read_and_display::spawn().unwrap();

        //hprintln!("exit init").unwrap();
        (Shared { led, }, Local {dht, delay, display }, init::Monotonics(mono))
    }

    #[shared]
    struct Shared {
        led:   LedType,      //impl LED, would be nice
    }

// see disply_stuff_rtic and  https://github.com/jamwaffles/ssd1306/issues/164 regarding
// problem Shared local text_style. Workaroound by building in the task (may not be very efficient).

    #[local]
    struct Local {
        dht:   DhtType,
        delay: Delay1Type,
        //display: Ssd1306<impl WriteOnlyDataCommand, DISPLAYSIZE, BufferedGraphicsMode<DISPLAYSIZE>>,
        display:  Ssd1306<I2CInterface<I2cProxy<'static, Mutex<RefCell<I2c1Type>>>>, 
                          ssd1306::prelude::DisplaySize128x32,  //DisplaySizeType, 
                          BufferedGraphicsMode<DISPLAYSIZE>>,
//        text_style: MonoTextStyle<BinaryColor>,
    }

    #[task(shared = [led, ], local = [dht, delay, display ], capacity=2)]
    fn read_and_display(cx: read_and_display::Context) {
        //hprintln!("read_and_display").unwrap();
        blink::spawn(BLINK_DURATION.millis()).ok();

        let delay = cx.local.delay;
        let dht = cx.local.dht;
        let z = Reading::read(delay, dht);
        let (_temperature, _humidity) = match z {
            Ok(Reading {temperature, relative_humidity,})
               =>  {//hprintln!("{} deg C, {}% RH", temperature, relative_humidity).unwrap();
                    //show_display(temperature, relative_humidity, text_style, &mut display)
                    show_display(temperature, relative_humidity, cx.local.display);
                    (temperature, relative_humidity)
                   },
            Err(_e) 
               =>  {//hprintln!("dht Error {:?}", e).unwrap(); 
                    //panic!("Error reading DHT")
                    (25, 40)  //supply default values   panic reset would be better
                   },
        };

        read_and_display::spawn_after(READ_INTERVAL.secs()).unwrap();
    }

    #[task(shared = [led], capacity=2)]
    fn blink(_cx: blink::Context, duration: TimerDuration<u64, MONOTICK>) {
        led_on::spawn().unwrap();
        led_off::spawn_after(duration).unwrap();
    }

    #[task(shared = [led], capacity=2)]
    fn led_on(mut cx: led_on::Context) {
        cx.shared.led.lock(|led| led.on());
    }

    #[task(shared = [led], capacity=2)]
    fn led_off(mut cx: led_off::Context) {
        cx.shared.led.lock(|led| led.off());
    }
}


// consider putting some real tests here

#[test]
#[should_panic]
panic_halt::panic!();

#[test]
assert_eq!(42, 42);
