#![no_std]
#![no_main]

use panic_halt as _;

// Chapter 6!
// MAKE A PLAN! What all needs to be done?
// In:
//   ✅ Random Flash the LEDs
//   - Add a variable for the length of time the light is on
// Out:
//   - Track button clicks
//   - Any sort of levels

#[rtic::app(device = rp_pico::hal::pac, peripherals = true)]
mod app {
    // use fugit::MicrosDurationU32;

    use rp_pico::{
        hal::{
            self,
            clocks::init_clocks_and_plls,
            gpio::Interrupt::LevelLow,
            gpio::{PullUp, PushPull},
            watchdog::Watchdog,
            Clock, Sio,
        },
        XOSC_CRYSTAL_FREQ,
    };

    use oorandom::Rand32;
    // #[monotonic(binds = TIMER_IRQ_0, default = true)]
    // type Rp2040Mono = Rp2040Monotonic;

    #[shared]
    struct Shared {
        left_led: hal::gpio::Pin<hal::gpio::pin::bank0::Gpio0, hal::gpio::Output<PushPull>>,
        center_led: hal::gpio::Pin<hal::gpio::pin::bank0::Gpio1, hal::gpio::Output<PushPull>>,
        right_led: hal::gpio::Pin<hal::gpio::pin::bank0::Gpio2, hal::gpio::Output<PushPull>>,
        world: u32,
        level: u32,
        round: u32,
    }

    #[local]
    struct Local {
        left_button: hal::gpio::Pin<hal::gpio::pin::bank0::Gpio16, hal::gpio::Input<PullUp>>,
        center_button: hal::gpio::Pin<hal::gpio::pin::bank0::Gpio17, hal::gpio::Input<PullUp>>,
        right_button: hal::gpio::Pin<hal::gpio::pin::bank0::Gpio21, hal::gpio::Input<PullUp>>,
        delay: cortex_m::delay::Delay,
    }

    #[init]
    fn init(c: init::Context) -> (Shared, Local, init::Monotonics) {
        // Soft-reset does not release the hardware spinlocks
        // Release them now to avoid a deadlock after debug or watchdog reset
        unsafe {
            hal::sio::spinlock_reset();
        }
        // notice how in this chapter we use "c"
        // for example c.deviceresets
        let mut resets = c.device.RESETS;
        let mut watchdog = Watchdog::new(c.device.WATCHDOG);
        let _clocks = init_clocks_and_plls(
            XOSC_CRYSTAL_FREQ,
            c.device.XOSC,
            c.device.CLOCKS,
            c.device.PLL_SYS,
            c.device.PLL_USB,
            &mut resets,
            &mut watchdog,
        )
        .ok()
        .unwrap();

        let sio = Sio::new(c.device.SIO);
        let pins = rp_pico::Pins::new(
            c.device.IO_BANK0,
            c.device.PADS_BANK0,
            sio.gpio_bank0,
            &mut resets,
        );

        let delay = cortex_m::delay::Delay::new(c.core.SYST, _clocks.system_clock.freq().to_Hz());
        let left_led = pins.gpio0.into_push_pull_output();
        let center_led = pins.gpio1.into_push_pull_output();
        let right_led = pins.gpio2.into_push_pull_output();

        // Our button inputs
        let left_button = pins.gpio16.into_pull_up_input();
        left_button.set_interrupt_enabled(LevelLow, true);

        let center_button = pins.gpio17.into_pull_up_input();
        center_button.set_interrupt_enabled(LevelLow, true);

        let right_button = pins.gpio21.into_pull_up_input();
        right_button.set_interrupt_enabled(LevelLow, true);

        let world: u32 = 1;
        let level: u32 = 1;
        let round: u32 = 1;

        (
            Shared {
                left_led,
                center_led,
                right_led,
                world,
                level,
                round,
            },
            Local {
                left_button,
                center_button,
                right_button,
                delay,
            },
            init::Monotonics(),
        )
    }

    #[idle(
      shared = [left_led, center_led, right_led, world, level, round],
      local = [delay]
    )]
    fn idle(mut ctx: idle::Context) -> ! {
        use embedded_hal::digital::v2::OutputPin;
        let some_seed = 9;
        let mut rng = Rand32::new(some_seed);
        let timer: u32 = 500;

        loop {
            let rand_led = Rand32::rand_range(&mut rng, 1..4);
            if rand_led == 1 {
                ctx.shared.left_led.lock(|left_led| {
                    left_led.set_high().unwrap();
                });
                ctx.local.delay.delay_ms(timer);
                ctx.shared.left_led.lock(|left_led| {
                    left_led.set_low().unwrap();
                });
            }

            if rand_led == 2 {
                ctx.shared.center_led.lock(|center_led| {
                    center_led.set_high().unwrap();
                });
                ctx.local.delay.delay_ms(timer);
                ctx.shared.center_led.lock(|center_led| {
                    center_led.set_low().unwrap();
                });
            }

            if rand_led == 3 {
                ctx.shared.right_led.lock(|right_led| {
                    right_led.set_high().unwrap();
                });
                ctx.local.delay.delay_ms(timer);
                ctx.shared.right_led.lock(|right_led| {
                    right_led.set_low().unwrap();
                });
            }

            ctx.local.delay.delay_ms(timer);
        }
    }

    #[task(binds = IO_IRQ_BANK0, local = [left_button, center_button, right_button], shared = [left_led, center_led, right_led])]
    fn button_press(mut ctx: button_press::Context) {
        use embedded_hal::digital::v2::{InputPin, OutputPin};
        loop {
            if ctx.local.left_button.is_high().unwrap()
                && ctx.local.center_button.is_high().unwrap()
                && ctx.local.right_button.is_high().unwrap()
            {
                (
                    ctx.shared.left_led,
                    ctx.shared.center_led,
                    ctx.shared.right_led,
                )
                    .lock(|left_led, center_led, right_led| {
                        left_led.set_low().unwrap();
                        center_led.set_low().unwrap();
                        right_led.set_low().unwrap();
                    });
                break;
            }
            if ctx.local.left_button.is_low().unwrap() {
                ctx.shared.left_led.lock(|left_led| {
                    left_led.set_high().unwrap();
                })
            } else {
                ctx.shared.left_led.lock(|left_led| {
                    left_led.set_low().unwrap();
                })
            }
            if ctx.local.center_button.is_low().unwrap() {
                ctx.shared.center_led.lock(|center_led| {
                    center_led.set_high().unwrap();
                })
            } else {
                ctx.shared.center_led.lock(|center_led| {
                    center_led.set_low().unwrap();
                })
            }
            if ctx.local.right_button.is_low().unwrap() {
                ctx.shared.right_led.lock(|right_led| {
                    right_led.set_high().unwrap();
                })
            } else {
                ctx.shared.right_led.lock(|right_led| {
                    right_led.set_low().unwrap();
                })
            }
        }
    }
}
