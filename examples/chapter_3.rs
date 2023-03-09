#![no_std]
#![no_main]

use panic_halt as _;

#[rtic::app(device = rp_pico::hal::pac, peripherals = true)]
mod app {

    use rp_pico::{
        hal::{self, clocks::init_clocks_and_plls, gpio::PushPull, watchdog::Watchdog, Clock, Sio},
        XOSC_CRYSTAL_FREQ,
    };

    #[shared]
    struct Shared {}

    #[local]
    struct Local {
        left_led: hal::gpio::Pin<hal::gpio::pin::bank0::Gpio0, hal::gpio::Output<PushPull>>,
        center_led: hal::gpio::Pin<hal::gpio::pin::bank0::Gpio1, hal::gpio::Output<PushPull>>,
        right_led: hal::gpio::Pin<hal::gpio::pin::bank0::Gpio2, hal::gpio::Output<PushPull>>,
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

        // Our button input
        // let left_button = pins.gpio16.into_pull_up_input();
        // left_button.set_interrupt_enabled(interrupt, enabled);

        (
            Shared {},
            Local {
                left_led,
                center_led,
                right_led,
                delay,
            },
            init::Monotonics(),
        )
    }

    #[idle(
      local = [left_led, center_led, right_led, delay]
    )]
    fn idle(ctx: idle::Context) -> ! {
        use embedded_hal::digital::v2::OutputPin;
        loop {
            ctx.local.left_led.set_high().unwrap();
            ctx.local.right_led.set_low().unwrap();
            ctx.local.delay.delay_ms(250);

            ctx.local.center_led.set_high().unwrap();
            ctx.local.left_led.set_low().unwrap();
            ctx.local.delay.delay_ms(250);

            ctx.local.right_led.set_high().unwrap();
            ctx.local.center_led.set_low().unwrap();
            ctx.local.delay.delay_ms(250);
        }
    }
}
