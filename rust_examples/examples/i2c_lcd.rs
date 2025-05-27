#![no_std]
#![no_main]
use core::panic::PanicInfo;
use hippomenes_rt as _;

#[rtic::app(device = hippomenes_core)]
mod app {
    use core::fmt::Write;
    use hippomenes_core::{OutputPin, Pout0, UART};
    #[shared]
    struct Shared {
        dummy: bool,
    }

    #[local]
    struct Local {
        // uart: UART,
        led0: Pout0,
        test: bool,
    }

    #[init]
    fn init(cx: init::Context) -> (Shared, Local) {
        let peripherals = cx.device;
        let timer = peripherals.timer;
        // let mut uart = peripherals.uart;

        unsafe {
            let indev = hippomenes_rust_examples::PeripheralsIndev::steal();
            let i2c = indev.i2c;

            i2c.write_i2c(60);
            i2c.write_i2c(43);
            i2c.write_i2c(28);
        }

        let led0 = peripherals.gpo.split().pout0;
        led0.set_high();

        // write!(uart, "init").ok();
        timer.write(0x100F); //timer interrupt every
                             // 500*2^15 ~ 16M cycles ~0.75s @ 20MHz
        (Shared { dummy: true }, Local { led0, test: true })
    }

    #[idle]
    fn idle(_: idle::Context) -> ! {
        loop {}
    }

    #[task(binds = Interrupt0, priority=1, shared=[dummy], local=[led0, test])]
    fn some_task(cx: some_task::Context) {
        // write!(cx.local.uart, "A").ok();
        //cx.local.uart.write_byte(0); // force sentinel, notice NOT end of packet
        // write!(cx.local.uart, "C").ok();
        // write!(cx.local.uart, "B").ok();

        if *cx.local.test {
            cx.local.led0.set_low();
        } else {
            cx.local.led0.set_high();
        }
        *cx.local.test = !*cx.local.test;
    }
}

#[panic_handler]
fn p(_: &PanicInfo) -> ! {
    loop {}
}
