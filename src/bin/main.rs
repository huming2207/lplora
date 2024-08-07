#![no_main]
#![no_std]

use lplora as _; // global logger + panicking-behavior + memory layout

// TODO(7) Configure the `rtic::app` macro
#[rtic::app(
    // TODO: Replace `some_hal::pac` with the path to the PAC
    device = stm32wlxx_hal::pac,
)]
mod app {
    use cortex_m::interrupt::CriticalSection;
    use stm32wlxx_hal::{gpio::{pins, Output, PortA, PortB, PortC}, pac::Peripherals, rcc, uart::{LpUart, NoTx}, uart};
    use stm32wlxx_hal::gpio::pins::{B8, C13};
    use lplora::constants::RFSW_GPIO_OUTPUT_ARGS;

    // Shared resources go here
    #[shared]
    struct Shared {
        
    }

    // Local resources go here
    #[local]
    struct Local {
        uart: LpUart<pins::A3, pins::A2>,
        rf_sw_1: Output<B8>,
        rf_sw_2: Output<C13>,
    }

    #[init]
    fn init(cx: init::Context) -> (Shared, Local) {
        defmt::info!("init");

        let mut dp = Peripherals::take().unwrap();
        let cs = unsafe { &CriticalSection::new() };

        unsafe {
            rcc::set_sysclk_msi(
                &mut dp.FLASH,
                &mut dp.PWR,
                &mut dp.RCC,
                rcc::MsiRange::Range16M, // Maybe we should consider 16MHz here so that no flash wait cycle needed??
                cs,
            );
        }

        // Enable HSE, no TCXO for now cuz we don't need that
        if dp.RCC.cr.read().hserdy().bit_is_clear() {
            dp.RCC.cr.write(|w| w.hseon().set_bit());
            while dp.RCC.cr.read().hserdy().bit_is_clear() {}
        }

        let io_a: PortA = PortA::split(dp.GPIOA, &mut dp.RCC);
        let io_b: PortB = PortB::split(dp.GPIOB, &mut dp.RCC);
        let io_c: PortC = PortC::split(dp.GPIOC, &mut dp.RCC);

        // Enable LSE
        dp.RCC
            .bdcr
            .modify(|_, w| w.lseon().on().lsesysen().enabled());
        while dp.RCC.bdcr.read().lserdy().is_not_ready() {}
        while dp.RCC.bdcr.read().lsesysrdy().is_not_ready() {}

        let mut uart: LpUart<pins::A3, pins::A2> =
            LpUart::new(dp.LPUART, 9600, uart::Clk::Lse, &mut dp.RCC).enable_rx(io_a.a3, cs).enable_tx(io_a.a2, cs);

        // Set up RF Switch GPIOs
        let mut rf_sw_1 = Output::new(io_b.b8, &RFSW_GPIO_OUTPUT_ARGS, cs);
        let mut rf_sw_2 = Output::new(io_c.c13, &RFSW_GPIO_OUTPUT_ARGS, cs);

        (
            Shared {
                // Initialization of shared resources go here
            },
            Local {
                uart,
                rf_sw_1,
                rf_sw_2,
            },
        )
    }

    #[task(binds = LPUART1, local = [uart])]
    fn uart_task(ctx: uart_task::Context) {

    }

    #[task(binds = RADIO_IRQ_BUSY)]
    fn radio_task(ctx: radio_task::Context) {

    }

    // Optional idle, can be removed if not needed.
    #[idle]
    fn idle(_: idle::Context) -> ! {
        defmt::info!("idle");

        loop {
            continue;
        }
    }
}
