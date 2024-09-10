#![no_main]
#![no_std]

static mut TX_COUNTER: u32 = 0;

use lplora as _; // global logger + panicking-behavior + memory layout

// TODO(7) Configure the `rtic::app` macro
#[rtic::app(
    // TODO: Replace `some_hal::pac` with the path to the PAC
    device = stm32wlxx_hal::pac,
    dispatchers = [DAC, USART2, USART1],
    peripherals = true,
)]
mod app {
    use cortex_m::asm::wfi;
    use cortex_m::interrupt::CriticalSection;
    use heapless::spsc::Queue;
    use lplora::constants::{CacheQueue, RFSW_GPIO_OUTPUT_ARGS};
    use lplora::radio::{handle_radio_rx_done, setup_radio, start_radio_rx, start_radio_tx};
    use lplora::uid::read_uid;
    use stm32wlxx_hal::gpio::pins::{B8, C13};
    use stm32wlxx_hal::pwr::{enter_lprun_msi, LprunRange};
    use stm32wlxx_hal::spi::{SgMiso, SgMosi};
    use stm32wlxx_hal::subghz::{Irq, SubGhz};
    use stm32wlxx_hal::{
        gpio::{Output, PortB, PortC},
        rcc
    };

    use crate::TX_COUNTER;

    // Shared resources go here
    #[shared]
    struct Shared {
        #[lock_free]
        uart_tx_q: CacheQueue,

        #[lock_free]
        uart_rx_q: CacheQueue,
    }

    // Local resources go here
    #[local]
    struct Local {
        rf_sw_1: Output<B8>,
        rf_sw_2: Output<C13>,
        radio: SubGhz<SgMiso, SgMosi>,
    }

    #[init]
    fn init(ctx: init::Context) -> (Shared, Local) {
        defmt::info!("Init begin");

        let mut dp = ctx.device;
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

        let io_b: PortB = PortB::split(dp.GPIOB, &mut dp.RCC);
        let io_c: PortC = PortC::split(dp.GPIOC, &mut dp.RCC);

        // Enable debug domain in STOP mode
        if cfg!(debug_assertions) {
            defmt::info!("Enable debug at STOP mode");
            dp.DBGMCU.cr.modify(|_, w| w.dbg_stop().set_bit());
        } else {
            dp.DBGMCU.cr.modify(|_, w| w.dbg_stop().clear_bit());
        }

        // Set up RF Switch GPIOs
        let mut rf_sw_1 = Output::new(io_b.b8, &RFSW_GPIO_OUTPUT_ARGS, cs);
        let mut rf_sw_2 = Output::new(io_c.c13, &RFSW_GPIO_OUTPUT_ARGS, cs);

        let uart_tx_q: CacheQueue = Queue::new();
        let uart_rx_q: CacheQueue = Queue::new();

        let mut radio = SubGhz::new(dp.SPI3, &mut dp.RCC);
        setup_radio(&mut radio).unwrap();

        rf_sw_1.set_level_low();
        rf_sw_2.set_level_high();
        let mut tx_buf: [u8; 24] = [0; 24];
        let uid = read_uid();
        tx_buf[..20].copy_from_slice(&uid);
        unsafe { tx_buf[20..].copy_from_slice(&TX_COUNTER.to_le_bytes()); }

        start_radio_tx(&mut radio, &tx_buf, 5000).unwrap();

        cortex_m::interrupt::free(|cs| unsafe {
            enter_lprun_msi(&mut dp.FLASH, &mut dp.PWR, &mut dp.RCC, LprunRange::Range1M, cs)
        });

        defmt::info!("Init setup complete!");

        (
            Shared { uart_tx_q, uart_rx_q },
            Local {
                rf_sw_1,
                rf_sw_2,
                radio,
            },
        )
    }

    #[task(binds = RADIO_IRQ_BUSY, local = [was_tx: bool = false, radio, rf_sw_1, rf_sw_2], shared = [uart_tx_q])]
    fn radio_task(ctx: radio_task::Context) {
        let mut radio = ctx.local.radio;
        let rf_sw_1 = ctx.local.rf_sw_1;
        let rf_sw_2 = ctx.local.rf_sw_2;

        let was_tx = ctx.local.was_tx;
        let (_, irq) = radio.irq_status().unwrap();
        radio.clear_irq_status(irq).unwrap();

        if irq & Irq::Timeout.mask() != 0 {
            if *was_tx {
                defmt::error!("radio: TxTimeout! Something fucked?");
            } else {
                defmt::info!("radio: RxTimeout! Re-enter Tx");
                rf_sw_1.set_level_low();
                rf_sw_2.set_level_high();
                let mut tx_buf: [u8; 24] = [0; 24];
                let uid = read_uid();
                tx_buf[..20].copy_from_slice(&uid);
                unsafe { tx_buf[20..].copy_from_slice(&TX_COUNTER.to_le_bytes()); }
    
                start_radio_tx(&mut radio, &tx_buf, 5000).unwrap();
            }
        } else if irq & Irq::RxDone.mask() != 0 {
            defmt::info!("radio: RxDone, handling...");
            let uart_tx_queue = ctx.shared.uart_tx_q;
            handle_radio_rx_done(radio, irq, uart_tx_queue).unwrap();

            rf_sw_1.set_level_low();
            rf_sw_2.set_level_high();
            let mut tx_buf: [u8; 24] = [0; 24];
            let uid = read_uid();
            tx_buf[..20].copy_from_slice(&uid);
            unsafe { tx_buf[20..].copy_from_slice(&TX_COUNTER.to_le_bytes()); }

            start_radio_tx(&mut radio, &tx_buf, 5000).unwrap();
        } else if irq & Irq::TxDone.mask() != 0 {
            unsafe { 
                TX_COUNTER += 1; 
                defmt::info!("radio: TxDone, re-enter Rx, ctr={}", TX_COUNTER);
            }
            
            rf_sw_1.set_level_high();
            rf_sw_2.set_level_low();

            start_radio_rx(radio, 15000).unwrap();
        } else {
            // Nothing in IRQ reading?? Maybe this is a manual triggered one?
            defmt::warn!("SubGhz IRQ triggered while nothing needed?");
        }
    }

    // Optional idle, can be removed if not needed.
    #[idle]
    fn idle(_: idle::Context) -> ! {
        loop {
            //enter_stop2_mode();
            wfi();
            continue;
        }
    }
}
