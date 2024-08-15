#![no_main]
#![no_std]

use lplora as _; // global logger + panicking-behavior + memory layout

// TODO(7) Configure the `rtic::app` macro
#[rtic::app(
    // TODO: Replace `some_hal::pac` with the path to the PAC
    device = stm32wlxx_hal::pac,
    dispatchers = [DAC, USART2]
)]
mod app {
    use cortex_m::interrupt::CriticalSection;
    use cortex_m::prelude::*;
    use heapless::spsc::Queue;
    use lplora::constants::{CacheQueue, RFSW_GPIO_OUTPUT_ARGS, SLIP_END, SLIP_ESC, SLIP_START};
    use lplora::radio::{handle_radio_rx_done, start_radio_rx, start_radio_tx};
    use stm32wlxx_hal::gpio::pins::{B8, C13};
    use stm32wlxx_hal::spi::{SgMiso, SgMosi};
    use stm32wlxx_hal::subghz::{Irq, LoRaPacketStatus, SubGhz, Timeout};
    use stm32wlxx_hal::{
        gpio::{pins, Output, PortA, PortB, PortC},
        pac::Peripherals,
        rcc, uart,
        uart::{LpUart, NoTx},
    };

    // Shared resources go here
    #[shared]
    struct Shared {
        uart_tx_q: CacheQueue,
        radio_tx_q: CacheQueue,
        uart_rx_q: CacheQueue,
        radio_rx_q: CacheQueue,
        uart: LpUart<pins::A3, pins::A2>,
    }

    // Local resources go here
    #[local]
    struct Local {
        radio: SubGhz<SgMiso, SgMosi>,
        rf_sw_1: Output<B8>,
        rf_sw_2: Output<C13>,
    }

    #[init]
    fn init(ctx: init::Context) -> (Shared, Local) {
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
            LpUart::new(dp.LPUART, 9600, uart::Clk::Lse, &mut dp.RCC)
                .enable_rx(io_a.a3, cs)
                .enable_tx(io_a.a2, cs);

        // Set up RF Switch GPIOs
        let mut rf_sw_1 = Output::new(io_b.b8, &RFSW_GPIO_OUTPUT_ARGS, cs);
        let mut rf_sw_2 = Output::new(io_c.c13, &RFSW_GPIO_OUTPUT_ARGS, cs);

        let mut uart_tx_q: CacheQueue = Queue::new();
        let mut uart_rx_q: CacheQueue = Queue::new();
        let mut  radio_tx_q: CacheQueue = Queue::new();
        let mut radio_rx_q: CacheQueue = Queue::new();

        let mut radio = SubGhz::new(dp.SPI3, &mut dp.RCC);

        start_radio_rx(&mut radio, 5000).unwrap();

        (
            Shared {
                uart_tx_q,
                radio_tx_q,
                uart_rx_q,
                radio_rx_q,

                uart,
            },
            Local {
                radio,
                rf_sw_1,
                rf_sw_2,
            },
        )
    }

    #[task(binds = LPUART1, shared = [uart_rx_q, uart])]
    fn uart_task(mut ctx: uart_task::Context) {
        let recv_byte = ctx.shared.uart.lock(|uart| {
            uart.read().unwrap()
        });

        let mut packet_ended: bool = false;
        ctx.shared.uart_rx_q.lock(|queue| match recv_byte {
            SLIP_START => {
                defmt::info!("UART packet started");
                while !queue.is_empty() {
                    queue.dequeue().unwrap();
                }

                queue.enqueue(recv_byte).unwrap();
            }
            SLIP_END => {
                defmt::info!("UART packet ended");
                queue.enqueue(recv_byte).unwrap();
                packet_ended = true;
            }
            _ => {
                queue.enqueue(recv_byte).unwrap();
            }
        });

        if packet_ended {
            
        }
    }

    #[task(binds = RADIO_IRQ_BUSY, local = [radio, rf_sw_1, rf_sw_2, was_tx: bool = false], shared = [uart_tx_q])]
    fn radio_task(mut ctx: radio_task::Context) {
        let radio = ctx.local.radio;
        let rfsw_1 = ctx.local.rf_sw_1;
        let rfsw_2 = ctx.local.rf_sw_2;
        let was_tx = ctx.local.was_tx;
        let (_, irq) = radio.irq_status().unwrap();
        radio.clear_irq_status(irq).unwrap();

        if irq & Irq::Timeout.mask() != 0 { 
            if *was_tx {
                defmt::error!("radio: TxTimeout! Something fucked?");
            } else {
                defmt::info!("radio: RxTimeout! Re-enter Rx");
                rfsw_1.set_level_high();
                rfsw_2.set_level_low();
                start_radio_rx(radio, 5000).unwrap();
            }
        } else if irq & Irq::RxDone.mask() != 0 {
            defmt::info!("radio: RxDone, handling...");
            rfsw_1.set_level_low();
            rfsw_2.set_level_low();
            ctx.shared.uart_tx_q.lock(|q| {
                handle_radio_rx_done(radio, irq, q).unwrap();
            });

            // TODO: probably spawn UART Tx here
            uart_tx::spawn().unwrap();

            // ...and then go back to Rx?
            rfsw_1.set_level_high();
            rfsw_2.set_level_low();
            start_radio_rx(radio, 5000).unwrap();
        } else if irq & Irq::TxDone.mask() != 0 {
            defmt::info!("radio: TxDone, re-enter Rx");
            rfsw_1.set_level_high();
            rfsw_2.set_level_low();
            start_radio_rx(radio, 5000).unwrap();
        }
    }

    #[task(priority = 2, shared = [uart_tx_q, uart])]
    async fn uart_tx(ctx: uart_tx::Context) {
        let queue = ctx.shared.uart_tx_q;
        let uart = ctx.shared.uart;
        (queue, uart).lock(|q, uart| {
            let tx_byte = match q.dequeue() {
                Some(b) => b,
                None => return,
            };

            loop {
                match uart.write(tx_byte) {
                    Ok(_) => break,
                    Err(_) => continue,
                }
            }
        });
    }

    #[task(priority = 2)]
    async fn radio_ctrl(mut ctx: radio_ctrl::Context) {}

    // Optional idle, can be removed if not needed.
    #[idle]
    fn idle(_: idle::Context) -> ! {
        defmt::info!("idle");

        loop {
            continue;
        }
    }
}
