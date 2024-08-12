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
    use lplora::constants::{RFSW_GPIO_OUTPUT_ARGS, SLIP_END, SLIP_ESC, SLIP_START};
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
        uart_tx_q: Queue<u8, 1024>,
        radio_tx_q: Queue<u8, 1024>,
        uart_rx_q: Queue<u8, 1024>,
        radio_rx_q: Queue<u8, 1024>,
    }

    // Local resources go here
    #[local]
    struct Local {
        uart: LpUart<pins::A3, pins::A2>,
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

        let mut uart_tx_q: Queue<u8, 1024> = Queue::new();
        let mut uart_rx_q: Queue<u8, 1024> = Queue::new();
        let mut  radio_tx_q: Queue<u8, 1024> = Queue::new();
        let mut radio_rx_q: Queue<u8, 1024> = Queue::new();

        let mut radio = SubGhz::new(dp.SPI3, &mut dp.RCC);

        start_radio_rx(&mut radio, 5000).unwrap();

        (
            Shared {
                uart_tx_q,
                radio_tx_q,
                uart_rx_q,
                radio_rx_q,
            },
            Local {
                uart,
                radio,
                rf_sw_1,
                rf_sw_2,
            },
        )
    }

    #[task(binds = LPUART1, local = [uart], shared = [uart_rx_q])]
    fn uart_task(mut ctx: uart_task::Context) {
        let uart = ctx.local.uart;
        let recv_byte = uart.read().unwrap();

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
                uart_parser::spawn().unwrap();
            }
            _ => {
                queue.enqueue(recv_byte).unwrap();
            }
        })
    }

    #[task(binds = RADIO_IRQ_BUSY, local = [radio, rf_sw_1, rf_sw_2, was_tx: bool = false], shared = [radio_rx_q])]
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
                start_radio_rx(radio, 5000).unwrap();
            }
        } else if irq & Irq::RxDone.mask() != 0 {
            let mut buf: [u8; 256] = [0; 256];
            let dummy_status: [u8; 4] = [0; 4];
            let mut pkt_status: LoRaPacketStatus = LoRaPacketStatus::from(dummy_status);
            let mut recv_len: u8 = 0;
            defmt::info!("radio: RxDone, handling...");
            handle_radio_rx_done(radio, irq, buf.as_mut_slice(), &mut pkt_status, &mut recv_len).unwrap();
            defmt::info!("radio: RxDone, got {:?}; len={}", pkt_status, recv_len);
            ctx.shared.radio_rx_q.lock(|q| {
                for b in buf {
                    match q.enqueue(b) {
                        Ok(_) => {},
                        Err(e) => {
                            defmt::error!("radio: Rx queue full, ditching oldest...");
                            q.dequeue();
                            q.enqueue(e).unwrap();
                        }
                    }
                        
                }
            })
        } else if irq & Irq::TxDone.mask() != 0 {
            defmt::info!("radio: TxDone, re-enter Rx");
            start_radio_rx(radio, 5000).unwrap();
        }

        radio.clear_irq_status(irq).unwrap();

    }

    #[task(priority = 2, shared = [uart_tx_q])]
    async fn uart_parser(ctx: uart_parser::Context) {

    }

    #[task(priority = 2)]
    async fn radio_ctrl(ctx: radio_ctrl::Context) {}

    // Optional idle, can be removed if not needed.
    #[idle]
    fn idle(_: idle::Context) -> ! {
        defmt::info!("idle");

        loop {
            continue;
        }
    }
}
