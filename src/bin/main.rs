#![no_main]
#![no_std]

use lplora as _; // global logger + panicking-behavior + memory layout

// TODO(7) Configure the `rtic::app` macro
#[rtic::app(
    // TODO: Replace `some_hal::pac` with the path to the PAC
    device = stm32wlxx_hal::pac,
    dispatchers = [DAC, USART2, USART1]
)]
mod app {
    use cortex_m::interrupt::CriticalSection;
    use cortex_m::prelude::*;
    use heapless::spsc::Queue;
    use lplora::constants::{CacheQueue, RFSW_GPIO_OUTPUT_ARGS, SLIP_END, SLIP_START};
    use lplora::packet::uart_pkt_decoder::UartPacketDecoder;
    use lplora::packet::uart_pkt_encoder::UartPacketEncoder;
    use lplora::packet::UartPacketType;
    use lplora::radio::{handle_radio_rx_done, start_radio_rx, start_radio_tx};
    use stm32wlxx_hal::gpio::pins::{B8, C13};
    use stm32wlxx_hal::pac::{Interrupt, NVIC};
    use stm32wlxx_hal::spi::{SgMiso, SgMosi};
    use stm32wlxx_hal::subghz::{Irq, SubGhz};
    use stm32wlxx_hal::{
        gpio::{pins, Output, PortA, PortB, PortC},
        pac::Peripherals,
        rcc, uart,
        uart::{LpUart, NoTx},
    };

    // Shared resources go here
    #[shared]
    struct Shared {
        #[lock_free]
        uart_tx_q: CacheQueue,

        #[lock_free]
        radio_tx_q: CacheQueue,

        #[lock_free]
        uart_rx_q: CacheQueue,

        #[lock_free]
        radio_rx_q: CacheQueue,
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
        dp.RCC.bdcr.modify(|_, w| w.lseon().on().lsesysen().enabled());
        while dp.RCC.bdcr.read().lserdy().is_not_ready() {}
        while dp.RCC.bdcr.read().lsesysrdy().is_not_ready() {}

        let dp_dirty = unsafe { Peripherals::steal() };
        let mut uart: LpUart<pins::A3, pins::A2> = LpUart::new(dp.LPUART, 9600, uart::Clk::Lse, &mut dp.RCC)
            .enable_rx(io_a.a3, cs)
            .enable_tx(io_a.a2, cs);

        // Enable Rx and Tx interrupt
        dp_dirty.LPUART.cr1.write(|w| {
            w.tcie().set_bit();
            w.rxneie().set_bit()
        });

        // Set up RF Switch GPIOs
        let mut rf_sw_1 = Output::new(io_b.b8, &RFSW_GPIO_OUTPUT_ARGS, cs);
        let mut rf_sw_2 = Output::new(io_c.c13, &RFSW_GPIO_OUTPUT_ARGS, cs);

        let mut uart_tx_q: CacheQueue = Queue::new();
        let mut uart_rx_q: CacheQueue = Queue::new();
        let mut radio_tx_q: CacheQueue = Queue::new();
        let mut radio_rx_q: CacheQueue = Queue::new();

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

    #[task(binds = LPUART1, shared = [uart_rx_q, uart_tx_q, radio_tx_q], local = [uart])]
    fn uart_task(ctx: uart_task::Context) {
        let uart_rx_queue = ctx.shared.uart_rx_q;
        let uart_tx_queue = ctx.shared.uart_tx_q;
        let radio_queue = ctx.shared.radio_tx_q;
        let uart = ctx.local.uart;

        let dp = unsafe { Peripherals::steal() };
        let isr = dp.LPUART.isr.read();
        if isr.pe().bit_is_set() || isr.fe().bit_is_set() || isr.ne().bit_is_set() || isr.ore().bit_is_set() {
            defmt::warn!("uart_task: LPUART_ISR indicate something screwed up: 0x{:x}", isr.bits());
            dp.LPUART.icr.write(|w| {
                w.pecf().set_bit();
                w.fecf().set_bit();
                w.ncf().set_bit();
                w.orecf().set_bit()
            });
            return;
        } else if isr.rxfne().bit_is_set() {
            defmt::info!("uart_task: LPUART_ISR RXNE set!");
            let mut packet_ended: bool = false;
            let recv_byte = uart.read().unwrap();
            match recv_byte {
                SLIP_START => {
                    defmt::info!("UART packet started");
                    while !uart_rx_queue.is_empty() {
                        uart_rx_queue.dequeue().unwrap();
                    }
    
                    uart_rx_queue.enqueue(recv_byte).unwrap();
                }
                SLIP_END => {
                    defmt::info!("UART packet ended");
                    uart_rx_queue.enqueue(recv_byte).unwrap();
                    packet_ended = true;
                }
                _ => {
                    uart_rx_queue.enqueue(recv_byte).unwrap();
                }
            }
    
            if packet_ended {
                let packet = match UartPacketDecoder::new(uart_rx_queue) {
                    Ok(p) => p,
                    Err(err) => {
                        defmt::error!("Something wrong when decode: {:?}", err);
                        return;
                    },
                };
                
                let (payload, len) = packet.get_payload();
                match packet.get_type() {
                    UartPacketType::RadioSendPacket => {
                        defmt::info!("Got RadioSendPacket, len={}", len);
                        for byte in &payload[0..(len as usize)] {
                            match radio_queue.enqueue(*byte) {
                                Ok(_) => continue,
                                Err(b) => {
                                    radio_queue.dequeue().unwrap();
                                    radio_queue.enqueue(b).unwrap();
                                },
                            }
                        }
                        NVIC::pend(Interrupt::RADIO_IRQ_BUSY);
                    }
                    UartPacketType::RadioRecvLoRaPacket => todo!(),
                    UartPacketType::Ping => {
                        UartPacketEncoder::make_pong(uart_tx_queue);
                        NVIC::pend(Interrupt::LPUART1);
                    }
                    _ => todo!(),
                }
            }
        } else if isr.tc().bit_is_set() {
            defmt::info!("uart_task: LPUART_ISR TC set!");
            match uart_tx_queue.dequeue() {
                Some(b) => uart.write(b).unwrap(),
                None => return,
            }
        } else { // Software triggered?
            defmt::info!("uart_task: Software-triggered Tx!");
            match uart_tx_queue.dequeue() {
                Some(b) => uart.write(b).unwrap(),
                None => return,
            }
        }
    }

    #[task(binds = RADIO_IRQ_BUSY, local = [radio, rf_sw_1, rf_sw_2, was_tx: bool = false], shared = [uart_tx_q, radio_tx_q])]
    fn radio_task(ctx: radio_task::Context) {
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
            let uart_tx_queue = ctx.shared.uart_tx_q;
            handle_radio_rx_done(radio, irq, uart_tx_queue).unwrap();

            // ...and then go back to Rx?
            rfsw_1.set_level_high();
            rfsw_2.set_level_low();
            start_radio_rx(radio, 5000).unwrap();
            NVIC::pend(Interrupt::LPUART1); // Let UART to send off the stuff received too
        } else if irq & Irq::TxDone.mask() != 0 {
            defmt::info!("radio: TxDone, re-enter Rx");
            rfsw_1.set_level_high();
            rfsw_2.set_level_low();
            start_radio_rx(radio, 5000).unwrap();
        } else {
            // Nothing in IRQ reading?? Maybe this is a manual triggered one?
            rfsw_1.set_level_low();
            rfsw_2.set_level_high();

            let mut tx_buf: [u8; 256] = [0; 256];
            let mut ctr: usize = 0;
            loop {
                match ctx.shared.radio_tx_q.dequeue() {
                    Some(b) => {
                        tx_buf[ctr] = b;
                        ctr += 1;
                        if ctr >= tx_buf.len() {
                            break;
                        }
                    },
                    None => break,
                };
            }

            defmt::info!("radio: Tx'ing, len={}", ctr);
            start_radio_tx(radio, &tx_buf[0..ctr], 0).unwrap();
        }
    }

    #[task(priority = 1)]
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
