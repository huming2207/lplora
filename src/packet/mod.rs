use core::cmp;

use stm32wlxx_hal::subghz::LoRaPacketStatus;

use crate::constants::{CacheQueue, CRC, SLIP_END, SLIP_ESC, SLIP_ESC_END, SLIP_ESC_ESC, SLIP_ESC_START, SLIP_START};

pub mod uart_packet;

pub fn enqueue_radio_recv_pkt(pkt_status: LoRaPacketStatus, buf: &[u8], queue: &mut CacheQueue, data_len: u8) {
    defmt::info!("radio: RxDone, got {:?}; len={}", pkt_status, data_len);
    let mut digest = CRC.digest();
    match queue.enqueue(SLIP_START) {
        Ok(_) => {},
        Err(b) => {
            defmt::warn!("radio: Rx buffer full! Ditching oldest");
            queue.dequeue(); // Drop the oldest
            queue.enqueue(b).unwrap();
        }
    }

    let pkt_rssi = cmp::max(pkt_status.signal_rssi_pkt().to_integer(), 0);
    digest.update(&[(pkt_rssi * -1) as u8]);

    let snr = cmp::max(pkt_status.snr_pkt().to_integer(), 0);
    digest.update(&[(snr * -1) as u8]);

    for b in buf {
        digest.update(&[*b]);
    }

    let checksum: [u8; 2] = digest.finalize().to_le_bytes();
    let pkt_len: [u8; 2] = ((checksum.len() + data_len as usize + 2) as u16).to_le_bytes();
    slip_enqueue(queue, pkt_len[0]);
    slip_enqueue(queue, pkt_len[1]);
    slip_enqueue(queue, checksum[0]);
    slip_enqueue(queue, checksum[1]);
    
    slip_enqueue(queue, (pkt_rssi * -1) as u8);
    slip_enqueue(queue, (snr * -1) as u8);

    for b in buf {
        slip_enqueue(queue, *b);
    }

    match queue.enqueue(SLIP_END) {
        Ok(_) => {},
        Err(b) => {
            defmt::warn!("radio: Rx buffer full! Ditching oldest");
            queue.dequeue(); // Drop the oldest
            queue.enqueue(b).unwrap();
        }
    }

}

pub fn parse_uart_packet(queue: &mut CacheQueue) -> Result<(), ()> {
    Ok(())
}

fn enqueue_ditch_oldest(queue: &mut CacheQueue, b: u8) {
    match queue.enqueue(SLIP_START) {
        Ok(_) => {},
        Err(b) => {
            queue.dequeue(); // Drop the oldest
            queue.enqueue(b).unwrap();
        }
    }
}

fn slip_enqueue(queue: &mut CacheQueue, b: u8) {
    match b {
        SLIP_START => {
            enqueue_ditch_oldest(queue, SLIP_ESC);
            enqueue_ditch_oldest(queue, SLIP_ESC_START);
        },
        SLIP_ESC => {
            enqueue_ditch_oldest(queue, SLIP_ESC);
            enqueue_ditch_oldest(queue, SLIP_ESC_ESC);
        },
        SLIP_END => {
            enqueue_ditch_oldest(queue, SLIP_ESC);
            enqueue_ditch_oldest(queue, SLIP_ESC_END);
        },
        _ => {
            enqueue_ditch_oldest(queue, b);
        }
    }
}

fn slip_dequeue(queue: &mut CacheQueue, out: &mut [u8]) -> Result<(), ()> {
    let mut begin: bool = false;
    let mut ctr: usize = 0;

    loop {
        let dequeued_byte = match queue.dequeue() {
            Some(b) => b,
            None => break,
        };

        match dequeued_byte {
            SLIP_START => {
                if !begin {
                    begin = true;
                } else {
                    // Something fucked, handle error
                    defmt::error!("slip_decode: Start byte occur twice!");
                }
            },

            SLIP_END => {
                if begin {
                    begin = false;
                    break;
                }
            },

            SLIP_ESC => {
                let esc = match queue.dequeue() {
                    Some(b) => b,
                    None => break,
                };

                match esc {
                    SLIP_ESC_END => {
                        out[ctr] = SLIP_END;
                        ctr += 1;
                    },
                    SLIP_ESC_START => {
                        out[ctr] = SLIP_START;
                        ctr += 1;
                    },
                    SLIP_ESC_ESC => {
                        out[ctr] = SLIP_ESC;
                        ctr += 1;
                    },
                    _ => {
                        defmt::error!("slip_decode: unknown ESC byte {:02x}", esc);
                        break;
                    }
                }
            }

            _ => {
                out[ctr] = dequeued_byte;
                ctr += 1;
            }
        }

        if out.len() <= ctr {
            defmt::warn!("slip_decode: output buffer full!");
            break;
        }
    }

    if !begin {
        Ok(())
    } else {
        Err(())
    }
}

