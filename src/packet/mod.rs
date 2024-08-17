use crate::constants::{
    CacheQueue, SLIP_END, SLIP_ESC, SLIP_ESC_END, SLIP_ESC_ESC, SLIP_ESC_START, SLIP_START,
};

pub mod uart_pkt_decoder;
pub mod uart_pkt_encoder;

#[derive(Debug, PartialEq, Eq, Clone, Copy, defmt::Format)]
pub enum UartPacketError {
    ChecksumError,
    BufferFullError,
    EncodingError, // SLIP state invalid
    UnknownPacketError,
}

#[repr(u8)]
#[derive(Debug, PartialEq, Eq, Clone, Copy, defmt::Format)]
pub enum UartPacketType {
    RadioSendPacket = 0x01,
    RadioRecvLoRaPacket = 0x81,
}

impl TryFrom<u8> for UartPacketType {
    type Error = UartPacketError;

    fn try_from(value: u8) -> Result<Self, Self::Error> {
        match value {
            0x01 => Ok(Self::RadioSendPacket),
            0x81 => Ok(Self::RadioRecvLoRaPacket),
            _ => Err(UartPacketError::UnknownPacketError),
        }
    }
}

fn enqueue_ditch_oldest(queue: &mut CacheQueue, b: u8) {
    match queue.enqueue(b) {
        Ok(_) => {}
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
        }
        SLIP_ESC => {
            enqueue_ditch_oldest(queue, SLIP_ESC);
            enqueue_ditch_oldest(queue, SLIP_ESC_ESC);
        }
        SLIP_END => {
            enqueue_ditch_oldest(queue, SLIP_ESC);
            enqueue_ditch_oldest(queue, SLIP_ESC_END);
        }
        _ => {
            enqueue_ditch_oldest(queue, b);
        }
    }
}

fn slip_dequeue(queue: &mut CacheQueue, out: &mut [u8]) -> Result<(), UartPacketError> {
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
                }
            }

            SLIP_END => {
                if begin {
                    begin = false;
                    break;
                }
            }

            SLIP_ESC => {
                let esc = match queue.dequeue() {
                    Some(b) => b,
                    None => break,
                };

                match esc {
                    SLIP_ESC_END => {
                        out[ctr] = SLIP_END;
                        ctr += 1;
                    }
                    SLIP_ESC_START => {
                        out[ctr] = SLIP_START;
                        ctr += 1;
                    }
                    SLIP_ESC_ESC => {
                        out[ctr] = SLIP_ESC;
                        ctr += 1;
                    }
                    _ => {
                        defmt::error!("slip_decode: unknown ESC byte {:02x}", esc);
                        return Err(UartPacketError::EncodingError);
                    }
                }
            }

            _ => {
                out[ctr] = dequeued_byte;
                ctr += 1;
            }
        }

        if out.len() <= ctr {
            defmt::error!("slip_decode: output buffer full!");
            return Err(UartPacketError::BufferFullError);
        }
    }

    if !begin {
        Ok(())
    } else {
        Err(UartPacketError::EncodingError)
    }
}
