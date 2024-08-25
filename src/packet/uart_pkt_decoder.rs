use crate::constants::CacheQueue;

use super::{slip_dequeue, UartPacketError, UartPacketType, CRC};

#[derive(Debug, PartialEq, Eq, Clone, Copy, defmt::Format)]
pub struct UartPacketDecoder {
    pkt_type: UartPacketType,
    curr_payload_len: u16,
    payload_buf: [u8; 300],
}

impl UartPacketDecoder {
    pub fn new(queue: &mut CacheQueue) -> Result<UartPacketDecoder, UartPacketError> {
        let mut buf: [u8; 300] = [0; 300];
        let mut decoded_len: usize = 0;
        slip_dequeue(queue, &mut buf, &mut decoded_len)?;

        let pkt_type = UartPacketType::try_from(buf[0])?;
        let payload_len_bytes: [u8; 2] = [buf[0], buf[1]];
        let curr_payload_len = u16::from_le_bytes(payload_len_bytes);
        if buf.len() < curr_payload_len as usize {
            defmt::error!("UartPacketDecoder: invalid length: {}", curr_payload_len);
            return Err(UartPacketError::CorruptedError);
        }

        let crc_bytes: [u8; 2] = [
            buf[(decoded_len - 2) as usize],
            buf[(decoded_len - 1) as usize],
        ];
        let expected_crc = u16::from_le_bytes(crc_bytes);
        let mut digest = CRC.digest();
        digest.update(&buf[0..(decoded_len - 2) as usize]);
        let actual_crc = digest.finalize();
        if actual_crc != expected_crc {
            defmt::error!(
                "UartPacketDecoder: invalid CRC: 0x{:04x} vs. 0x{:04x}",
                expected_crc,
                actual_crc
            );
            return Err(UartPacketError::CorruptedError);
        }

        if curr_payload_len > (buf.len() as u16) {
            return Err(UartPacketError::BufferFullError);
        }

        Ok(UartPacketDecoder {
            pkt_type,
            curr_payload_len,
            payload_buf: buf,
        })
    }

    pub fn get_type(&self) -> UartPacketType {
        self.pkt_type
    }

    pub fn get_payload(&self) -> (&[u8], u16) {
        (self.payload_buf.as_slice(), self.curr_payload_len)
    }

    pub fn get_payload_len(&self) -> u16 {
        self.curr_payload_len
    }
}
