use core::cmp;

use crc::Digest;
use stm32wlxx_hal::subghz::LoRaPacketStatus;

use crate::constants::{CacheQueue, SLIP_END, SLIP_START};

use super::{slip_enqueue, UartPacketType, CRC};

pub struct UartPacketEncoder<'a> {
    queue: &'a mut CacheQueue,
    digest: Digest<'a, u16>,
}

impl<'a> UartPacketEncoder<'a> {
    pub fn make_ping(queue: &'a mut CacheQueue) {
        let pkt = UartPacketEncoder::new(UartPacketType::Ping, queue);
        pkt.finalize()
    }

    pub fn make_pong(queue: &'a mut CacheQueue) {
        let pkt = UartPacketEncoder::new(UartPacketType::Pong, queue);
        pkt.finalize()
    }
    
    pub fn new(pkt_type: UartPacketType, queue: &'a mut CacheQueue) -> UartPacketEncoder<'a> {
        let mut digest = CRC.digest();
        digest.update(&[pkt_type as u8]);

        match queue.enqueue(SLIP_START) {
            Ok(_) => {}
            Err(b) => {
                defmt::warn!("UartPacket: Rx buffer full! Ditching oldest");
                queue.dequeue(); // Drop the oldest
                queue.enqueue(b).unwrap();
            }
        }

        slip_enqueue(queue, pkt_type as u8);
        UartPacketEncoder { queue, digest }
    }

    pub fn add_packet_len(&mut self, pkt_len: usize) {
        let pkt_len_bytes: [u8; 2] = (pkt_len as u16).to_le_bytes();
        slip_enqueue(self.queue, pkt_len_bytes[0]);
        slip_enqueue(self.queue, pkt_len_bytes[1]);
    }

    pub fn add_payload_with_lora_status(&mut self, payload: &[u8], data_len: u8, pkt_status: LoRaPacketStatus) {
        let pkt_rssi = cmp::max(pkt_status.signal_rssi_pkt().to_integer(), 0);
        self.digest.update(&[(pkt_rssi * -1) as u8]);

        let snr = cmp::max(pkt_status.snr_pkt().to_integer(), 0);
        self.digest.update(&[(snr * -1) as u8]);

        for b in payload {
            self.digest.update(&[*b]);
        }

        self.add_packet_len((2 + data_len as usize + 2) as usize); // 2 bytes of CRC, 2 bytes of RSSI and SNR, plus data length

        slip_enqueue(self.queue, (pkt_rssi * -1) as u8);
        slip_enqueue(self.queue, (snr * -1) as u8);

        for b in payload {
            slip_enqueue(self.queue, *b);
        }
    }

    pub fn finalize(self) {
        let checksum: [u8; 2] = self.digest.finalize().to_le_bytes();
        slip_enqueue(self.queue, checksum[0]);
        slip_enqueue(self.queue, checksum[1]);

        match self.queue.enqueue(SLIP_END) {
            Ok(_) => {}
            Err(b) => {
                defmt::warn!("radio: Rx buffer full! Ditching oldest");
                self.queue.dequeue(); // Drop the oldest
                self.queue.enqueue(b).unwrap();
            }
        }
    }
}
