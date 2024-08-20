use stm32wlxx_hal::{
    spi::{SgMiso, SgMosi},
    subghz::{
        AddrComp, CrcType, Error, FskBandwidth, FskBitrate, FskFdev, FskModParams, FskPulseShape, GenericPacketParams,
        HeaderType, PacketType, PreambleDetection, StandbyClk, SubGhz,
    },
};

use super::{uart_pkt_decoder::UartPacketDecoder, UartPacketError};

pub struct RadioGfskConfigurator {
    pkt_params: GenericPacketParams,
    fsk_mod: FskModParams,
    sync_word: [u8; 8],
}

impl TryFrom<UartPacketDecoder> for RadioGfskConfigurator {
    type Error = UartPacketError;

    fn try_from(value: UartPacketDecoder) -> Result<Self, Self::Error> {
        let (buf, len) = value.get_payload();

        // First 9 bytes are FSK Packet parameters,
        // then next 10 bytes are FSK modulation parameters
        // then finally 8 bytes of sync word
        if len < (9 + 10 + 8) {
            defmt::error!("RadioGfskConfigurator: invalid packet length = {}", len);
            return Err(UartPacketError::CorruptedError);
        }

        // First 9 bytes starts here for FSK Packet Parameters
        let preamble_len = ((buf[0] as u32) << 8u32) as u16 | buf[1] as u16;
        let preamble_detection: PreambleDetection = match buf[2] {
            0 => PreambleDetection::Disabled,
            4 => PreambleDetection::Bit8,
            5 => PreambleDetection::Bit16,
            6 => PreambleDetection::Bit24,
            7 => PreambleDetection::Bit32,
            _ => {
                defmt::error!("RadioGfskConfigurator: invalid preamble detection: {}", buf[2]);
                return Err(UartPacketError::CorruptedError);
            }
        };

        let sync_word_len = buf[3];
        let addr_comp: AddrComp = match buf[4] {
            0 => AddrComp::Disabled,
            1 => AddrComp::Node,
            2 => AddrComp::Broadcast,
            _ => {
                defmt::error!("RadioGfskConfigurator: invalid AddrComp: {}", buf[4]);
                return Err(UartPacketError::CorruptedError);
            }
        };

        let header_type: HeaderType = if buf[5] == 0 {
            HeaderType::Fixed
        } else {
            HeaderType::Variable
        };

        let payload_len = buf[6];
        let crc_type: CrcType = match buf[7] {
            0 => CrcType::Byte1,
            1 => CrcType::Disabled,
            2 => CrcType::Byte2,
            4 => CrcType::Byte1Inverted,
            6 => CrcType::Byte2Inverted,
            _ => {
                defmt::error!("RadioGfskConfigurator: invalid CrcType: {}", buf[7]);
                return Err(UartPacketError::CorruptedError);
            }
        };

        let whiten_enable = buf[8] != 0;

        let pkt_params = GenericPacketParams::new()
            .set_preamble_len(preamble_len)
            .set_preamble_detection(preamble_detection)
            .set_sync_word_len(sync_word_len)
            .set_addr_comp(addr_comp)
            .set_header_type(header_type)
            .set_payload_len(payload_len)
            .set_crc_type(crc_type)
            .set_whitening_enable(whiten_enable);

        // Then the 10th bytes to 19th bytes starts from here for FSK modulation paramaters
        let bitrate: u32 = u32::from_le_bytes(buf[9..=12].try_into().unwrap());
        let pulse_shape: FskPulseShape = match buf[13] {
            0 => FskPulseShape::None,
            0x08 => FskPulseShape::Bt03,
            0x09 => FskPulseShape::Bt05,
            0x0A => FskPulseShape::Bt07,
            0x0B => FskPulseShape::Bt10,
            _ => {
                defmt::error!("RadioGfskConfigurator: invalid FskPulseShape: {}", buf[13]);
                return Err(UartPacketError::CorruptedError);
            }
        };

        let bandwidth: FskBandwidth = match FskBandwidth::from_bits(buf[14]) {
            Ok(val) => val,
            Err(err) => {
                defmt::error!("RadioGfskConfigurator: invalid FskBandwidth: 0x{:x}", err);
                return Err(UartPacketError::CorruptedError);
            }
        };

        let fdev = u32::from_le_bytes(buf[15..=18].try_into().unwrap());

        let fsk_mod = FskModParams::new()
            .set_bitrate(FskBitrate::from_bps(bitrate))
            .set_pulse_shape(pulse_shape)
            .set_bandwidth(bandwidth)
            .set_fdev(FskFdev::from_hertz(fdev));

        let sync_word: [u8; 8] = buf[19..=26].try_into().unwrap();
        return Ok(RadioGfskConfigurator {
            pkt_params,
            fsk_mod,
            sync_word,
        });
    }
}

impl RadioGfskConfigurator {
    pub fn configure_radio(&self, radio: &mut SubGhz<SgMiso, SgMosi>) -> Result<(), Error> {
        // Do we need to reset...?
        // let dp = unsafe { Peripherals::steal() };
        // dp.RCC.csr.modify(|_, w| w.rfrst().set_bit());
        // dp.RCC.csr.modify(|_, w| w.rfrst().clear_bit());

        radio.set_standby(StandbyClk::Rc)?;
        radio.set_packet_type(PacketType::Fsk)?;
        radio.set_sync_word(&self.sync_word)?;
        radio.set_fsk_mod_params(&self.fsk_mod)?;
        radio.set_packet_params(&self.pkt_params)?;

        Ok(())
    }
}
