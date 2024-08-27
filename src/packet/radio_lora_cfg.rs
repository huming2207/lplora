use stm32wlxx_hal::{
    spi::{SgMiso, SgMosi},
    subghz::{
        self, CodingRate, HeaderType, LoRaBandwidth, LoRaModParams, LoRaPacketParams, LoRaSyncWord, PacketType,
        SpreadingFactor, StandbyClk, SubGhz,
    },
};

use super::{uart_pkt_decoder::UartPacketDecoder, UartPacketError};

pub struct RadioLoraConfigurator {
    pkt_params: LoRaPacketParams,
    lora_mod: LoRaModParams,
    sync_word: [u8; 2],
}

impl TryFrom<UartPacketDecoder> for RadioLoraConfigurator {
    type Error = UartPacketError;

    fn try_from(value: UartPacketDecoder) -> Result<Self, Self::Error> {
        let (buf, len) = value.get_payload();

        if len < 12 {
            defmt::error!("RadioLoraConfigurator: require 12 bytes while got {} bytes", len);
            return Err(UartPacketError::CorruptedError);
        }

        let preamble_len = u16::from_le_bytes(buf[0..=1].try_into().unwrap());
        let header_type = if buf[2] == 0 {
            HeaderType::Fixed
        } else {
            HeaderType::Variable
        };

        let pkt_params = LoRaPacketParams::new()
            .set_preamble_len(preamble_len)
            .set_header_type(header_type)
            .set_payload_len(buf[3])
            .set_crc_en(buf[4] != 0)
            .set_invert_iq(buf[5] != 0);

        let sf: SpreadingFactor = match buf[6] {
            0x05 => SpreadingFactor::Sf5,
            0x06 => SpreadingFactor::Sf6,
            0x07 => SpreadingFactor::Sf7,
            0x08 => SpreadingFactor::Sf8,
            0x09 => SpreadingFactor::Sf9,
            0x0A => SpreadingFactor::Sf10,
            0x0B => SpreadingFactor::Sf11,
            0x0C => SpreadingFactor::Sf12,
            _ => {
                defmt::error!("RadioLoraConfigurator: invalid SF: 0x{:x}", buf[6]);
                return Err(UartPacketError::CorruptedError);
            }
        };

        let bw: LoRaBandwidth = match buf[7] {
            0x00 => LoRaBandwidth::Bw7,
            0x08 => LoRaBandwidth::Bw10,
            0x01 => LoRaBandwidth::Bw15,
            0x09 => LoRaBandwidth::Bw20,
            0x02 => LoRaBandwidth::Bw31,
            0x0A => LoRaBandwidth::Bw41,
            0x03 => LoRaBandwidth::Bw62,
            0x04 => LoRaBandwidth::Bw125,
            0x05 => LoRaBandwidth::Bw250,
            0x06 => LoRaBandwidth::Bw500,
            _ => {
                defmt::error!("RadioLoraConfigurator: invalid BW: 0x{:x}", buf[7]);
                return Err(UartPacketError::CorruptedError);
            }
        };

        let cr: CodingRate = match buf[8] {
            0x00 => CodingRate::Cr44,
            0x01 => CodingRate::Cr45,
            0x02 => CodingRate::Cr46,
            0x03 => CodingRate::Cr47,
            0x04 => CodingRate::Cr48,
            _ => {
                defmt::error!("RadioLoraConfigurator: invalid CR: 0x{:x}", buf[8]);
                return Err(UartPacketError::CorruptedError);
            }
        };

        let lora_mod = LoRaModParams::new()
            .set_sf(sf)
            .set_bw(bw)
            .set_cr(cr)
            .set_ldro_en(buf[9] != 0);
        let sync_word: [u8; 2] = buf[10..=11].try_into().unwrap();

        Ok(RadioLoraConfigurator {
            lora_mod,
            pkt_params,
            sync_word,
        })
    }
}

impl RadioLoraConfigurator {
    pub fn configure_radio(&self, radio: &mut SubGhz<SgMiso, SgMosi>) -> Result<(), subghz::Error> {
        radio.set_standby(StandbyClk::Rc)?;
        radio.set_packet_type(PacketType::LoRa)?;
        radio.set_lora_sync_word(LoRaSyncWord::Custom(self.sync_word))?;
        radio.set_lora_mod_params(&self.lora_mod)?;
        radio.set_lora_packet_params(&self.pkt_params)?;

        defmt::info!("RadioLoraConfigurator: LoRa config OK, sync word {:x} {:x}, mod: {:?}", self.sync_word[0], self.sync_word[1], self.lora_mod);
        Ok(())
    }
}
