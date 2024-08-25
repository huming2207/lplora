use stm32wlxx_hal::{
    spi::{SgMiso, SgMosi},
    subghz::{self, SubGhz},
};

use crate::{packet::UartPacketError, radio::start_radio_rx};

use super::uart_pkt_decoder::UartPacketDecoder;

pub struct RadioRxCommand {
    timeout_ms: u32,
}

impl TryFrom<UartPacketDecoder> for RadioRxCommand {
    type Error = UartPacketError;

    fn try_from(value: UartPacketDecoder) -> Result<Self, Self::Error> {
        let (buf, len) = value.get_payload();

        if len < 4 {
            defmt::error!("RadioRxCommand: require 4 bytes while got {} bytes", len);
            return Err(UartPacketError::CorruptedError);
        }

        let timeout_ms = u32::from_le_bytes(buf[0..=3].try_into().unwrap());

        Ok(RadioRxCommand { timeout_ms })
    }
}

impl RadioRxCommand {
    pub fn configure_radio(&self, radio: &mut SubGhz<SgMiso, SgMosi>) -> Result<(), subghz::Error> {
        start_radio_rx(radio, self.timeout_ms)?;
        Ok(())
    }
}
