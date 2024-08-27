use stm32wlxx_hal::{
    spi::{SgMiso, SgMosi},
    subghz::{self, CalibrateImage, RfFreq, SubGhz},
};

use super::{uart_pkt_decoder::UartPacketDecoder, UartPacketError};

pub struct RadioFreqConfigurator {
    freq_hz: u32,
}

impl TryFrom<UartPacketDecoder> for RadioFreqConfigurator {
    type Error = UartPacketError;

    fn try_from(value: UartPacketDecoder) -> Result<Self, Self::Error> {
        let (buf, len) = value.get_payload();

        if len < 4 {
            defmt::error!("RadioFreqConfigurator: require 4 bytes while got {} bytes", len);
            return Err(UartPacketError::CorruptedError);
        }

        let freq_hz = u32::from_le_bytes(buf[0..=3].try_into().unwrap());
        if freq_hz < (100 * 1000000) || freq_hz > (960 * 1000000) {
            defmt::error!("RadioFreqConfigurator: frequency out of range! freq_hz={}; 0x{:x} 0x{:x} 0x{:x} 0x{:x}", freq_hz, buf[0], buf[1], buf[2], buf[3]);
            return Err(UartPacketError::CorruptedError);
        }

        Ok(RadioFreqConfigurator { freq_hz })
    }
}

impl RadioFreqConfigurator {
    pub fn configure_radio(&self, radio: &mut SubGhz<SgMiso, SgMosi>) -> Result<(), subghz::Error> {
        radio.set_rf_frequency(&RfFreq::from_frequency(self.freq_hz))?;

        let mhz = self.freq_hz / 1000000;
        let freqx4 = mhz - (mhz % 4);
        radio.calibrate_image(CalibrateImage::from_freq((freqx4 - 4) as u16, (freqx4 + 4) as u16))?;

        defmt::info!("RadioFreqConfigurator: config OK, freq={:?}", self.freq_hz);
        Ok(())
    }
}
