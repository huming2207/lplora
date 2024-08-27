use stm32wlxx_hal::{
    spi::{SgMiso, SgMosi},
    subghz::{self, Ocp, PaConfig, PaSel, RampTime, RegMode, StandbyClk, SubGhz, TxParams},
};

use super::{uart_pkt_decoder::UartPacketDecoder, UartPacketError};

pub struct RadioPhyConfigurator {
    tx_params: TxParams,
    pa_config: PaConfig,
    ocp: Ocp,
    rx_boost: bool,
}

impl TryFrom<UartPacketDecoder> for RadioPhyConfigurator {
    type Error = UartPacketError;

    fn try_from(value: UartPacketDecoder) -> Result<Self, Self::Error> {
        let (buf, len) = value.get_payload();

        if len < 6 {
            defmt::error!("RadioPhyConfigurator: require 6 bytes while got {} bytes", len);
            return Err(UartPacketError::CorruptedError);
        }

        // First 3 bytes are PA config
        let pa_sel = if buf[2] == 0 { PaSel::Lp } else { PaSel::Hp };

        let ocp = if buf[2] == 0 { Ocp::Max60m } else { Ocp::Max140m };

        let pa_config = PaConfig::new()
            .set_pa_duty_cycle(buf[0])
            .set_hp_max(buf[1])
            .set_pa(pa_sel);

        let ramp_time: RampTime = match buf[4] {
            0x00 => RampTime::Micros10,
            0x01 => RampTime::Micros20,
            0x02 => RampTime::Micros40,
            0x03 => RampTime::Micros80,
            0x04 => RampTime::Micros200,
            0x05 => RampTime::Micros800,
            0x06 => RampTime::Micros1700,
            0x07 => RampTime::Micros3400,
            _ => {
                defmt::error!("RadioGfskConfigurator: invalid RampTime: 0x{:x}", buf[4]);
                return Err(UartPacketError::CorruptedError);
            }
        };

        let tx_params = TxParams::new().set_ramp_time(ramp_time).set_power(buf[3]);
        let rx_boost = buf[5] != 0;
        Ok(RadioPhyConfigurator {
            tx_params,
            pa_config,
            ocp,
            rx_boost,
        })
    }
}

impl RadioPhyConfigurator {
    pub fn configure_radio(&self, radio: &mut SubGhz<SgMiso, SgMosi>) -> Result<(), subghz::Error> {
        radio.set_standby(StandbyClk::Rc)?;
        radio.set_buffer_base_address(0, 0)?;
        radio.set_regulator_mode(RegMode::Smps)?;
        radio.set_pa_config(&self.pa_config)?;
        radio.set_pa_ocp(self.ocp)?;
        radio.set_tx_params(&self.tx_params)?;

        if self.rx_boost {
            radio.set_rx_gain(subghz::PMode::Boost2)?;
        } else {
            radio.set_rx_gain(subghz::PMode::PowerSaving)?;
        }

        defmt::info!("RadioPhyConfigurator: config OK, PA config: {:?}, OCP: {:?}, TxParams: {:?}", self.pa_config, self.ocp, self.tx_params);
        Ok(())
    }
}
