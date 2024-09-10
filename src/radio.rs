use stm32wlxx_hal::{
    spi::{Error, SgMiso, SgMosi},
    subghz::{CfgIrq, CodingRate, FallbackMode, HeaderType, Irq, LoRaBandwidth, LoRaModParams, LoRaPacketParams, LoRaSyncWord, Ocp, PacketType, RegMode, SpreadingFactor, StandbyClk, SubGhz, Timeout},
};

use crate::{
    constants::CacheQueue,
    packet::{uart_pkt_encoder::UartPacketEncoder, UartPacketType},
};

const IRQ_CFG: CfgIrq = CfgIrq::new()
    .irq_enable_all(Irq::TxDone)
    .irq_enable_all(Irq::RxDone)
    .irq_enable_all(Irq::Timeout)
    .irq_enable_all(Irq::HeaderErr)
    .irq_enable_all(Irq::Err);
const TX_BUF_OFFSET: u8 = 0;
const RX_BUF_OFFSET: u8 = 0;

fn radio_encode_packet(radio: &mut SubGhz<SgMiso, SgMosi>, rx_queue: &mut CacheQueue) -> Result<(), Error> {
    let pkt_status = radio.lora_packet_status()?;

    let mut output_buf: [u8; 256] = [0; 256];
    let (_, data_len, ptr) = radio.rx_buffer_status()?;
    radio.read_buffer(ptr, &mut output_buf[0..(data_len as usize)])?;

    defmt::info!("radio: RxDone, got {:?}; len={}", pkt_status, data_len);
    let mut encoder = UartPacketEncoder::new(UartPacketType::RadioReceivedPacket, rx_queue);
    encoder.add_payload_with_lora_status(&output_buf[0..(data_len as usize)], data_len, pkt_status);
    encoder.finalize();

    Ok(())
}

const MOD_PARAMS: LoRaModParams = LoRaModParams::new()
    .set_sf(SpreadingFactor::Sf10)
    .set_bw(LoRaBandwidth::Bw125)
    .set_cr(CodingRate::Cr45)
    .set_ldro_en(false);

const PKT_PARAMS: LoRaPacketParams = LoRaPacketParams::new()
    .set_preamble_len(16)
    .set_header_type(HeaderType::Variable)
    .set_payload_len(24)
    .set_crc_en(true)
    .set_invert_iq(false);

pub fn setup_radio(radio: &mut SubGhz<SgMiso, SgMosi>) -> Result<(), Error> {
    radio.set_standby(StandbyClk::Rc)?;
    radio.set_tx_rx_fallback_mode(FallbackMode::StandbyHse)?;

    radio.set_irq_cfg(&IRQ_CFG)?;
    radio.set_regulator_mode(RegMode::Smps)?;
    radio.set_buffer_base_address(TX_BUF_OFFSET, RX_BUF_OFFSET)?;
    radio.set_pa_ocp(Ocp::Max140m)?;

    radio.set_standby(StandbyClk::Rc)?;
    radio.set_packet_type(PacketType::LoRa)?;
    radio.set_lora_sync_word(LoRaSyncWord::Custom([0x24, 0x34]))?;
    radio.set_lora_mod_params(&MOD_PARAMS)?;
    radio.set_lora_packet_params(&PKT_PARAMS)?;

    Ok(())
}

pub fn start_radio_rx(radio: &mut SubGhz<SgMiso, SgMosi>, timeout_ms: u32) -> Result<(), Error> {
    defmt::info!("radio: start Rx, timeout={}", timeout_ms);
    if timeout_ms == 0 || timeout_ms == u32::MAX {
        radio.set_rx(Timeout::DISABLED)?;
    } else {
        radio.set_rx(Timeout::from_millis_sat(timeout_ms))?;
    }

    Ok(())
}

pub fn handle_radio_rx_done(
    radio: &mut SubGhz<SgMiso, SgMosi>,
    irq: u16,
    rx_queue: &mut CacheQueue,
) -> Result<(), Error> {
    if irq & Irq::RxDone.mask() != 0 {
        radio_encode_packet(radio, rx_queue)?;
        return Ok(());
    }

    Ok(())
}

pub fn start_radio_tx(radio: &mut SubGhz<SgMiso, SgMosi>, tx_buf: &[u8], timeout_ms: u32) -> Result<(), Error> {
    radio.write_buffer(TX_BUF_OFFSET, tx_buf)?;
    let (_, irq) = radio.irq_status()?;
    radio.clear_irq_status(irq)?;

    if timeout_ms == 0 || timeout_ms == u32::MAX {
        radio.set_tx(Timeout::DISABLED)?;
    } else {
        radio.set_tx(Timeout::from_millis_sat(timeout_ms))?;
    }

    Ok(())
}

pub fn set_radio_to_standby(radio: &mut SubGhz<SgMiso, SgMosi>) -> Result<(), Error> {
    radio.set_standby(StandbyClk::Rc)?;

    Ok(())
}
