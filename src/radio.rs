use core::cmp;

use heapless::spsc::Queue;
use stm32wlxx_hal::{
    gpio::{
        pins::{B8, C13},
        Output,
    },
    spi::{Error, SgMiso, SgMosi},
    subghz::{
        CalibrateImage, FallbackMode, Irq, LoRaModParams, LoRaPacketParams, LoRaPacketStatus,
        LoRaSyncWord, Ocp, PaConfig, PacketType, RegMode, RfFreq, StandbyClk, SubGhz, Timeout,
        TxParams,
    },
};

use crate::{constants::{SLIP_END, SLIP_START}, misc::slip_enqueue};

const TX_BUF_OFFSET: u8 = 128;
const RX_BUF_OFFSET: u8 = 0;

fn radio_encode_packet(radio: &mut SubGhz<SgMiso, SgMosi>, rx_queue: &mut Queue<u8, 1024>) -> Result<(), Error> {
    let pkt_status = radio.lora_packet_status()?;

    let mut output_buf: [u8; 256] = [0; 256];
    let (_, data_len, ptr) = radio.rx_buffer_status()?;
    radio.read_buffer(ptr, output_buf.as_mut_slice())?;
    
    defmt::info!("radio: RxDone, got {:?}; len={}", pkt_status, data_len);
    match rx_queue.enqueue(SLIP_START) {
        Ok(_) => {},
        Err(b) => {
            defmt::warn!("radio: Rx buffer full! Ditching oldest");
            rx_queue.dequeue(); // Drop the oldest
            rx_queue.enqueue(b).unwrap();
        }
    }

    let pkt_rssi = cmp::max(pkt_status.signal_rssi_pkt().to_integer(), 0);
    slip_enqueue(rx_queue, (pkt_rssi * -1) as u8);

    let snr = cmp::max(pkt_status.snr_pkt().to_integer(), 0);
    slip_enqueue(rx_queue, (snr * -1) as u8);
    slip_enqueue(rx_queue, data_len);
    
    for b in output_buf {
        slip_enqueue(rx_queue, b);
    }

    match rx_queue.enqueue(SLIP_END) {
        Ok(_) => {},
        Err(b) => {
            defmt::warn!("radio: Rx buffer full! Ditching oldest");
            rx_queue.dequeue(); // Drop the oldest
            rx_queue.enqueue(b).unwrap();
        }
    }

    Ok(())
}

pub fn setup_radio(
    radio: &mut SubGhz<SgMiso, SgMosi>,
    freq: u32,
    pkt_params: LoRaPacketParams,
    mod_params: LoRaModParams,
    pa_cfg: PaConfig,
    tx_params: TxParams,
    sync_word: LoRaSyncWord,
) -> Result<(), Error> {
    radio.set_standby(StandbyClk::Rc)?;
    radio.set_tx_rx_fallback_mode(FallbackMode::StandbyHse)?;

    radio.set_regulator_mode(RegMode::Smps)?;
    radio.set_buffer_base_address(TX_BUF_OFFSET, RX_BUF_OFFSET)?;
    radio.set_pa_config(&pa_cfg)?;
    radio.set_pa_ocp(Ocp::Max140m)?;
    radio.set_tx_params(&tx_params)?;

    radio.set_packet_type(PacketType::LoRa)?;
    radio.set_lora_sync_word(sync_word)?;
    radio.set_lora_mod_params(&mod_params)?;
    radio.set_lora_packet_params(&pkt_params)?;

    radio.calibrate_image(CalibrateImage::ISM_902_928)?;

    let rf_freq = RfFreq::from_frequency(freq);
    radio.set_rf_frequency(&rf_freq)?;
    radio.set_standby(StandbyClk::Hse)?;

    Ok(())
}

pub fn start_radio_rx(radio: &mut SubGhz<SgMiso, SgMosi>, timeout_ms: u32) -> Result<(), Error> {
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
    rx_queue: &mut Queue<u8, 1024>
) -> Result<(), Error> {
    if irq & Irq::Timeout.mask() == 0 && irq & Irq::RxDone.mask() != 0 {
        radio_encode_packet(radio, rx_queue)?;
        return Ok(());
    }

    Ok(())
}

pub fn start_radio_tx(
    radio: &mut SubGhz<SgMiso, SgMosi>,
    tx_buf: &[u8],
    timeout_ms: u32,
) -> Result<(), Error> {
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
