use cortex_m::interrupt::CriticalSection;
use stm32wlxx_hal::{
    pac::{self, Peripherals},
    rcc,
};

const SCB_SCR_SLEEPDEEP: u32 = 0x1 << 2;

pub fn enter_stop2_mode() {
    unsafe {
        if (*pac::RCC::PTR).cfgr.read().sws().is_msi() {
            (*pac::RCC::PTR).cfgr.modify(|_, w| w.stopwuck().clear_bit()); // If currently using MSI, set wakeup clock to MSI
        } else if (*pac::RCC::PTR).cfgr.read().sws().is_hsi16() {
            (*pac::RCC::PTR).cfgr.modify(|_, w| w.stopwuck().set_bit()); // If currently using HSI, set wakeup clock to HSI
        }
    }

    // Wait till LDORDY is set, this is for errata 2.2.11 workaround
    while unsafe { (*pac::PWR::PTR).sr2.read().ldordy().is_not_ready() } {}

    unsafe {
        (*pac::PWR::PTR).scr.write(|w| {
            w.cwuf1().clear_bit();
            w.cwuf2().clear_bit();
            w.cwuf3().clear_bit()
        });

        (*pac::PWR::PTR).cr3.modify(|_, w| w.eulpen().set_bit());

        (*pac::PWR::PTR).cr1.modify(|_, w| w.lpms().stop2());

        (*pac::SCB::PTR).scr.modify(|scr| scr | SCB_SCR_SLEEPDEEP);

        cortex_m::asm::wfi();

        let cs = &CriticalSection::new();
        let mut dp = Peripherals::steal();

        rcc::set_sysclk_msi(
            &mut dp.FLASH,
            &mut dp.PWR,
            &mut dp.RCC,
            rcc::MsiRange::Range16M, // Maybe we should consider 16MHz here so that no flash wait cycle needed??
            cs,
        );

        // Enable HSE, no TCXO for now cuz we don't need that
        if dp.RCC.cr.read().hserdy().bit_is_clear() {
            dp.RCC.cr.write(|w| w.hseon().set_bit());
            while dp.RCC.cr.read().hserdy().bit_is_clear() {}
        }
    }
}
