use stm32wlxx_hal::gpio;

pub const RFSW_GPIO_OUTPUT_ARGS: gpio::OutputArgs = gpio::OutputArgs {
    level: gpio::PinState::Low,
    speed: gpio::Speed::High,
    ot: gpio::OutputType::PushPull,
    pull: gpio::Pull::Up,
};

pub const SLIP_START: u8 = 0xa5;
pub const SLIP_END: u8 = 0xc0;
pub const SLIP_ESC: u8 = 0xdb;
pub const SLIP_ESC_END: u8 = 0xdc;
pub const SLIP_ESC_ESC: u8 = 0xdd;
pub const SLIP_ESC_START: u8 = 0xde;
