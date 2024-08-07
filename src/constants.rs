use stm32wlxx_hal::gpio;

pub const RFSW_GPIO_OUTPUT_ARGS: gpio::OutputArgs = gpio::OutputArgs {
    level: gpio::PinState::Low,
    speed: gpio::Speed::High,
    ot: gpio::OutputType::PushPull,
    pull: gpio::Pull::Up,
};
