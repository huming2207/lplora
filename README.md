# LpLoRa 

Yet another LoRa/GFSK <-> UART radio transceiver, based on STM32WLE5 (targeting RAK's [RAK3172](https://docs.rakwireless.com/Product-Categories/WisDuo/RAK3172-Module/Datasheet/) module), RTIC and Rust. Try focusing low-power & interrupt driven optimisation.

Also check out the Node.js host library here: https://github.com/huming2207/lplora-host

## How to compile

1. Refer to [RTIC template's dependencies installation guide](https://github.com/rtic-rs/defmt-app-template?tab=readme-ov-file#dependencies) to install dependencies, also don't forget to install the toolchain first.
2. Run `cargo build` for debug build, or `cargo build --release` for release build.

## Todo list

- [x] UART protocol bringup
- [ ] Radio transceive testing
- [ ] Power management & optimisation
- [ ] Proper STM32WL LPUART FIFO mode implementation


## UART Protocol

TBD

## License

MIT or Apache
