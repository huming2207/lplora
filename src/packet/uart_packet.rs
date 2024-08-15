use crate::constants::PacketType;

pub struct UartCmdPacket {
    pkt_type: PacketType,
    data_buf: [u8; 256],
}

impl UartCmdPacket {
    pub fn new() -> Result<UartCmdPacket, ()> {
        Err(())
    }
}