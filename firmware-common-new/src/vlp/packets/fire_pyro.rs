use packed_struct::prelude::*;
use serde::{Deserialize, Serialize};

use super::VLPUplinkPacket;

#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(
    PrimitiveEnum_u8, Clone, Copy, Debug, PartialEq, Eq, Ord, PartialOrd, Serialize, Deserialize,
)]
#[repr(C)]
pub enum PyroSelect {
    Pyro1 = 0,
    Pyro2 = 1,
}

#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(PackedStruct, Debug, Clone, PartialEq, Eq, Deserialize, Serialize)]
#[packed_struct(bit_numbering = "msb0", endian = "msb", size_bytes = "1")]
pub struct FirePyroPacket {
    #[packed_field(bits = "0..2", ty = "enum")]
    pub pyro: PyroSelect,
}

impl Into<VLPUplinkPacket> for FirePyroPacket {
    fn into(self) -> VLPUplinkPacket {
        VLPUplinkPacket::FirePyro(self)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_serialize_deserialize() {
        let packet = FirePyroPacket {
            pyro: PyroSelect::Pyro2,
        };
        let packet: VLPUplinkPacket = packet.into();

        let mut buffer = [0u8; 10];
        let len = packet.serialize(&mut buffer);

        let deserialized_packet = VLPUplinkPacket::deserialize(&buffer[..len]).unwrap();

        assert_eq!(deserialized_packet, packet);
    }
}
