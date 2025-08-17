use packed_struct::prelude::*;
use serde::{Deserialize, Serialize};

use super::{CanBusMessage, CanBusMessageEnum, amp_overwrite::PowerOutputOverwrite};

#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(PackedStruct, Clone, Debug, PartialEq, Eq, Ord, PartialOrd, Serialize, Deserialize)]
#[packed_struct(bit_numbering = "msb0", endian = "msb", size_bytes = "3")]
#[repr(C)]
pub struct PayloadEPSOutputOverwriteMessage {
    #[packed_field(bits = "0..2", ty = "enum")]
    pub out_3v3: PowerOutputOverwrite,
    #[packed_field(bits = "2..4", ty = "enum")]
    pub out_5v: PowerOutputOverwrite,
    #[packed_field(bits = "4..6", ty = "enum")]
    pub out_9v: PowerOutputOverwrite,

    /// Node ID of EPS to control
    #[packed_field(element_size_bits = "12")]
    pub node_id: u16,
}

impl CanBusMessage for PayloadEPSOutputOverwriteMessage {
    fn priority(&self) -> u8 {
        2
    }
}

impl Into<CanBusMessageEnum> for PayloadEPSOutputOverwriteMessage {
    fn into(self) -> CanBusMessageEnum {
        CanBusMessageEnum::PayloadEPSOutputOverwrite(self)
    }
}

#[cfg(test)]
mod test {
    use crate::tests::init_logger;

    use super::*;

    #[test]
    fn test_serialize_deserialize() {
        init_logger();

        let status_message = PayloadEPSOutputOverwriteMessage {
            out_3v3: PowerOutputOverwrite::ForceDisabled,
            out_5v: PowerOutputOverwrite::ForceEnabled,
            out_9v: PowerOutputOverwrite::NoOverwrite,
            node_id: 10,
        };

        let source_message: CanBusMessageEnum = status_message.into();
        let message = source_message.clone();
        let mut buffer = [0u8; 64];
        let message_type = message.get_message_type();
        let len = message.serialize(&mut buffer);

        log_info!("{:?}", &buffer[..len]);

        let deserialized = CanBusMessageEnum::deserialize(message_type, &buffer[..len]).unwrap();
        log_info!("{:?}", deserialized);

        assert_eq!(deserialized, source_message,);
    }
}
