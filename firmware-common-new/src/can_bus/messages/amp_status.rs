#[cfg(feature = "wasm")]
use tsify::Tsify;
#[cfg(feature = "wasm")]
use wasm_bindgen::prelude::*;

use packed_struct::prelude::*;
use serde::{Deserialize, Serialize};

use super::{CanBusMessage, CanBusMessageEnum};

#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg_attr(feature = "wasm", derive(Tsify))]
#[cfg_attr(feature = "wasm", tsify(into_wasm_abi, from_wasm_abi))]
#[derive(PrimitiveEnum_u8, Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[repr(C)]
pub enum PowerOutputStatus {
    Disabled = 0,
    PowerGood = 1,
    PowerBad = 2,
}

#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg_attr(feature = "wasm", derive(Tsify))]
#[cfg_attr(feature = "wasm", tsify(into_wasm_abi, from_wasm_abi))]
#[derive(PackedStruct, Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[packed_struct(bit_numbering = "msb0", endian = "msb", size_bytes = "1")]
#[repr(C)]
pub struct AmpOutputStatus {
    #[packed_field(bits = "0..1")]
    pub overwrote: bool,
    #[packed_field(bits = "1..3", ty = "enum")]
    pub status: PowerOutputStatus,
}

#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg_attr(feature = "wasm", derive(Tsify))]
#[cfg_attr(feature = "wasm", tsify(into_wasm_abi, from_wasm_abi))]
#[derive(PackedStruct, Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[packed_struct(bit_numbering = "msb0", endian = "msb", size_bytes = "6")]
#[repr(C)]
pub struct AmpStatusMessage {
    pub shared_battery_mv: u16,

    // Can't use `#[packed_field(element_size_bits = "3")]` here due to packed_struct crate bug
    #[packed_field(element_size_bytes = "1")]
    pub out1: AmpOutputStatus,
    #[packed_field(element_size_bytes = "1")]
    pub out2: AmpOutputStatus,
    #[packed_field(element_size_bytes = "1")]
    pub out3: AmpOutputStatus,
    #[packed_field(element_size_bytes = "1")]
    pub out4: AmpOutputStatus,
}

impl CanBusMessage for AmpStatusMessage {
    fn priority(&self) -> u8 {
        5
    }
}

impl Into<CanBusMessageEnum> for AmpStatusMessage {
    fn into(self) -> CanBusMessageEnum {
        CanBusMessageEnum::AmpStatus(self)
    }
}

#[cfg(test)]
mod test {
    use std::assert_matches::assert_matches;

    use crate::tests::init_logger;

    use super::*;

    #[test]
    fn test_serialize_deserialize() {
        init_logger();

        let status_message = AmpStatusMessage {
            shared_battery_mv: 8001,
            out1: AmpOutputStatus {
                overwrote: true,
                status: PowerOutputStatus::PowerGood,
            },
            out2: AmpOutputStatus {
                overwrote: true,
                status: PowerOutputStatus::PowerGood,
            },
            out3: AmpOutputStatus {
                overwrote: false,
                status: PowerOutputStatus::PowerBad,
            },
            out4: AmpOutputStatus {
                overwrote: false,
                status: PowerOutputStatus::PowerBad,
            },
        };

        let source_message: CanBusMessageEnum = status_message.into();
        let message = source_message.clone();
        let mut buffer = [0u8; 64];
        let message_type = message.get_message_type();
        let len = message.serialize(&mut buffer);

        log_info!("{:?}", &buffer[..len]);

        let deserialized = CanBusMessageEnum::deserialize(message_type, &buffer[..len]).unwrap();
        log_info!("{:?}", deserialized);

        assert_matches!(
            deserialized,
            CanBusMessageEnum::AmpStatus(AmpStatusMessage {
                shared_battery_mv: 8001,
                out1: AmpOutputStatus {
                    overwrote: true,
                    status: PowerOutputStatus::PowerGood,
                },
                ..
            })
        );
    }
}
