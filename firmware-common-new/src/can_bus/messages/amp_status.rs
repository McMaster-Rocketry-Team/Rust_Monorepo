use packed_struct::prelude::*;
use serde::{Deserialize, Serialize};

use super::{CanBusMessage, CanBusMessageEnum};

#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(
    PrimitiveEnum_u8, Clone, Copy, Debug, PartialEq, Eq, Ord, PartialOrd, Serialize, Deserialize,
)]
#[repr(C)]
pub enum PowerOutputStatus {
    Disabled = 0,
    PowerGood = 1,
    PowerBad = 2,
    /// AMP has not reported this output's status.
    ///
    /// Not the same as [`PowerOutputStatus::Disabled`], which is an output AMP
    /// is actively reporting as commanded off — a normal, deliberate state.
    /// This one means nobody has said anything about the output at all: AMP is
    /// offline, has not sent its first `AmpStatusMessage` yet, or the field was
    /// never populated. Rendering that as "disabled" would put a report on
    /// screen that the rocket never made.
    ///
    /// It also has to exist for a second reason. The field is 2 bits wide
    /// everywhere it appears (here and in three VLP packets), so `0b11` is on
    /// the wire whether or not anything means to put it there. While the code
    /// was undefined, `from_primitive` returned `None` for it and unpacking
    /// failed for the WHOLE containing packet — one stray bit pattern in an
    /// AMP status field would drop an entire telemetry frame rather than one
    /// output's status.
    Unknown = 3,
}

#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(PackedStruct, Clone, Debug, PartialEq, Eq, Ord, PartialOrd, Serialize, Deserialize)]
#[packed_struct(bit_numbering = "msb0", endian = "msb", size_bytes = "1")]
#[repr(C)]
pub struct AmpOutputStatus {
    #[packed_field(bits = "0..1")]
    pub overwrote: bool,
    #[packed_field(bits = "1..3", ty = "enum")]
    pub status: PowerOutputStatus,
}

#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(PackedStruct, Clone, Debug, PartialEq, Eq, Ord, PartialOrd, Serialize, Deserialize)]
#[packed_struct(bit_numbering = "msb0", endian = "msb", size_bytes = "5")]
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
mod tests {
    use crate::{can_bus::messages::tests as can_bus_messages_test, tests::init_logger};

    use super::*;

    fn create_test_messages() -> Vec<CanBusMessageEnum> {
        vec![
            AmpStatusMessage {
                shared_battery_mv: 0,
                out1: AmpOutputStatus {
                    overwrote: true,
                    status: PowerOutputStatus::PowerGood,
                },
                out2: AmpOutputStatus {
                    overwrote: false,
                    status: PowerOutputStatus::Disabled,
                },
                out3: AmpOutputStatus {
                    overwrote: true,
                    status: PowerOutputStatus::PowerBad,
                },
            }
            .into(),
            AmpStatusMessage {
                shared_battery_mv: u16::MAX,
                out1: AmpOutputStatus {
                    overwrote: true,
                    status: PowerOutputStatus::PowerGood,
                },
                out2: AmpOutputStatus {
                    overwrote: false,
                    status: PowerOutputStatus::Disabled,
                },
                out3: AmpOutputStatus {
                    overwrote: true,
                    status: PowerOutputStatus::PowerBad,
                },
            }
            .into(),
            // The 2-bit field's fourth code. It used to be undefined, so this
            // message did not decode at all.
            AmpStatusMessage {
                shared_battery_mv: 7400,
                out1: AmpOutputStatus {
                    overwrote: false,
                    status: PowerOutputStatus::Unknown,
                },
                out2: AmpOutputStatus {
                    overwrote: false,
                    status: PowerOutputStatus::Unknown,
                },
                out3: AmpOutputStatus {
                    overwrote: false,
                    status: PowerOutputStatus::Unknown,
                },
            }
            .into(),
        ]
    }

    /// `0b11` is on the wire whether or not anything means to put it there —
    /// the field is 2 bits and only three codes were defined. While it was
    /// undefined, `from_primitive` returned `None` and unpacking failed for the
    /// entire message, so one stray bit pattern in an output status field cost
    /// the shared battery voltage and the other two outputs as well.
    #[test]
    fn all_ones_output_code_decodes_instead_of_failing() {
        init_logger();

        assert_eq!(
            PowerOutputStatus::from_primitive(0b11),
            Some(PowerOutputStatus::Unknown)
        );

        let unpacked = AmpOutputStatus::unpack(&[0b0_11_00000]).unwrap();
        assert_eq!(unpacked.status, PowerOutputStatus::Unknown);
        assert!(!unpacked.overwrote);
        // And it is emphatically not `Disabled`, which is AMP reporting an
        // output as commanded off.
        assert_ne!(unpacked.status, PowerOutputStatus::Disabled);
    }

    #[test]
    fn test_serialize_deserialize() {
        init_logger();

        can_bus_messages_test::test_serialize_deserialize(create_test_messages());
    }

    #[test]
    fn create_reference_data() {
        init_logger();

        can_bus_messages_test::create_reference_data(create_test_messages(), "amp_status");
    }
}
