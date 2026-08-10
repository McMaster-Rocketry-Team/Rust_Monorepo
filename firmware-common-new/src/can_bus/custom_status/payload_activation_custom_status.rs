use packed_struct::prelude::*;
use serde::{Deserialize, Serialize};

use crate::can_bus::custom_status::NodeCustomStatus;

/// Stack state of the payload activation node (SDRM), carried in
/// `NodeStatusMessage::custom_status_raw`.
///
/// Uses all 11 available bits. Per-channel tare / home progress is not exposed
/// individually; use `prep_complete` followed by `expN_active`.
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(PackedStruct, Clone, Debug, PartialEq, Eq, Ord, PartialOrd, Serialize, Deserialize)]
#[packed_struct(bit_numbering = "msb0", endian = "msb", size_bytes = "2")]
#[repr(C)]
pub struct PayloadActivationCustomStatus {
    /// EPM responded on the intra-stack bus
    pub epm_alive: bool,
    /// SEM responded on the intra-stack bus
    pub sem_alive: bool,
    /// `power_on` complete
    pub stack_powered: bool,
    /// SDRM SD log active
    pub sdrm_sd_logging: bool,
    /// SEM SD log active
    pub sem_sd_logging: bool,
    /// Experiment channel 1 active
    pub exp1_active: bool,
    /// Experiment channel 2 active
    pub exp2_active: bool,
    /// Experiment channel 3 active
    pub exp3_active: bool,
    /// Tare + home complete for channels 1..3
    pub prep_complete: bool,
    /// Full `Armed` sequence finished OK
    pub armed_bundle_complete: bool,
    /// Last stack action failed
    pub fault: bool,
}

impl PayloadActivationCustomStatus {
    /// Nothing brought up yet, no fault.
    pub fn new() -> Self {
        Self {
            epm_alive: false,
            sem_alive: false,
            stack_powered: false,
            sdrm_sd_logging: false,
            sem_sd_logging: false,
            exp1_active: false,
            exp2_active: false,
            exp3_active: false,
            prep_complete: false,
            armed_bundle_complete: false,
            fault: false,
        }
    }

    /// Clears bits 5..9 (`expN_active`, `prep_complete`, `armed_bundle_complete`),
    /// leaving liveness, power, logging and `fault` untouched.
    ///
    /// Applied after the `LowPower` safe-reset completes.
    pub fn clear_experiment_flags(&mut self) {
        self.exp1_active = false;
        self.exp2_active = false;
        self.exp3_active = false;
        self.prep_complete = false;
        self.armed_bundle_complete = false;
    }

    /// Clears bits 2..9 (`clear_experiment_flags` plus `stack_powered` and both
    /// SD logging flags), leaving liveness and `fault` untouched.
    ///
    /// Applied after the `Landed` shutdown completes.
    pub fn clear_powered_flags(&mut self) {
        self.clear_experiment_flags();
        self.stack_powered = false;
        self.sdrm_sd_logging = false;
        self.sem_sd_logging = false;
    }
}

impl Default for PayloadActivationCustomStatus {
    fn default() -> Self {
        Self::new()
    }
}

impl NodeCustomStatus for PayloadActivationCustomStatus {}

#[cfg(test)]
mod tests {
    use crate::can_bus::custom_status::NodeCustomStatusExt;
    use crate::can_bus::messages::node_status::{NodeHealth, NodeMode, NodeStatusMessage};
    use crate::utils::FixedLenSerializable;

    use super::*;

    #[test]
    fn test_serialize_deserialize() {
        let status = PayloadActivationCustomStatus {
            epm_alive: true,
            sem_alive: true,
            stack_powered: true,
            sdrm_sd_logging: false,
            sem_sd_logging: false,
            exp1_active: false,
            exp2_active: false,
            exp3_active: false,
            prep_complete: true,
            armed_bundle_complete: false,
            fault: true,
        };

        let status_u16 = status.to_u16();
        assert_eq!(status_u16, 0b11100000101);

        let status2 = PayloadActivationCustomStatus::from_u16(status_u16);
        assert_eq!(status, status2);
    }

    #[test]
    fn test_uses_all_11_bits() {
        let mut status = PayloadActivationCustomStatus::new();
        assert_eq!(status.to_u16(), 0);

        status.epm_alive = true;
        assert_eq!(status.to_u16(), 0b10000000000);

        let mut status = PayloadActivationCustomStatus::new();
        status.fault = true;
        assert_eq!(status.to_u16(), 0b00000000001);
    }

    /// Reference frame for the payload team: armed bundle complete, all experiments
    /// active, uptime 120s, no fault.
    ///
    /// The 11 bits are laid out MSB first (`epm_alive` is the most significant),
    /// matching `VLCustomStatus` and `OzysCustomStatus`.
    #[test]
    fn test_reference_node_status_frame() {
        let status = PayloadActivationCustomStatus {
            epm_alive: true,
            sem_alive: true,
            stack_powered: true,
            sdrm_sd_logging: true,
            sem_sd_logging: true,
            exp1_active: true,
            exp2_active: true,
            exp3_active: true,
            prep_complete: true,
            armed_bundle_complete: true,
            fault: false,
        };
        assert_eq!(status.to_u16(), 0x7FE);

        let message =
            NodeStatusMessage::new(120, NodeHealth::Healthy, NodeMode::Operational, status);

        let mut buffer = [0u8; 5];
        FixedLenSerializable::serialize(&message, &mut buffer);
        assert_eq!(buffer, [0x00, 0x00, 0x78, 0x0F, 0xFC]);
    }

    #[test]
    fn test_clear_flags() {
        let all_set = PayloadActivationCustomStatus {
            epm_alive: true,
            sem_alive: true,
            stack_powered: true,
            sdrm_sd_logging: true,
            sem_sd_logging: true,
            exp1_active: true,
            exp2_active: true,
            exp3_active: true,
            prep_complete: true,
            armed_bundle_complete: true,
            fault: true,
        };

        // LowPower safe-reset: bits 5..9 cleared, everything else kept
        let mut status = all_set.clone();
        status.clear_experiment_flags();
        assert_eq!(status.to_u16(), 0b11111000001);

        // Landed shutdown: bits 2..9 cleared, liveness and fault kept
        let mut status = all_set.clone();
        status.clear_powered_flags();
        assert_eq!(status.to_u16(), 0b11000000001);
    }
}
