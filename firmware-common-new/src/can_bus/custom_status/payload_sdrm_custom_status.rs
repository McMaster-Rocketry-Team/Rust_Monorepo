use packed_struct::prelude::*;
use serde::{Deserialize, Serialize};

use crate::can_bus::custom_status::NodeCustomStatus;

/// Stack state of the payload SDRM node, carried in
/// `NodeStatusMessage::custom_status_raw`.
///
/// The SDRM's own layout, least significant bit first (matching
/// `stack_protocol.h` on the payload side). Bits 8..10 of the 11 available are
/// spare.
///
/// | Bit | Flag |
/// |-----|------|
/// | 0 | `epm_alive` |
/// | 1 | `sem_alive` |
/// | 2 | `epm_rails_on` |
/// | 3 | `exp1_active` |
/// | 4 | `exp2_active` |
/// | 5 | `exp3_active` |
/// | 6 | `sdrm_sd_logging` |
/// | 7 | `sem_sd_logging` |
/// | 8..10 | spare |
///
/// The `bits` attributes are positions in the packed 2-byte buffer, which
/// `NodeCustomStatusExt::to_u16` shifts right by 5: packed bit `i` becomes
/// status bit `10 - i`. `test_bit_positions` pins every flag.
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(PackedStruct, Clone, Debug, PartialEq, Eq, Ord, PartialOrd, Serialize, Deserialize)]
#[packed_struct(bit_numbering = "msb0", endian = "msb", size_bytes = "2")]
#[repr(C)]
pub struct PayloadSDRMCustomStatus {
    /// SEM SD log active
    #[packed_field(bits = "3..4")]
    pub sem_sd_logging: bool,
    /// SDRM SD log active
    #[packed_field(bits = "4..5")]
    pub sdrm_sd_logging: bool,
    /// Experiment channel 3 active
    #[packed_field(bits = "5..6")]
    pub exp3_active: bool,
    /// Experiment channel 2 active
    #[packed_field(bits = "6..7")]
    pub exp2_active: bool,
    /// Experiment channel 1 active
    #[packed_field(bits = "7..8")]
    pub exp1_active: bool,
    /// EPM reports the peripheral rails energized (`kEpmFlagRailsOn` in the
    /// latest intra-stack status frame). A live observation, not a latch: it
    /// drops on its own if a rail falls over after `power_on`.
    #[packed_field(bits = "8..9")]
    pub epm_rails_on: bool,
    /// SEM responded on the intra-stack bus
    #[packed_field(bits = "9..10")]
    pub sem_alive: bool,
    /// EPM responded on the intra-stack bus
    #[packed_field(bits = "10..11")]
    pub epm_alive: bool,
}

impl PayloadSDRMCustomStatus {
    /// Nothing brought up yet.
    pub fn new() -> Self {
        Self {
            sem_sd_logging: false,
            sdrm_sd_logging: false,
            exp3_active: false,
            exp2_active: false,
            exp1_active: false,
            epm_rails_on: false,
            sem_alive: false,
            epm_alive: false,
        }
    }

    /// Clears `expN_active`, leaving liveness, rail state and logging untouched.
    ///
    /// Applied after the `LowPower` safe-reset completes.
    pub fn clear_experiment_flags(&mut self) {
        self.exp1_active = false;
        self.exp2_active = false;
        self.exp3_active = false;
    }

    /// `clear_experiment_flags` plus `epm_rails_on` and both SD logging flags,
    /// leaving liveness untouched.
    ///
    /// Applied after the `Landed` shutdown completes. Clearing `epm_rails_on`
    /// here is only an immediate optimistic update — EPM's next intra-stack
    /// status frame is what the bit actually tracks.
    pub fn clear_powered_flags(&mut self) {
        self.clear_experiment_flags();
        self.epm_rails_on = false;
        self.sdrm_sd_logging = false;
        self.sem_sd_logging = false;
    }
}

impl Default for PayloadSDRMCustomStatus {
    fn default() -> Self {
        Self::new()
    }
}

impl NodeCustomStatus for PayloadSDRMCustomStatus {}

#[cfg(test)]
mod tests {
    use crate::can_bus::custom_status::NodeCustomStatusExt;
    use crate::can_bus::messages::node_status::{NodeHealth, NodeMode, NodeStatusMessage};
    use crate::utils::FixedLenSerializable;

    use super::*;

    #[test]
    fn test_serialize_deserialize() {
        let status = PayloadSDRMCustomStatus {
            epm_alive: true,
            sem_alive: true,
            epm_rails_on: true,
            sdrm_sd_logging: false,
            sem_sd_logging: false,
            exp1_active: false,
            exp2_active: false,
            exp3_active: false,
        };

        let status_u16 = status.to_u16();
        assert_eq!(status_u16, 0b00000000111);

        let status2 = PayloadSDRMCustomStatus::from_u16(status_u16);
        assert_eq!(status, status2);
    }

    /// Every flag sits exactly where the SDRM's `packCustomStatusRaw` puts it,
    /// and bits 8..10 stay clear.
    #[test]
    fn test_bit_positions() {
        let mut status = PayloadSDRMCustomStatus::new();
        assert_eq!(status.to_u16(), 0);

        for (bit, apply) in [
            (0u16, (|s: &mut PayloadSDRMCustomStatus| s.epm_alive = true)
                as fn(&mut PayloadSDRMCustomStatus)),
            (1, |s| s.sem_alive = true),
            (2, |s| s.epm_rails_on = true),
            (3, |s| s.exp1_active = true),
            (4, |s| s.exp2_active = true),
            (5, |s| s.exp3_active = true),
            (6, |s| s.sdrm_sd_logging = true),
            (7, |s| s.sem_sd_logging = true),
        ] {
            let mut only_this = PayloadSDRMCustomStatus::new();
            apply(&mut only_this);
            assert_eq!(only_this.to_u16(), 1 << bit, "bit {} misplaced", bit);

            apply(&mut status);
        }

        // Bits 8..10 are spare and must stay zero even with everything set.
        assert_eq!(status.to_u16(), 0xFF);
    }

    /// Reference frame for the payload team: rails up, all experiments active,
    /// both SD logs healthy, uptime 120s.
    #[test]
    fn test_reference_node_status_frame() {
        let status = PayloadSDRMCustomStatus {
            epm_alive: true,
            sem_alive: true,
            epm_rails_on: true,
            sdrm_sd_logging: true,
            sem_sd_logging: true,
            exp1_active: true,
            exp2_active: true,
            exp3_active: true,
        };
        assert_eq!(status.to_u16(), 0xFF);

        let message =
            NodeStatusMessage::new(120, NodeHealth::Healthy, NodeMode::Operational, status);

        let mut buffer = [0u8; 5];
        FixedLenSerializable::serialize(&message, &mut buffer);
        assert_eq!(buffer, [0x00, 0x00, 0x78, 0x01, 0xFE]);
    }

    #[test]
    fn test_clear_flags() {
        let all_set = PayloadSDRMCustomStatus {
            epm_alive: true,
            sem_alive: true,
            epm_rails_on: true,
            sdrm_sd_logging: true,
            sem_sd_logging: true,
            exp1_active: true,
            exp2_active: true,
            exp3_active: true,
        };

        // LowPower safe-reset: experiments cleared, rest kept
        let mut status = all_set.clone();
        status.clear_experiment_flags();
        assert_eq!(status.to_u16(), 0b11000111);

        // Landed shutdown: also rails and both SD logging flags, liveness kept
        let mut status = all_set.clone();
        status.clear_powered_flags();
        assert_eq!(status.to_u16(), 0b00000011);
    }
}
