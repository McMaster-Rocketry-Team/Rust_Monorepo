use core::fmt::Debug;
use packed_struct::prelude::*;

use crate::{fixed_point_factory, utils::FixedLenSerializable};
use ack::AckPacket;
use amp_output_overwrite::AMPOutputOverwritePacket;
use change_mode::ChangeModePacket;
use fire_pyro::FirePyroPacket;
use set_target_apogee::SetTargetApogeePacket;
use low_power_telemetry::LowPowerTelemetryPacket;
use reset::ResetPacket;
use self_test_result::SelfTestResultPacket;
use telemetry::TelemetryPacket;
use landed_telemetry::LandedTelemetryPacket;

pub mod ack;
pub mod amp_output_overwrite;
pub mod change_mode;
pub mod fire_pyro;
pub mod landed_telemetry;
pub mod low_power_telemetry;
pub mod reset;
pub mod self_test_result;
pub mod telemetry;
pub mod set_target_apogee;

// TODO change
pub const MAX_VLP_PACKET_SIZE: usize = 100;

// Shared by every packet that carries a temperature, so the same reading
// decodes identically whichever one it arrives on. Kept here rather than
// duplicated per packet because the two copies had drifted: the low-power
// packet's floor was 0 C, which silently clamped sub-freezing pad readings.
fixed_point_factory!(TemperatureFac, f32, -10.0, 85.0, 0.2);

// 10 bits over 2.5..8.5V, so (8.5 - 2.5) / (2^10 - 1) = 5.87mV per code.
//
// Shared by every packet that carries a battery voltage, for the same reason as
// `TemperatureFac` above: all three downlink packets carry `shared_battery_v`,
// and a voltage that decodes one way on the flight packet and another way on
// the landed packet would be worse than either choice on its own. Three copies
// of a factory is three chances for the ranges to drift apart, which is exactly
// how the temperature floor got out of step.
fixed_point_factory!(BatteryVFac, f32, 2.5, 8.5, 0.01);

/// The code reserved for "this battery voltage was never reported", spent on
/// `shared_battery_v` in all three downlink packets.
///
/// A sentinel rather than a validity bit, and not because any one packet is
/// short of bits: `LandedTelemetryPacket` has exactly zero spare bits, so a
/// validity bit could not propagate there without a twelfth byte. The top code
/// costs 5.87mV of headroom at 8.5V — a bus that is over-range is already
/// pegged at the top of the scale and is not read to that precision anyway. The
/// bottom of the range stays untouchable for the usual reason: 2.5V is a
/// collapsed pack, a fault the ground has to be able to see.
///
/// `vl_battery_v` / `battery_v` deliberately do NOT use this. Those are the
/// voltage of the board building the packet, so they are always present.
const SHARED_BATTERY_V_UNAVAILABLE_CODE: BatteryVFacBase = (1 << BATTERY_V_FAC_BITS) - 1;

/// Encode a relayed battery voltage, clamping real readings one code below the
/// sentinel so a present value can never collide with absence.
fn encode_shared_battery_v(
    shared_battery_v: Option<f32>,
) -> Integer<BatteryVFacBase, packed_bits::Bits<BATTERY_V_FAC_BITS>> {
    // NaN is absence that lost its `Option`, and it panics inside
    // `to_fixed_point_capped`, so it is folded back into absence here.
    match shared_battery_v.filter(|v| !v.is_nan()) {
        None => SHARED_BATTERY_V_UNAVAILABLE_CODE.into(),
        Some(v) => {
            let code: BatteryVFacBase = BatteryVFac::to_fixed_point_capped(v).into();
            code.min(SHARED_BATTERY_V_UNAVAILABLE_CODE - 1).into()
        }
    }
}

/// `None` when the packet carries the sentinel — the node relaying this voltage
/// has not reported it.
fn decode_shared_battery_v(
    shared_battery_v: Integer<BatteryVFacBase, packed_bits::Bits<BATTERY_V_FAC_BITS>>,
) -> Option<f32> {
    let code: BatteryVFacBase = shared_battery_v.into();
    if code == SHARED_BATTERY_V_UNAVAILABLE_CODE {
        None
    } else {
        Some(BatteryVFac::to_float(shared_battery_v))
    }
}

// EPM battery bus, a 4S-ish pack sitting well above the regulated rails.
// 9 bits over 12..17V, so (17 - 12) / (2^9 - 1) = 9.8mV per code.
//
// The floor was 0 V until 2026-08-18, on the reasoning that a collapsed or
// disconnected bus reading 0.0 V is a real fault the ground has to see and a
// floor of 11 would have decoded it as a plausible 11.0 V. That reasoning is
// still right; what changed is that the range below 12 V is now spent on one
// code instead of on two bits. `EPM_BATT_V_BELOW_RANGE_CODE` says "under
// 12 V" without saying by how much, which is the whole of what the ground
// needs — anything under 12 V is a fault, and the exact number is on the
// payload's SD card. The two bits went to the payload's experiment flags.
//
// Both ends of the range are therefore reserved: the top code is absence and
// the bottom code is under-range, so real readings live in 1..=510 and span
// 12.010..16.990 V.
//
// Lives here rather than in `telemetry`, for the same reason as `BatteryVFac`
// above: `TelemetryPacket` and `LowPowerTelemetryPacket` both carry this
// voltage, and the pad reading must not change meaning when the rocket drops
// into low power mode.
fixed_point_factory!(EpmBattVFac, f32, 12.0, 17.0, 0.01);

/// Bottom of the encodable range for the payload's EPM battery bus, volts.
///
/// Must match the floor of `EpmBattVFac` above — the factory takes literals,
/// so the two cannot be written once; `epm_batt_v_range_matches_the_factory`
/// pins them together.
pub const EPM_BATT_V_MIN: f32 = 12.0;

/// The code reserved for "the payload could not take this reading", spent on
/// `epm_batt_v` in both packets that carry it.
///
/// The top of the range is the cheapest code to give up: it is saturation,
/// which is already an approximation. [`encode_epm_batt_v`] clamps real values
/// one code below full scale, so a present reading can never collide with it.
const EPM_BATT_V_UNAVAILABLE_CODE: EpmBattVFacBase = (1 << EPM_BATT_V_FAC_BITS) - 1;

/// The code reserved for "the bus is below [`EPM_BATT_V_MIN`]", which is what
/// a collapsed or disconnected pack looks like.
///
/// Absence and under-range have to be different codes. A pack that has fallen
/// off the bottom of the range is not a pack nobody measured — it is the most
/// urgent measurement the payload can produce, and the ground has to be able
/// to tell it from silence. This is the code that keeps that distinction
/// alive now that 0.0 V is no longer representable.
const EPM_BATT_V_BELOW_RANGE_CODE: EpmBattVFacBase = 0;

/// What the packet says about the payload's EPM battery bus.
///
/// Three states rather than an `Option`, because the encodable range starts at
/// [`EPM_BATT_V_MIN`] and a bus below that is neither a reading nor a silence.
/// Collapsing it into either would lose the fault: as `None` it would read as
/// "the payload said nothing", and as `Some(12.0)` it would read as a healthy
/// pack.
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum EpmBattV {
    /// The payload could not read the bus, or has not reported one yet.
    Unavailable,
    /// Below [`EPM_BATT_V_MIN`] — a collapsed or disconnected pack. The packet
    /// deliberately does not say how far below; the exact millivolts are on
    /// the payload's own log, and no ground decision turns on the difference
    /// between a bus at 3 V and one at 9 V.
    BelowRange,
    /// A reading, volts.
    Volts(f32),
}

impl EpmBattV {
    /// The voltage, for callers that only want to plot or print a number.
    /// `None` for both non-readings — which is why it is not the primary
    /// accessor: a caller that wants to *display* the bus has to decide what
    /// `BelowRange` looks like, and this method is how it opts out of that.
    pub fn volts(self) -> Option<f32> {
        match self {
            Self::Volts(v) => Some(v),
            _ => None,
        }
    }
}

/// Encode the payload's EPM battery bus voltage from the millivolts the CAN
/// message carries. `None` — including the payload's own `0xFFFF` "could not
/// read this", which `CustomPayloadStatusMessage`'s accessors have already
/// turned into `None` — becomes the absence code; anything under
/// [`EPM_BATT_V_MIN`] becomes the under-range code.
fn encode_epm_batt_v(
    batt_mv: Option<u16>,
) -> Integer<EpmBattVFacBase, packed_bits::Bits<EPM_BATT_V_FAC_BITS>> {
    match batt_mv {
        None => EPM_BATT_V_UNAVAILABLE_CODE.into(),
        Some(mv) => {
            let volts = mv as f32 / 1000.0;
            if volts < EPM_BATT_V_MIN {
                return EPM_BATT_V_BELOW_RANGE_CODE.into();
            }
            // Clamped at both ends, for the same reason at each: a reading
            // must not land on either reserved code. A bus at exactly
            // 12.000 V is reported as 12.010 V rather than as under-range,
            // which costs one quantum at the very bottom of the range and
            // buys an unambiguous fault code.
            let code: EpmBattVFacBase = EpmBattVFac::to_fixed_point_capped(volts).into();
            code.clamp(
                EPM_BATT_V_BELOW_RANGE_CODE + 1,
                EPM_BATT_V_UNAVAILABLE_CODE - 1,
            )
            .into()
        }
    }
}

/// The read side of [`encode_epm_batt_v`]: both reserved codes come back as
/// their own variant, everything else as a voltage.
fn decode_epm_batt_v(
    batt_v: Integer<EpmBattVFacBase, packed_bits::Bits<EPM_BATT_V_FAC_BITS>>,
) -> EpmBattV {
    let code: EpmBattVFacBase = batt_v.into();
    if code == EPM_BATT_V_UNAVAILABLE_CODE {
        EpmBattV::Unavailable
    } else if code == EPM_BATT_V_BELOW_RANGE_CODE {
        EpmBattV::BelowRange
    } else {
        EpmBattV::Volts(EpmBattVFac::to_float(batt_v))
    }
}

#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Clone, Debug, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
pub enum VLPDownlinkPacket {
    Ack(AckPacket),
    LowPowerTelemetry(LowPowerTelemetryPacket),
    Telemetry(TelemetryPacket),
    SelfTestResult(SelfTestResultPacket),
    LandedTelemetry(LandedTelemetryPacket),
}

impl VLPDownlinkPacket {
    pub fn deserialize(data: &[u8]) -> Option<Self> {
        if data.is_empty() {
            return None;
        }
        let packet_type = data[0];
        let data = &data[1..];
        match packet_type {
            0 => AckPacket::deserialize(data).map(VLPDownlinkPacket::Ack),
            1 => {
                LowPowerTelemetryPacket::deserialize(data).map(VLPDownlinkPacket::LowPowerTelemetry)
            }
            2 => TelemetryPacket::deserialize(data).map(VLPDownlinkPacket::Telemetry),
            3 => SelfTestResultPacket::deserialize(data).map(VLPDownlinkPacket::SelfTestResult),
            4 => LandedTelemetryPacket::deserialize(data).map(VLPDownlinkPacket::LandedTelemetry),
            _ => None,
        }
    }

    pub fn packet_type(&self) -> u8 {
        match self {
            VLPDownlinkPacket::Ack(_) => 0,
            VLPDownlinkPacket::LowPowerTelemetry(_) => 1,
            VLPDownlinkPacket::Telemetry(_) => 2,
            VLPDownlinkPacket::SelfTestResult(_) => 3,
            VLPDownlinkPacket::LandedTelemetry(_) => 4,
        }
    }

    pub fn serialize(&self, mut buffer: &mut [u8]) -> usize {
        buffer[0] = self.packet_type();
        buffer = &mut buffer[1..];

        1 + match self {
            VLPDownlinkPacket::Ack(packet) => packet.serialize(buffer),
            VLPDownlinkPacket::LowPowerTelemetry(packet) => packet.serialize(buffer),
            VLPDownlinkPacket::Telemetry(packet) => packet.serialize(buffer),
            VLPDownlinkPacket::SelfTestResult(packet) => packet.serialize(buffer),
            VLPDownlinkPacket::LandedTelemetry(packet) => packet.serialize(buffer),
        }
    }
}

#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Clone, Debug, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
pub enum VLPUplinkPacket {
    ChangeMode(ChangeModePacket),
    Reset(ResetPacket),
    AMPOutputOverwrite(AMPOutputOverwritePacket),
    FirePyro(FirePyroPacket),
    SetTargetApogee(SetTargetApogeePacket)
}

impl VLPUplinkPacket {
    pub fn deserialize(data: &[u8]) -> Option<Self> {
        if data.is_empty() {
            return None;
        }
        let packet_type = data[0];
        let data = &data[1..];
        match packet_type {
            0 => ChangeModePacket::deserialize(data).map(VLPUplinkPacket::ChangeMode),
            1 => ResetPacket::deserialize(data).map(VLPUplinkPacket::Reset),
            2 => {
                AMPOutputOverwritePacket::deserialize(data).map(VLPUplinkPacket::AMPOutputOverwrite)
            }
            3 => FirePyroPacket::deserialize(data).map(VLPUplinkPacket::FirePyro),
            4 => SetTargetApogeePacket::deserialize(data).map(VLPUplinkPacket::SetTargetApogee),
            _ => None,
        }
    }

    pub fn serialize(&self, mut buffer: &mut [u8]) -> usize {
        buffer[0] = match self {
            VLPUplinkPacket::ChangeMode(_) => 0,
            VLPUplinkPacket::Reset(_) => 1,
            VLPUplinkPacket::AMPOutputOverwrite(_) => 2,
            VLPUplinkPacket::FirePyro(_) => 3,
            VLPUplinkPacket::SetTargetApogee(_) => 4,
        };
        buffer = &mut buffer[1..];

        1 + match self {
            VLPUplinkPacket::ChangeMode(packet) => packet.serialize(buffer),
            VLPUplinkPacket::Reset(packet) => packet.serialize(buffer),
            VLPUplinkPacket::AMPOutputOverwrite(packet) => packet.serialize(buffer),
            VLPUplinkPacket::FirePyro(packet) => packet.serialize(buffer),
            VLPUplinkPacket::SetTargetApogee(packet) => packet.serialize(buffer),
        }
    }
}
