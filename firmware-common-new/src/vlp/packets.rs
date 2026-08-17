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
// 11 bits over 0..17V, so (17 - 0) / (2^11 - 1) = 8.3mV per code. The floor is
// 0 rather than 11 V because a collapsed / disconnected battery bus reading
// 0.0 V is a real fault the ground should see; a floor of 11 would have
// decoded that as a plausible 11.0 V. Absence is the all-ones code (see
// `EPM_BATT_V_UNAVAILABLE_CODE`), not 0, precisely so that 0.0 V stays
// available for that fault. Real readings therefore cap one code below full
// scale, at 16.992 V.
//
// Lives here rather than in `telemetry`, for the same reason as `BatteryVFac`
// above: `TelemetryPacket` and `LowPowerTelemetryPacket` both carry this
// voltage, and the pad reading must not change meaning when the rocket drops
// into low power mode.
fixed_point_factory!(EpmBattVFac, f32, 0.0, 17.0, 0.01);

/// The code reserved for "the payload could not take this reading", spent on
/// `epm_batt_v` in both packets that carry it.
///
/// The top of the range is the cheapest code to give up: the bottom is a
/// reading the ground genuinely needs to be able to see — 0.0 V is a collapsed
/// or disconnected pack — whereas the top is saturation, which is already an
/// approximation. [`encode_epm_batt_v`] therefore clamps real values one code
/// below full scale, so a present reading can never collide with the sentinel.
const EPM_BATT_V_UNAVAILABLE_CODE: EpmBattVFacBase = (1 << EPM_BATT_V_FAC_BITS) - 1;

/// Encode the payload's EPM battery bus voltage from the millivolts the CAN
/// message carries. `None` — including the payload's own `0xFFFF` "could not
/// read this", which `CustomPayloadStatusMessage`'s accessors have already
/// turned into `None` — becomes the sentinel.
fn encode_epm_batt_v(
    batt_mv: Option<u16>,
) -> Integer<EpmBattVFacBase, packed_bits::Bits<EPM_BATT_V_FAC_BITS>> {
    match batt_mv {
        None => EPM_BATT_V_UNAVAILABLE_CODE.into(),
        Some(mv) => {
            let code: EpmBattVFacBase =
                EpmBattVFac::to_fixed_point_capped(mv as f32 / 1000.0).into();
            code.min(EPM_BATT_V_UNAVAILABLE_CODE - 1).into()
        }
    }
}

/// `None` when the packet carries the sentinel. A real 0.0 V decodes as
/// `Some(0.0)`, which is the whole point of putting absence at the top.
fn decode_epm_batt_v(
    batt_v: Integer<EpmBattVFacBase, packed_bits::Bits<EPM_BATT_V_FAC_BITS>>,
) -> Option<f32> {
    let code: EpmBattVFacBase = batt_v.into();
    if code == EPM_BATT_V_UNAVAILABLE_CODE {
        None
    } else {
        Some(EpmBattVFac::to_float(batt_v))
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
