use packed_struct::prelude::*;
use serde::{Deserialize, Serialize};

use super::{CanBusMessage, CanBusMessageEnum};

/// Reported for a reading that is invalid or unavailable.
///
/// Reserved: it is not a value, and no field in this message may carry it as
/// one. See [`MAX_REPORTED_PAYLOAD_READING`].
pub const PAYLOAD_READING_UNAVAILABLE: u16 = 0xFFFF;

/// Largest value any field here may report, one below the reserved sentinel.
///
/// Seven of the ten fields are safe from the collision by physics — 0xFFFF mA
/// is 65 A and 0xFFFF mV is 65 V, neither of which the stack can produce — but
/// `sem_actuator_*_steps` is not: `TelemetryPacket` documents it as "the full
/// u16 step range", so 65535 is a legal actuator position that would relay to
/// the ground as "SEM could not read this actuator". [`CustomPayloadStatusMessage::new`] caps every
/// reading here rather than only the three that need it, because "which fields
/// happen to be out of physical reach today" is not a rule anyone can be
/// expected to re-derive when a rail or a sensor is rescaled.
pub const MAX_REPORTED_PAYLOAD_READING: u16 = PAYLOAD_READING_UNAVAILABLE - 1;

/// Reported for a load cell reading SEM could not take.
///
/// [`PAYLOAD_READING_UNAVAILABLE`] cannot serve here. The load cells are
/// signed, and its bit pattern is a legal reading of minus one centinewton —
/// one gram of compression, which is exactly the kind of near-zero reading an
/// idle channel produces all day. So this field spends the bottom of its range
/// instead of the top, the opposite of every unsigned field above: for a
/// reading centred on zero the extreme end is the cheap one either way, and
/// `i16::MIN` is the only code that is not already spoken for.
pub const PAYLOAD_LOAD_CELL_UNAVAILABLE: i16 = i16::MIN;

/// Most compressive load a channel may report, one above the reserved sentinel.
///
/// The mirror of [`MAX_REPORTED_PAYLOAD_READING`]: 327.67 N of tension is
/// reportable, 327.67 N of compression is not, and a channel pressed that hard
/// reports 327.66 N rather than reporting itself unreadable.
pub const MIN_REPORTED_PAYLOAD_LOAD_CELL: i16 = PAYLOAD_LOAD_CELL_UNAVAILABLE + 1;

/// State of one fracture-experiment channel, unpacked from
/// [`CustomPayloadStatusMessage::experiment_flags`].
///
/// Seven flags per channel, three channels, packed as seven groups of three
/// bits: group `g` occupies bits `3g..3g+3` of the word, and within a group
/// bit 0 is channel 1, bit 1 is channel 2, bit 2 is channel 3. Grouped by flag
/// rather than by channel because that is the layout the payload's own
/// firmware packs, and a second layout here would be a second thing to get
/// wrong.
///
/// A channel that is not fitted reports [`enabled`](Self::enabled) clear,
/// every other flag clear, and its load cell absent. That is "absent", not
/// "failed" — do not read a clear `finished` on an unfitted channel as an
/// experiment that did not run.
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Ord, PartialOrd, Serialize, Deserialize)]
pub struct ExperimentChannelFlags {
    /// The sample on this channel has fractured.
    pub fractured: bool,
    /// This experiment ran to completion.
    pub finished: bool,
    /// This channel is in its fault state.
    pub fault: bool,
    /// This channel has a valid home reference.
    pub homed: bool,
    /// Closure reached the commanded target, which is the launch commit signal.
    pub closure_confirmed: bool,
    /// This channel is fitted and compiled in. Clear means absent, not failed.
    pub enabled: bool,
    /// This channel holds closure and streams load.
    pub monitoring: bool,
}

impl ExperimentChannelFlags {
    /// Bit position of flag group `group` for channel index `channel` (0..3).
    ///
    /// The one place the layout is written down; [`Self::from_raw`] and
    /// [`Self::to_raw`] both go through it, so they cannot disagree.
    const fn bit(group: u32, channel: usize) -> u32 {
        3 * group + channel as u32
    }

    /// Unpack one channel (index 0..3, i.e. experiment channels 1..3) from the
    /// packed word.
    pub fn from_raw(raw: u32, channel: usize) -> Self {
        let flag = |group: u32| raw & (1 << Self::bit(group, channel)) != 0;
        Self {
            fractured: flag(0),
            finished: flag(1),
            fault: flag(2),
            homed: flag(3),
            closure_confirmed: flag(4),
            enabled: flag(5),
            monitoring: flag(6),
        }
    }

    /// This channel's contribution to the packed word. Bits belonging to the
    /// other two channels are zero, so the three ORed together are the word.
    pub fn to_raw(&self, channel: usize) -> u32 {
        let flag = |set: bool, group: u32| {
            if set {
                1 << Self::bit(group, channel)
            } else {
                0
            }
        };
        flag(self.fractured, 0)
            | flag(self.finished, 1)
            | flag(self.fault, 2)
            | flag(self.homed, 3)
            | flag(self.closure_confirmed, 4)
            | flag(self.enabled, 5)
            | flag(self.monitoring, 6)
    }
}

/// Extended EPM / SEM telemetry from the payload SDRM node, sent every 500ms.
///
/// Supplementary to `NodeStatusMessage`, which stays the primary go/no-go source.
/// Deliberately does not repeat `uptime_s`, `health`, `mode` or the stack flags, so
/// the two messages can not drift apart.
///
/// Everything here is relayed from EPM / SEM on the intra-stack bus, not measured
/// by SDRM: EPM reports the battery bus voltage and the load current of all six
/// switched rails, SEM reports the linear actuator positions, the fracture load
/// on each experiment channel, and how far each experiment has got.
///
/// Bytes 0..20 are the original twenty and have never moved; the load cells and
/// the experiment flag word were appended in 2026-08 when the payload started
/// sequencing its own experiments off the flight stage and the ground had no
/// way to see whether that sequence was working. Appending does not make the
/// two lengths interchangeable — the message codec is exact-length, so a
/// 20-byte frame does not decode as a short 30-byte one and vice versa. Both
/// ends move together or neither does.
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(PackedStruct, Clone, Debug, PartialEq, Eq, Ord, PartialOrd, Serialize, Deserialize)]
#[packed_struct(bit_numbering = "msb0", endian = "msb", size_bytes = "30")]
#[repr(C)]
pub struct CustomPayloadStatusMessage {
    /// EPM battery bus voltage
    pub epm_batt_mv: u16,

    /// System 3.3V rail load current
    pub epm_sys_3v3_ma: u16,
    /// System 5V rail load current
    pub epm_sys_5v_ma: u16,
    /// Peripheral 3.3V rail load current
    pub epm_per_3v3_ma: u16,
    /// Peripheral 5V rail load current
    pub epm_per_5v_ma: u16,
    /// Peripheral 9V rail load current
    pub epm_per_9v_ma: u16,
    /// Peripheral 12V rail load current
    pub epm_per_12v_ma: u16,

    /// SEM linear actuator position, experiment channel 1
    pub sem_actuator_1_steps: u16,
    /// SEM linear actuator position, experiment channel 2
    pub sem_actuator_2_steps: u16,
    /// SEM linear actuator position, experiment channel 3
    pub sem_actuator_3_steps: u16,

    /// Fracture load on experiment channel 1, centinewtons, tension positive
    pub sem_load_cell_1_cn: i16,
    /// Fracture load on experiment channel 2, centinewtons, tension positive
    pub sem_load_cell_2_cn: i16,
    /// Fracture load on experiment channel 3, centinewtons, tension positive
    pub sem_load_cell_3_cn: i16,

    /// Per-channel experiment state, seven flags per channel. Decode with
    /// [`Self::experiment_flags`] rather than by hand; the layout lives in
    /// [`ExperimentChannelFlags`]. Bits 21..32 are spare and sent as zero.
    pub experiment_flags: u32,
}

impl CustomPayloadStatusMessage {
    /// Build a message from readings that may or may not have been taken,
    /// which is the only way to build one that cannot lie.
    ///
    /// The struct's fields are public raw `u16`s because a `PackedStruct`
    /// literal has to be built from them, and until this constructor existed
    /// that literal was the only way to build the message — so nothing stopped
    /// a caller writing a real 65535-step actuator position straight into a
    /// field whose 65535 means "no reading". The sender is the only place that
    /// collision can be resolved (once it is on the wire, the two are the same
    /// sixteen bits), so this is where it is resolved: `Some` is clamped to
    /// [`MAX_REPORTED_PAYLOAD_READING`] and only `None` produces
    /// [`PAYLOAD_READING_UNAVAILABLE`].
    ///
    /// Losing one step at the very top of the actuator range is the whole cost.
    /// The alternative — an actuator at full extension reporting itself as
    /// unreadable — is worse than an actuator at full extension reporting
    /// itself one step short of it.
    ///
    /// The load cells get the same treatment against their own sentinel, from
    /// the other end of the range: see [`PAYLOAD_LOAD_CELL_UNAVAILABLE`]. The
    /// experiment flags need none — every bit pattern of that word is a legal
    /// state, and "the payload has said nothing" is already carried by the
    /// absence of the message rather than by a value inside it.
    ///
    /// Rail index order: 0 `SYS_3V3`, 1 `SYS_5V`, 2 `PER_3V3`, 3 `PER_5V`,
    /// 4 `PER_9V`, 5 `PER_12V`. Actuator, load cell and flag index order:
    /// experiment channels 1..3.
    pub fn new(
        epm_batt_mv: Option<u16>,
        epm_rail_ma: [Option<u16>; 6],
        sem_actuator_steps: [Option<u16>; 3],
        sem_load_cell_cn: [Option<i16>; 3],
        experiment_flags: [ExperimentChannelFlags; 3],
    ) -> Self {
        Self {
            epm_batt_mv: Self::encode(epm_batt_mv),

            epm_sys_3v3_ma: Self::encode(epm_rail_ma[0]),
            epm_sys_5v_ma: Self::encode(epm_rail_ma[1]),
            epm_per_3v3_ma: Self::encode(epm_rail_ma[2]),
            epm_per_5v_ma: Self::encode(epm_rail_ma[3]),
            epm_per_9v_ma: Self::encode(epm_rail_ma[4]),
            epm_per_12v_ma: Self::encode(epm_rail_ma[5]),

            sem_actuator_1_steps: Self::encode(sem_actuator_steps[0]),
            sem_actuator_2_steps: Self::encode(sem_actuator_steps[1]),
            sem_actuator_3_steps: Self::encode(sem_actuator_steps[2]),

            sem_load_cell_1_cn: Self::encode_load_cell(sem_load_cell_cn[0]),
            sem_load_cell_2_cn: Self::encode_load_cell(sem_load_cell_cn[1]),
            sem_load_cell_3_cn: Self::encode_load_cell(sem_load_cell_cn[2]),

            experiment_flags: Self::pack_experiment_flags(experiment_flags),
        }
    }

    /// Fold three channels' flags into the word that goes on the wire.
    ///
    /// Public because the payload side has to build the word and this is the
    /// only implementation of the layout that is checked against the Rust
    /// side in CI.
    pub fn pack_experiment_flags(channels: [ExperimentChannelFlags; 3]) -> u32 {
        channels
            .iter()
            .enumerate()
            .fold(0, |raw, (channel, flags)| raw | flags.to_raw(channel))
    }

    /// The write-side inverse of [`Self::reading`]: absence becomes the
    /// reserved code, and a present reading is capped one below it so it can
    /// never become absence on the way out.
    pub fn encode(reading: Option<u16>) -> u16 {
        match reading {
            None => PAYLOAD_READING_UNAVAILABLE,
            Some(value) => value.min(MAX_REPORTED_PAYLOAD_READING),
        }
    }

    /// [`Self::encode`] for a signed load cell: same rule, opposite end of the
    /// range. A reading is clamped *up* to [`MIN_REPORTED_PAYLOAD_LOAD_CELL`],
    /// so a channel under extreme compression reports one centinewton short of
    /// full scale rather than reporting itself unreadable.
    pub fn encode_load_cell(reading: Option<i16>) -> i16 {
        match reading {
            None => PAYLOAD_LOAD_CELL_UNAVAILABLE,
            Some(value) => value.max(MIN_REPORTED_PAYLOAD_LOAD_CELL),
        }
    }

    /// Every reading unavailable and every experiment flag clear, e.g. before
    /// EPM / SEM have reported.
    pub fn new_unavailable() -> Self {
        Self::new(
            None,
            [None; 6],
            [None; 3],
            [None; 3],
            [ExperimentChannelFlags::default(); 3],
        )
    }

    /// `None` if the reading is invalid or unavailable.
    ///
    /// The struct fields stay raw `u16` because that is what goes on the wire
    /// and what a `PackedStruct` literal has to be built from, but nothing
    /// downstream should be reading them directly — every accessor below runs
    /// the field through here first, so a caller gets an `Option` it has to
    /// deal with rather than a `0xFFFF` it has to remember to check for. Each
    /// accessor is deliberately named exactly like its field, so
    /// `msg.epm_batt_mv()` is the obvious thing to reach for and
    /// `msg.epm_batt_mv` is the thing you have to go out of your way to write.
    pub fn reading(raw: u16) -> Option<u16> {
        if raw == PAYLOAD_READING_UNAVAILABLE {
            None
        } else {
            Some(raw)
        }
    }

    /// [`Self::reading`] for a signed load cell. `None` if SEM did not report
    /// that channel — which includes every channel that is not fitted.
    pub fn load_cell_reading(raw: i16) -> Option<i16> {
        if raw == PAYLOAD_LOAD_CELL_UNAVAILABLE {
            None
        } else {
            Some(raw)
        }
    }

    /// EPM battery bus voltage, mV. `None` if EPM could not read it.
    pub fn epm_batt_mv(&self) -> Option<u16> {
        Self::reading(self.epm_batt_mv)
    }

    /// System 3.3V rail load current, mA. `None` if EPM could not read it.
    /// A rail that is switched off reads `Some(0)`, not `None`.
    pub fn epm_sys_3v3_ma(&self) -> Option<u16> {
        Self::reading(self.epm_sys_3v3_ma)
    }

    /// System 5V rail load current, mA. `None` if EPM could not read it.
    pub fn epm_sys_5v_ma(&self) -> Option<u16> {
        Self::reading(self.epm_sys_5v_ma)
    }

    /// Peripheral 3.3V rail load current, mA. `None` if EPM could not read it.
    pub fn epm_per_3v3_ma(&self) -> Option<u16> {
        Self::reading(self.epm_per_3v3_ma)
    }

    /// Peripheral 5V rail load current, mA. `None` if EPM could not read it.
    pub fn epm_per_5v_ma(&self) -> Option<u16> {
        Self::reading(self.epm_per_5v_ma)
    }

    /// Peripheral 9V rail load current, mA. `None` if EPM could not read it.
    pub fn epm_per_9v_ma(&self) -> Option<u16> {
        Self::reading(self.epm_per_9v_ma)
    }

    /// Peripheral 12V rail load current, mA. `None` if EPM could not read it.
    pub fn epm_per_12v_ma(&self) -> Option<u16> {
        Self::reading(self.epm_per_12v_ma)
    }

    /// Experiment channel 1 actuator position, steps. `None` if SEM could not
    /// read it. An actuator parked at its home position reads `Some(0)`.
    pub fn sem_actuator_1_steps(&self) -> Option<u16> {
        Self::reading(self.sem_actuator_1_steps)
    }

    /// Experiment channel 2 actuator position, steps. `None` if SEM could not
    /// read it.
    pub fn sem_actuator_2_steps(&self) -> Option<u16> {
        Self::reading(self.sem_actuator_2_steps)
    }

    /// Experiment channel 3 actuator position, steps. `None` if SEM could not
    /// read it.
    pub fn sem_actuator_3_steps(&self) -> Option<u16> {
        Self::reading(self.sem_actuator_3_steps)
    }

    /// The six rail currents in the stack's rail index order (0 `SYS_3V3`,
    /// 1 `SYS_5V`, 2 `PER_3V3`, 3 `PER_5V`, 4 `PER_9V`, 5 `PER_12V`), each
    /// `None` if EPM could not read that rail. Rails fail to read
    /// individually — one dead INA does not take the other five with it — so
    /// this is an array of `Option`, not an `Option` of an array.
    pub fn rail_ma(&self) -> [Option<u16>; 6] {
        [
            self.epm_sys_3v3_ma(),
            self.epm_sys_5v_ma(),
            self.epm_per_3v3_ma(),
            self.epm_per_5v_ma(),
            self.epm_per_9v_ma(),
            self.epm_per_12v_ma(),
        ]
    }

    /// Actuator positions for experiment channels 1..3, each `None` if SEM
    /// could not read that channel.
    pub fn actuator_steps(&self) -> [Option<u16>; 3] {
        [
            self.sem_actuator_1_steps(),
            self.sem_actuator_2_steps(),
            self.sem_actuator_3_steps(),
        ]
    }

    /// Experiment channel 1 fracture load, centinewtons, tension positive.
    /// `None` if SEM did not report the channel. A channel resting at zero
    /// load reads `Some(0)`.
    pub fn sem_load_cell_1_cn(&self) -> Option<i16> {
        Self::load_cell_reading(self.sem_load_cell_1_cn)
    }

    /// Experiment channel 2 fracture load, centinewtons. `None` if SEM did not
    /// report the channel.
    pub fn sem_load_cell_2_cn(&self) -> Option<i16> {
        Self::load_cell_reading(self.sem_load_cell_2_cn)
    }

    /// Experiment channel 3 fracture load, centinewtons. `None` if SEM did not
    /// report the channel.
    pub fn sem_load_cell_3_cn(&self) -> Option<i16> {
        Self::load_cell_reading(self.sem_load_cell_3_cn)
    }

    /// Fracture loads for experiment channels 1..3, each `None` if SEM did not
    /// report that channel.
    pub fn load_cell_cn(&self) -> [Option<i16>; 3] {
        [
            self.sem_load_cell_1_cn(),
            self.sem_load_cell_2_cn(),
            self.sem_load_cell_3_cn(),
        ]
    }

    /// Experiment state for channels 1..3.
    ///
    /// Not an `Option`: every bit pattern is a legal state, and a channel with
    /// nothing to say says so by reporting `enabled` clear rather than by
    /// going absent. "The payload never sent this message" is carried one
    /// level up, by the absence of the message.
    pub fn experiment_flags(&self) -> [ExperimentChannelFlags; 3] {
        core::array::from_fn(|channel| {
            ExperimentChannelFlags::from_raw(self.experiment_flags, channel)
        })
    }
}

impl CanBusMessage for CustomPayloadStatusMessage {
    fn priority(&self) -> u8 {
        5
    }
}

impl Into<CanBusMessageEnum> for CustomPayloadStatusMessage {
    fn into(self) -> CanBusMessageEnum {
        CanBusMessageEnum::CustomPayloadStatus(self)
    }
}

#[cfg(test)]
mod test {
    use super::*;
    use crate::{
        can_bus::messages::{CUSTOM_PAYLOAD_STATUS_MESSAGE_TYPE, tests as can_bus_messages_test},
        tests::init_logger,
        utils::FixedLenSerializable,
    };

    fn create_test_messages() -> Vec<CanBusMessageEnum> {
        vec![
            CustomPayloadStatusMessage {
                epm_batt_mv: 0,
                epm_sys_3v3_ma: 0,
                epm_sys_5v_ma: 0,
                epm_per_3v3_ma: 0,
                epm_per_5v_ma: 0,
                epm_per_9v_ma: 0,
                epm_per_12v_ma: 0,
                sem_actuator_1_steps: 0,
                sem_actuator_2_steps: 0,
                sem_actuator_3_steps: 0,
                sem_load_cell_1_cn: 0,
                sem_load_cell_2_cn: 0,
                sem_load_cell_3_cn: 0,
                experiment_flags: 0,
            }
            .into(),
            CustomPayloadStatusMessage::new_unavailable().into(),
            CustomPayloadStatusMessage {
                epm_batt_mv: 12600,
                epm_sys_3v3_ma: 120,
                epm_sys_5v_ma: 340,
                epm_per_3v3_ma: 55,
                epm_per_5v_ma: 780,
                epm_per_9v_ma: 1500,
                epm_per_12v_ma: 2400,
                sem_actuator_1_steps: 0,
                sem_actuator_2_steps: 1200,
                sem_actuator_3_steps: 34567,
                sem_load_cell_1_cn: 0,
                sem_load_cell_2_cn: -250,
                sem_load_cell_3_cn: 12345,
                // ch1 enabled+homed+monitoring, ch2 the same plus finished and
                // fractured, ch3 not fitted.
                experiment_flags: 0b0_0100_1001_1011_0010_1010,
            }
            .into(),
        ]
    }

    #[test]
    fn test_serialize_deserialize() {
        init_logger();
        can_bus_messages_test::test_serialize_deserialize(create_test_messages());
    }

    #[test]
    fn test_reading() {
        let message = CustomPayloadStatusMessage::new_unavailable();
        assert_eq!(
            CustomPayloadStatusMessage::reading(message.epm_batt_mv),
            None
        );
        assert_eq!(CustomPayloadStatusMessage::reading(0), Some(0));
        assert_eq!(CustomPayloadStatusMessage::reading(12600), Some(12600));
    }

    #[test]
    fn test_accessors() {
        let message = CustomPayloadStatusMessage {
            epm_batt_mv: 12600,
            epm_sys_3v3_ma: 1,
            epm_sys_5v_ma: 2,
            epm_per_3v3_ma: 3,
            epm_per_5v_ma: 4,
            epm_per_9v_ma: 5,
            epm_per_12v_ma: 6,
            sem_actuator_1_steps: 7,
            sem_actuator_2_steps: 8,
            sem_actuator_3_steps: 9,
            sem_load_cell_1_cn: 10,
            sem_load_cell_2_cn: -11,
            sem_load_cell_3_cn: 12,
            experiment_flags: 0,
        };
        assert_eq!(message.epm_batt_mv(), Some(12600));
        assert_eq!(
            message.rail_ma(),
            [Some(1), Some(2), Some(3), Some(4), Some(5), Some(6)]
        );
        assert_eq!(message.actuator_steps(), [Some(7), Some(8), Some(9)]);
        assert_eq!(message.load_cell_cn(), [Some(10), Some(-11), Some(12)]);
    }

    /// A reading that is unavailable has to stay unavailable all the way to
    /// the caller, and a genuine 0 has to survive as a 0 — a switched-off rail
    /// and an actuator at its home position both read 0 in normal operation.
    #[test]
    fn unavailable_readings_are_none_and_zeros_are_not() {
        let message = CustomPayloadStatusMessage::new_unavailable();
        assert_eq!(message.epm_batt_mv(), None);
        assert_eq!(message.rail_ma(), [None; 6]);
        assert_eq!(message.actuator_steps(), [None; 3]);

        let message = CustomPayloadStatusMessage {
            epm_batt_mv: 0,
            epm_sys_3v3_ma: 0,
            epm_sys_5v_ma: PAYLOAD_READING_UNAVAILABLE,
            epm_per_3v3_ma: 0,
            epm_per_5v_ma: 0,
            epm_per_9v_ma: 0,
            epm_per_12v_ma: 0,
            sem_actuator_1_steps: 0,
            sem_actuator_2_steps: PAYLOAD_READING_UNAVAILABLE,
            sem_actuator_3_steps: 0,
            sem_load_cell_1_cn: 0,
            sem_load_cell_2_cn: PAYLOAD_LOAD_CELL_UNAVAILABLE,
            sem_load_cell_3_cn: -1,
            experiment_flags: 0,
        };
        assert_eq!(message.epm_batt_mv(), Some(0));
        assert_eq!(
            message.rail_ma(),
            [Some(0), None, Some(0), Some(0), Some(0), Some(0)]
        );
        assert_eq!(message.actuator_steps(), [Some(0), None, Some(0)]);
        // -1cN shares its sixteen bits with the unsigned sentinel and has to
        // survive as a reading anyway; that collision is the whole reason the
        // load cells have a sentinel of their own.
        assert_eq!(message.load_cell_cn(), [Some(0), None, Some(-1)]);
    }

    /// 65535 is a legal `sem_actuator_*_steps` value — the field is documented
    /// as the full u16 step range — and it is also the code for "no reading".
    /// The constructor is where that collision has to be broken, because on the
    /// wire the two are indistinguishable.
    #[test]
    fn a_full_scale_actuator_does_not_report_itself_as_unreadable() {
        let message = CustomPayloadStatusMessage::new(
            Some(u16::MAX),
            [Some(u16::MAX); 6],
            [Some(u16::MAX); 3],
            [Some(i16::MIN); 3],
            [ExperimentChannelFlags::default(); 3],
        );

        assert_eq!(message.epm_batt_mv(), Some(MAX_REPORTED_PAYLOAD_READING));
        assert_eq!(message.rail_ma(), [Some(MAX_REPORTED_PAYLOAD_READING); 6]);
        assert_eq!(
            message.actuator_steps(),
            [Some(MAX_REPORTED_PAYLOAD_READING); 3]
        );
        for steps in message.actuator_steps() {
            assert_ne!(steps, None);
        }

        assert_eq!(
            message.load_cell_cn(),
            [Some(MIN_REPORTED_PAYLOAD_LOAD_CELL); 3]
        );
    }

    /// The constructor has to carry absence and a genuine zero through
    /// unchanged; everything below 65535 is a value.
    #[test]
    fn constructor_round_trips_absence_and_zero() {
        let message = CustomPayloadStatusMessage::new(
            Some(0),
            [Some(0), None, Some(120), Some(0), None, Some(2400)],
            [Some(0), None, Some(34567)],
            [Some(0), None, Some(-32000)],
            [ExperimentChannelFlags::default(); 3],
        );

        assert_eq!(message.epm_batt_mv(), Some(0));
        assert_eq!(
            message.rail_ma(),
            [Some(0), None, Some(120), Some(0), None, Some(2400)]
        );
        assert_eq!(message.actuator_steps(), [Some(0), None, Some(34567)]);
        assert_eq!(message.load_cell_cn(), [Some(0), None, Some(-32000)]);

        assert_eq!(
            CustomPayloadStatusMessage::new_unavailable(),
            CustomPayloadStatusMessage::new(
                None,
                [None; 6],
                [None; 3],
                [None; 3],
                [ExperimentChannelFlags::default(); 3]
            )
        );
    }

    /// Every flag of every channel sits exactly where the payload's
    /// `packExperimentFlags` puts it: group `g` at bits `3g..3g+3`, channel
    /// `c` at bit `3g + c`. Pinned one flag at a time, because a layout that
    /// is only ever checked with everything set cannot catch two flags that
    /// swapped places.
    #[test]
    fn experiment_flag_bit_positions() {
        let setters: [(u32, fn(&mut ExperimentChannelFlags)); 7] = [
            (0, |f| f.fractured = true),
            (1, |f| f.finished = true),
            (2, |f| f.fault = true),
            (3, |f| f.homed = true),
            (4, |f| f.closure_confirmed = true),
            (5, |f| f.enabled = true),
            (6, |f| f.monitoring = true),
        ];

        let mut everything = [ExperimentChannelFlags::default(); 3];
        for (group, apply) in setters {
            for channel in 0..3 {
                let mut only_this = [ExperimentChannelFlags::default(); 3];
                apply(&mut only_this[channel]);

                let raw = CustomPayloadStatusMessage::pack_experiment_flags(only_this);
                assert_eq!(
                    raw,
                    1 << (3 * group + channel as u32),
                    "group {} channel {} misplaced",
                    group,
                    channel
                );
                assert_eq!(
                    ExperimentChannelFlags::from_raw(raw, channel),
                    only_this[channel],
                    "group {} channel {} did not round-trip",
                    group,
                    channel
                );

                apply(&mut everything[channel]);
            }
        }

        // 21 bits used, 11 spare and clear even with every flag set.
        assert_eq!(
            CustomPayloadStatusMessage::pack_experiment_flags(everything),
            0x001F_FFFF
        );
    }

    /// The appended block sits at the byte offsets the payload ICD names:
    /// load cells at 20/22/24 as big-endian `i16`, the flag word at 26..30 as
    /// a big-endian `u32`. This is the interop contract with a C++ decoder
    /// that reads the buffer by offset, so it is pinned here rather than left
    /// to whatever `packed_struct` happens to do with a field order.
    #[test]
    fn the_appended_block_is_where_the_icd_says() {
        assert_eq!(CustomPayloadStatusMessage::serialized_len(), 30);

        let message = CustomPayloadStatusMessage {
            epm_batt_mv: 12600,
            epm_sys_3v3_ma: 120,
            epm_sys_5v_ma: 340,
            epm_per_3v3_ma: 55,
            epm_per_5v_ma: 780,
            epm_per_9v_ma: 1500,
            epm_per_12v_ma: 2400,
            sem_actuator_1_steps: 0,
            sem_actuator_2_steps: 1200,
            sem_actuator_3_steps: 34567,
            sem_load_cell_1_cn: 1000,
            sem_load_cell_2_cn: -1000,
            sem_load_cell_3_cn: PAYLOAD_LOAD_CELL_UNAVAILABLE,
            experiment_flags: 0x0012_3456,
        };

        let mut buffer = [0u8; 30];
        FixedLenSerializable::serialize(&message, &mut buffer);

        assert_eq!(&buffer[20..22], &1000i16.to_be_bytes());
        assert_eq!(&buffer[22..24], &(-1000i16).to_be_bytes());
        assert_eq!(&buffer[24..26], &[0x80, 0x00]);
        assert_eq!(&buffer[26..30], &0x0012_3456u32.to_be_bytes());

        // The original twenty bytes did not move.
        assert_eq!(
            &buffer[..20],
            &[
                0x31, 0x38, 0x00, 0x78, 0x01, 0x54, 0x00, 0x37, 0x03, 0x0C, 0x05, 0xDC, 0x09, 0x60,
                0x00, 0x00, 0x04, 0xB0, 0x87, 0x07,
            ]
        );
    }

    /// A 20-byte type 35 is not a short 30-byte one. Appending a block to a
    /// fixed-length message is a breaking change on the wire, however much it
    /// looks like an extension, and the decoder says so by rejecting the old
    /// length outright rather than filling the tail with zeros.
    #[test]
    fn the_old_twenty_byte_length_does_not_decode() {
        let message = CustomPayloadStatusMessage::new_unavailable();
        let mut buffer = [0u8; 30];
        FixedLenSerializable::serialize(&message, &mut buffer);

        assert!(
            CanBusMessageEnum::deserialize(CUSTOM_PAYLOAD_STATUS_MESSAGE_TYPE, &buffer).is_some()
        );
        assert!(
            CanBusMessageEnum::deserialize(CUSTOM_PAYLOAD_STATUS_MESSAGE_TYPE, &buffer[..20])
                .is_none()
        );
    }

    #[test]
    fn create_reference_data() {
        init_logger();
        can_bus_messages_test::create_reference_data(
            create_test_messages(),
            "custom_payload_status",
        );
    }
}
