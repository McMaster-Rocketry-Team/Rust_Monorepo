//! What a baro innovation gate did with one sample.
//!
//! Both estimators gate their baro channel the same way — reject a reading
//! that disagrees with the prediction by more than a threshold, and give up
//! and resync if the disagreement persists — so both report the outcome in
//! this shape. It is the return value of the filter update rather than state
//! stored on the filter, because a resync happens on exactly one sample: a
//! caller that polls for it afterwards would have to be lucky, and the SD log
//! polls at a different rate than the filters run.

/// The fate of one baro measurement at the innovation gate.
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BaroGateOutcome {
    /// Inside the gate, fused normally.
    Accepted,
    /// Outside the gate and discarded — the filter coasts on its prediction.
    /// A transient (an ejection blast, a shock-disturbed port) looks like a
    /// run of these and nothing else.
    Rejected,
    /// Outside the gate, but the run had lasted long enough that the filter,
    /// not the sensor, was judged wrong: altitude was snapped to the
    /// measurement. Ends a rejection run and means the altitude either side
    /// of this sample is discontinuous.
    Resynced,
}

impl BaroGateOutcome {
    /// Whether the gate threw this sample out (either kind of rejection).
    pub fn rejected(&self) -> bool {
        matches!(self, Self::Rejected | Self::Resynced)
    }

    pub fn resynced(&self) -> bool {
        matches!(self, Self::Resynced)
    }
}
