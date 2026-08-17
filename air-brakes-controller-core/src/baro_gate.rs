//! What a baro innovation gate did with one sample.
//!
//! Both estimators gate their baro channel the same way — reject a reading
//! that disagrees with the prediction by more than a threshold, and give up
//! and resync if the disagreement persists — so both report the outcome in
//! this shape. It is the return value of the update, all the way out through
//! [`FlightEstimators::update`], and is stored nowhere.
//!
//! Both estimators used to keep it in a `last_baro_gate` field with an
//! accessor, and this doc used to justify that with a polling gap that did
//! not exist: the SD log has never polled on a clock of its own — its one
//! caller read the outcome in the same critical section as the `update` that
//! produced it. So the field bought nothing and cost a contract ("read this
//! immediately after `update`, before the next sample overwrites it") that
//! the type could not enforce. Returning the value deletes the contract:
//! there is no stale value to read because there is no value to read.
//!
//! The one behaviour this changed is the sample with no IMU on it. The
//! airbrakes half is skipped there, so its stored field used to hold the
//! PREVIOUS sample's outcome and the SD record repeated it. Now
//! [`FlightEstimators::update`] synthesises `Accepted` for that sample —
//! nothing was fused, so nothing was rejected — which is what every other
//! "no gate ran" path in both estimators already reported.
//!
//! [`FlightEstimators::update`]: crate::FlightEstimators::update

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
