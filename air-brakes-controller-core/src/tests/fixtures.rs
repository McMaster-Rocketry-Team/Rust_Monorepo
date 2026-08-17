//! Base configurations shared by the host tests, built to be *overridden*.
//!
//! Every fixture here is a whole struct, and every call site that cares
//! about one of its fields is expected to say so with struct-update
//! syntax:
//!
//! ```ignore
//! FlightProfile {
//!     deployment: DeploymentProfile::Dual { .. },  // what this test asserts
//!     ..subsonic_profile()                         // everything it does not
//! }
//! ```
//!
//! The split is deliberate and the rule is one line long: **a number a test
//! asserts against stays at the call site.** A deployment altitude in a
//! deployment test, a chute delay in a timing test, a lockout duration in a
//! lockout test — those are the test, and a reader who has to open this
//! file to find them has been handed a worse test, not a shorter one. What
//! belongs in here is the background an assertion never reads: the
//! airframe the drag check is not being asked about, the ignition
//! threshold every scripted motor clears by 25%, the chute profile on a
//! flight that never gets near a chute.
//!
//! Three bases rather than one, because there are three unrelated things
//! being defaulted — the deployment half's profile, the LC'25 airframe, and
//! the airbrakes half's config — and a single fixture with a dozen
//! overrides at each call site would put every number back on screen while
//! making it harder to see which ones matter.
//!
//! Not in here on purpose: `tests::osiris_sim::osiris_config`, which is a
//! verbatim copy of the flown `FLIGHT_CONFIG` and has to stay one; and the
//! synthetic `cd` tables in `controller` — those tests exist to exercise
//! the table.

use crate::airbrakes_estimator::AirbrakesConfig;
use crate::baro_state_estimator::{DeploymentProfile, FlightProfile};
use crate::controller::RocketParameters;

/// Deployment-half profile for tests that are not about deployment.
///
/// Subsonic — `mach_lockout_duration_us: None` — with the 4 g ignition
/// threshold and a single-deploy at 300 m AGL with no delay. The scripted
/// motors in `baro_state_estimator::tests` pull 5.1-11.2 g of specific
/// force and both replayed flights pull more, so 4 g clears every one of
/// them with margin while sitting far above pad handling and wind.
///
/// The 300 m / 0 s chute profile is the "this flight never gets near a
/// chute, or gets nowhere near this altitude" case: every test that takes
/// it bare either never deploys at all, or deploys thousands of metres
/// above it. A test that *asserts* on a deployment altitude, on a chute
/// delay, or on the Mach lockout overrides that field at its own call
/// site — see [`crate::baro_state_estimator::tests`], where nine of the
/// thirteen call sites do exactly that.
pub fn subsonic_profile() -> FlightProfile {
    FlightProfile {
        mach_lockout_duration_us: None,
        ignition_detection_acc_threshold: 4.0 * 9.81,
        deployment: DeploymentProfile::Single {
            minimum_deployment_altitude_agl: 300.0,
            delay_us: 0,
        },
    }
}

/// The LC'25 airframe — the one both replayed flights fly, and the one the
/// drag check inverts.
///
/// `cd[0] * reference_area / burnout_mass` = 2.4e-4, which the flight
/// itself corroborates: measured drag deceleration over dynamic pressure
/// sits at 0.00022-0.00026 across the whole subsonic coast, and rises about
/// 40% through the transonic peak — which is why inverting with the
/// SUBSONIC Cd makes the check read high exactly while supersonic (see
/// [`AirbrakesConfig::rocket`]).
pub fn lc25_rocket() -> RocketParameters {
    RocketParameters {
        burnout_mass: 17.607,
        cd: [0.47044, 0.5082, 0.57784, 0.665, 0.74313],
        reference_area: 0.008982476,
    }
}

/// The airbrakes half on the LC'25 airframe with **no Mach lockout**.
///
/// No lockout is the subsonic case: the drag check is never consulted, the
/// airframe cannot affect the run, and the vertical filter is born right
/// after the thrust-vector alignment finishes. That is Void Lake, and it is
/// also what the synthetic gate tests in [`crate::flight_estimators`] want
/// — a config that does nothing interesting while the pad gate is under
/// test.
///
/// `max_open_mach` is 0.8 because that is the Mach [`lc25_rocket`]'s stowed
/// `cd[0]` is tabulated at; the two have to agree, and pairing them here is
/// the reason they cannot drift apart across the four call sites.
///
/// The supersonic replay overrides `mach_lockout` at its own call site —
/// the two timers in it are precisely what that test measures, so they do
/// not get to live in here.
pub fn lc25_airbrakes() -> AirbrakesConfig {
    AirbrakesConfig {
        ignition_detection_acc_threshold: 4.0 * 9.81,
        mach_lockout: None,
        max_open_mach: 0.8,
        rocket: lc25_rocket(),
    }
}
