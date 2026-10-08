//! Per-motor overload and fault evaluation: pure state machine, no I/O.
//!
//! The caller steps one [`MotorHealthFilter`] per motor every control tick,
//! either with a decoded sample or with [`MotorHealthFilter::silent`] when
//! the motor has stopped answering, and publishes the returned report.
//!
//! Vendor-neutral on purpose: a driver maps its own wire status onto
//! [`MotorCondition`] and names its faults as text, so this policy is shared
//! by every node that judges a motor rather than only the ones holding a CAN
//! socket.

use crate::filters::Ewma;

/// Time constant of the sustained-torque average, sized against the two
/// spec load cases rather than a thermal model: long enough that the 3 s
/// peak-spec transient cannot reach the critical threshold, short enough
/// that a sustained overload warns within about 6 s.
const EWMA_TAU_S: f64 = 5.0;

/// Longest interval one sample may claim toward the average ([`Ewma`]'s
/// cap): one second rides out a scheduler stall without weighting the
/// resuming sample as a second of held load.
const MAX_STEP_S: f64 = 1.0;

/// Sustained |torque|/rated latch thresholds, judged against the EWMA `y`.
///
/// "Sustained above X" means exactly: the exponentially weighted average of
/// |torque|/rated, with time constant [`EWMA_TAU_S`], exceeds X. For a
/// constant load of F x rated stepping on at t = 0,
/// `y(t) = F * (1 - e^(-t / tau))`, so the warn engages at
/// `t = -tau * ln(1 - WARN_ON / F)`; a load at or below WARN_ON never
/// engages it.
///
/// The average is of the fraction itself, not its square: an i2t-style
/// thermal model would average F^2 (heating goes with current squared),
/// which warns sooner on loads that alternate. Loads here are quasi-static
/// holds, this is an operator signal rather than a thermal model, and the
/// winding temperature channel measures actual heat directly.
///
/// The average drives the warning level only, never critical: the payload
/// spec's own 4.1 kg / 1 min hold sits near 1.8x continuous on the loaded
/// joints, so a sustained threshold at any red-worthy value would fire
/// during manufacturer-blessed holds. Critical is reserved for the
/// channels that measure real danger directly: the instantaneous peak,
/// the temperatures, and the fault frames.
const TORQUE_WARN_ON: f64 = 0.90;
const TORQUE_WARN_OFF: f64 = 0.75;

/// Release band for the instantaneous peak channel, as a fraction of the
/// peak rating. The peak check is a bare threshold on a quantized reading,
/// so without its own release a torque dithering across the threshold
/// retriggers every tick: measured at 1 kHz with 2 mNm of dither across a
/// 40 Nm peak, that is 500 level transitions per second. Engaging at
/// `peak` and releasing only below `peak * PEAK_RELEASE` gives the channel
/// the same hysteresis every other channel has.
const PEAK_RELEASE: f64 = 0.90;

/// Temperature latch thresholds, degrees C, each clearing
/// `TEMP_HYSTERESIS_C` below its engage point. Critical sits deliberately
/// below the motor's own protections so the operator hears about it before
/// the joint goes limp. The winding pair is public so a node that can read
/// its motor's configured over-temperature trip can verify at bring-up that
/// these thresholds actually precede it.
const TEMP_DRIVER_WARN_C: f64 = 90.0;
const TEMP_DRIVER_CRIT_C: f64 = 105.0;
pub const TEMP_WINDING_WARN_C: f64 = 75.0;
pub const TEMP_WINDING_CRIT_C: f64 = 90.0;
const TEMP_HYSTERESIS_C: f64 = 5.0;

const _: () = {
    assert!(TORQUE_WARN_ON > TORQUE_WARN_OFF);
    assert!(PEAK_RELEASE > 0.0 && PEAK_RELEASE < 1.0);
    assert!(TEMP_DRIVER_CRIT_C - TEMP_HYSTERESIS_C > TEMP_DRIVER_WARN_C);
    assert!(TEMP_WINDING_CRIT_C - TEMP_HYSTERESIS_C > TEMP_WINDING_WARN_C);
    assert!(EWMA_TAU_S > 0.0);
    assert!(MAX_STEP_S > 0.0);
};

/// What a motor is doing, as far as its driver can tell. Drivers map their
/// own wire encoding onto this; the fault text is the vendor's name for the
/// protection that tripped, carried through to the operator verbatim.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MotorCondition {
    /// Powered and acting on commands.
    Driving,
    /// Powered but not acting on commands, so the joint is limp.
    Idle,
    /// A protection tripped and the motor stopped acting on commands.
    Faulted(&'static str),
    /// The driver decoded a status it does not have a meaning for. Treated
    /// as a fault rather than as health, so a firmware revision that defines
    /// new states cannot read as nominal.
    Unrecognised,
}

/// Severity of one motor's condition, worst-of across torque, temperature,
/// and condition checks. The discriminants are the motor_health wire
/// encoding.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
#[repr(u8)]
pub enum HealthLevel {
    Nominal = 0,
    Warning = 1,
    Critical = 2,
    Fault = 3,
    /// The motor has sent nothing recently, so nothing is known about it and
    /// its readings are last-known rather than current. Ranked above a
    /// fault: a motor that stopped talking may also have stopped acting, and
    /// unlike a fault it has not said so.
    NotReporting = 4,
}

impl HealthLevel {
    pub fn wire(self) -> u8 {
        self as u8
    }

    /// The level for a wire value, `None` for one outside the contract's
    /// scale. The single inverse of [`Self::wire`], so a consumer cannot
    /// drift its own decode table from the producer encoding.
    pub fn from_wire(wire: u8) -> Option<Self> {
        match wire {
            0 => Some(Self::Nominal),
            1 => Some(Self::Warning),
            2 => Some(Self::Critical),
            3 => Some(Self::Fault),
            4 => Some(Self::NotReporting),
            _ => None,
        }
    }
}

/// What drove a motor off nominal, so a report can name the measurement
/// behind its level rather than leaving the reader to guess.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum HealthCause {
    SustainedTorque,
    PeakTorque,
    DriverTemperature,
    WindingTemperature,
    /// The motor is powered but not acting on commands.
    NotDriving,
    /// A protection tripped; the text is the driver's name for it.
    Fault(&'static str),
    /// The motor has stopped answering.
    Silent,
}

/// Driver (MOS) temperature, degrees C.
#[derive(Debug, Clone, Copy, PartialEq, PartialOrd)]
pub struct DriverTempC(pub f64);

/// Motor winding temperature, degrees C. Distinct from [`DriverTempC`]
/// because the two carry different thresholds and are otherwise the same
/// type: swapping them at a call site would downgrade a cooking winding to a
/// warning, and no reader could see it.
#[derive(Debug, Clone, Copy, PartialEq, PartialOrd)]
pub struct WindingTempC(pub f64);

/// One control tick's decoded measurements for one motor.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct MotorSample {
    pub torque_nm: f64,
    pub driver_temp: DriverTempC,
    pub winding_temp: WindingTempC,
    pub condition: MotorCondition,
}

/// The filter's verdict for one motor after a tick.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct MotorHealth {
    pub level: HealthLevel,
    /// What drove the level off nominal; `None` while nominal.
    pub cause: Option<HealthCause>,
    /// Filtered |torque|/rated (the EWMA, not the instantaneous value).
    pub torque_fraction: f64,
    pub driver_temp: DriverTempC,
    pub winding_temp: WindingTempC,
}

/// A motor's torque limits. Constructed checked so a zero, non-finite, or
/// inverted rating cannot reach the filter, where it would poison the torque
/// fraction into NaN and silently disengage every latch.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Ratings {
    rated_nm: f64,
    peak_nm: f64,
}

impl Ratings {
    pub fn new(rated_nm: f64, peak_nm: f64) -> Result<Self, RatingsError> {
        if !(rated_nm.is_finite() && rated_nm > 0.0) {
            return Err(RatingsError::Rated(rated_nm));
        }
        if !(peak_nm.is_finite() && peak_nm > rated_nm) {
            return Err(RatingsError::Peak { peak_nm, rated_nm });
        }
        Ok(Self { rated_nm, peak_nm })
    }

    pub fn rated_nm(self) -> f64 {
        self.rated_nm
    }

    pub fn peak_nm(self) -> f64 {
        self.peak_nm
    }

    /// The same continuous rating with the peak pulled down to a measured
    /// trip point. Only ever lowers the peak, so a trip above the datasheet
    /// leaves the ratings alone.
    ///
    /// A non-finite trip is refused explicitly: `f64::min` would silently
    /// keep the datasheet for a NaN, turning garbage into a no-op. A trip at
    /// or below the continuous rating is refused because a threshold there
    /// would warn during legal rated operation instead of marking overload.
    pub fn tightened_to(self, trip_nm: f64) -> Result<Self, RatingsError> {
        if !trip_nm.is_finite() {
            return Err(RatingsError::Peak {
                peak_nm: trip_nm,
                rated_nm: self.rated_nm,
            });
        }
        Self::new(self.rated_nm, self.peak_nm.min(trip_nm))
    }
}

#[derive(Debug, Clone, Copy, PartialEq, thiserror::Error)]
pub enum RatingsError {
    #[error("continuous rating must be positive and finite, got {0}")]
    Rated(f64),
    #[error("peak/trip {peak_nm} must be finite and above the continuous rating {rated_nm}")]
    Peak { peak_nm: f64, rated_nm: f64 },
}

/// On/off latch with distinct engage and release thresholds.
#[derive(Debug, Clone, Copy, Default, PartialEq)]
struct Latch(bool);

impl Latch {
    fn updated(self, value: f64, on_at: f64, off_below: f64) -> Self {
        Self(if self.0 {
            value >= off_below
        } else {
            value >= on_at
        })
    }

    fn engaged(self) -> bool {
        self.0
    }
}

/// Overload and fault state machine for one motor.
///
/// The sustained channel starts all-clear: the EWMA seeds at zero, so no
/// averaged threshold can engage on the first tick. The instantaneous peak
/// channel and the condition checks are deliberately not averaged, so a
/// first sample already at peak, or already faulted, reports immediately.
///
/// Clone is for deliberate snapshots of the latch state.
#[derive(Debug, Clone)]
pub struct MotorHealthFilter {
    ratings: Ratings,
    fraction_ewma: Ewma,
    peak: Latch,
    torque_warn: Latch,
    driver_warn: Latch,
    driver_crit: Latch,
    winding_warn: Latch,
    winding_crit: Latch,
    fault: Option<&'static str>,
    last: Option<MotorHealth>,
}

impl MotorHealthFilter {
    pub fn new(ratings: Ratings) -> Self {
        Self {
            ratings,
            fraction_ewma: Ewma::new(EWMA_TAU_S, MAX_STEP_S)
                .expect("the tau and cap constants are positive"),
            peak: Latch::default(),
            torque_warn: Latch::default(),
            driver_warn: Latch::default(),
            driver_crit: Latch::default(),
            winding_warn: Latch::default(),
            winding_crit: Latch::default(),
            fault: None,
            last: None,
        }
    }

    /// Reports a motor that has stopped answering. The readings are the last
    /// ones actually measured, so a consumer showing them alongside the
    /// level sees the values the motor was last at rather than a fabricated
    /// zero. Before any sample has arrived there is nothing to carry, and
    /// the readings are reported absent.
    pub fn silent(&self) -> MotorHealth {
        match self.last {
            Some(last) => MotorHealth {
                level: HealthLevel::NotReporting,
                cause: Some(HealthCause::Silent),
                ..last
            },
            None => MotorHealth {
                level: HealthLevel::NotReporting,
                cause: Some(HealthCause::Silent),
                torque_fraction: 0.0,
                driver_temp: DriverTempC(f64::NAN),
                winding_temp: WindingTempC(f64::NAN),
            },
        }
    }

    /// Folds in one tick's sample and reports. `dt_s` is the measured
    /// interval since the previous sample, clamped to [`MAX_STEP_S`].
    ///
    /// The frame decode only produces finite torque and temperatures;
    /// asserted here anyway because a NaN would silently poison the EWMA or
    /// freeze a temperature latch (NaN comparisons never engage or release).
    pub fn step(&mut self, sample: MotorSample, dt_s: f64) -> MotorHealth {
        assert!(
            dt_s.is_finite() && dt_s >= 0.0,
            "dt_s must be finite and non-negative, got {dt_s}"
        );
        assert!(sample.torque_nm.is_finite(), "torque must be finite");
        assert!(
            sample.driver_temp.0.is_finite() && sample.winding_temp.0.is_finite(),
            "temperatures must be finite"
        );

        let fraction = sample.torque_nm.abs() / self.ratings.rated_nm;
        let sustained = self.fraction_ewma.step(fraction, dt_s);

        self.torque_warn = self
            .torque_warn
            .updated(sustained, TORQUE_WARN_ON, TORQUE_WARN_OFF);
        self.peak = self.peak.updated(
            sample.torque_nm.abs(),
            self.ratings.peak_nm,
            self.ratings.peak_nm * PEAK_RELEASE,
        );
        self.driver_warn = self.driver_warn.updated(
            sample.driver_temp.0,
            TEMP_DRIVER_WARN_C,
            TEMP_DRIVER_WARN_C - TEMP_HYSTERESIS_C,
        );
        self.driver_crit = self.driver_crit.updated(
            sample.driver_temp.0,
            TEMP_DRIVER_CRIT_C,
            TEMP_DRIVER_CRIT_C - TEMP_HYSTERESIS_C,
        );
        self.winding_warn = self.winding_warn.updated(
            sample.winding_temp.0,
            TEMP_WINDING_WARN_C,
            TEMP_WINDING_WARN_C - TEMP_HYSTERESIS_C,
        );
        self.winding_crit = self.winding_crit.updated(
            sample.winding_temp.0,
            TEMP_WINDING_CRIT_C,
            TEMP_WINDING_CRIT_C - TEMP_HYSTERESIS_C,
        );
        if let MotorCondition::Faulted(kind) = sample.condition {
            // Latched until cleared, like the motor's own fault state.
            self.fault.get_or_insert(kind);
        }

        let (level, cause) = self.verdict(sample.condition);
        let health = MotorHealth {
            level,
            cause,
            torque_fraction: sustained,
            driver_temp: sample.driver_temp,
            winding_temp: sample.winding_temp,
        };
        self.last = Some(health);
        health
    }

    /// This tick's severity and what drove it. Within a severity the causes
    /// are ordered torque before driver before winding, so the reported
    /// cause is stable while several conditions hold at once.
    fn verdict(&self, condition: MotorCondition) -> (HealthLevel, Option<HealthCause>) {
        if let Some(kind) = self.fault {
            return (HealthLevel::Fault, Some(HealthCause::Fault(kind)));
        }
        // A motor that is powered but not acting on commands is limp under
        // load. That is the condition this whole channel exists to surface,
        // so it outranks every measurement rather than reading as nominal.
        if condition == MotorCondition::Idle {
            return (HealthLevel::Fault, Some(HealthCause::NotDriving));
        }
        if condition == MotorCondition::Unrecognised {
            return (
                HealthLevel::Fault,
                Some(HealthCause::Fault("unrecognised state")),
            );
        }
        let critical = self
            .peak
            .engaged()
            .then_some(HealthCause::PeakTorque)
            .or_else(|| {
                self.driver_crit
                    .engaged()
                    .then_some(HealthCause::DriverTemperature)
            })
            .or_else(|| {
                self.winding_crit
                    .engaged()
                    .then_some(HealthCause::WindingTemperature)
            });
        if let Some(cause) = critical {
            return (HealthLevel::Critical, Some(cause));
        }
        let warning = self
            .torque_warn
            .engaged()
            .then_some(HealthCause::SustainedTorque)
            .or_else(|| {
                self.driver_warn
                    .engaged()
                    .then_some(HealthCause::DriverTemperature)
            })
            .or_else(|| {
                self.winding_warn
                    .engaged()
                    .then_some(HealthCause::WindingTemperature)
            });
        match warning {
            Some(cause) => (HealthLevel::Warning, Some(cause)),
            None => (HealthLevel::Nominal, None),
        }
    }
}

/// The alert kind every motor-condition alert carries.
///
/// One kind per motor on purpose. An alert is identified by (source, kind),
/// so a kind that tracked the cause would change identity when a motor moved
/// between conditions, leaving the previous kind raised with nothing left to
/// clear it. The motor's alert has to be one thing that gets upserted.
///
/// Named for what it covers rather than for one of its causes: overload,
/// overtemperature, communication loss and a limp motor all arrive here, and
/// filing a winding at 105 C under "overload" would misroute it.
pub const MOTOR_ALERT_KIND: &str = "motor_condition";

/// Health publish cadence shared by every producer, comfortably inside the
/// contract's 500 ms floor.
pub const HEALTH_PERIOD: std::time::Duration = std::time::Duration::from_millis(200);

/// How long a motor, or an engine standing in for one, may go unheard before
/// its last reading stops being presented as current and it is judged
/// silent. Shared so every producer names a quiet source on the same clock.
pub const STATE_STALE_AFTER: std::time::Duration = std::time::Duration::from_millis(500);

/// How long a motor may be not driving, with its loop still ticking, before
/// its follower stops the node instead of retrying: long enough to ride out
/// a transient power blip, short enough that a joint does not hang limp
/// under a held load.
pub const NOT_DRIVING_ESCALATE_AFTER: std::time::Duration = std::time::Duration::from_secs(1);

/// How often a producer re-publishes its unchanged alert set.
///
/// The set goes out whenever it changes, so this is the floor that bounds
/// three things a change alone cannot: the age of the measurement each
/// message carries, the recovery of a consumer that lost or refused a
/// message, and a consumer's judgement that a producer has gone quiet.
/// Comfortably inside the alert contract's 2000 ms ceiling, which is what
/// consumers age a producer's set out against.
pub const ALERT_FLOOR_PERIOD: std::time::Duration = std::time::Duration::from_millis(1600);

const _: () = {
    // The contract mandates a report at least every 500 ms; consumers age
    // reports out on multiples of that cadence, on their own clocks.
    assert!(HEALTH_PERIOD.as_millis() <= 500);
    // The alert set re-publishes inside the contract's 2000 ms ceiling.
    assert!(ALERT_FLOOR_PERIOD.as_millis() < 2000);
};

/// The severity scale of the alert contract: 1 warning, 2 critical, 3 fault.
///
/// A listed alert is active, so the scale has no value for a healthy motor
/// and `of_level` answers `None` for one. The wire value is produced and
/// parsed here alone, so a producer and a consumer cannot disagree on it.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub enum AlertSeverity {
    Warning,
    Critical,
    Fault,
}

impl AlertSeverity {
    pub fn wire(self) -> u8 {
        match self {
            Self::Warning => 1,
            Self::Critical => 2,
            Self::Fault => 3,
        }
    }

    /// The severity a wire value names, or `None` outside the scale.
    pub fn from_wire(value: u8) -> Option<Self> {
        match value {
            1 => Some(Self::Warning),
            2 => Some(Self::Critical),
            3 => Some(Self::Fault),
            _ => None,
        }
    }

    /// The severity of a health level, or `None` for a nominal motor.
    ///
    /// Silence lands on the fault severity: a motor that has stopped
    /// answering is at least as serious as one that said it faulted, because
    /// it has not said anything.
    pub fn of_level(level: HealthLevel) -> Option<Self> {
        match level {
            HealthLevel::Nominal => None,
            HealthLevel::Warning => Some(Self::Warning),
            HealthLevel::Critical => Some(Self::Critical),
            HealthLevel::Fault | HealthLevel::NotReporting => Some(Self::Fault),
        }
    }
}

/// The operator-facing one-liner for a condition, naming the measurement
/// that drove it.
///
/// The cause comes from the report's own verdict, so the two always agree
/// and the measurement named is the one the level was judged on. Each
/// message carries the reading of the round it went out in, and the set
/// re-publishes every [`ALERT_FLOOR_PERIOD`], so the number an operator
/// reads is at most that old.
fn describe(cause: HealthCause, report: &MotorHealth) -> String {
    match cause {
        HealthCause::SustainedTorque => format!(
            "holding {:.0}% of rated torque",
            report.torque_fraction * 100.0
        ),
        HealthCause::PeakTorque => "torque hit the motor's peak".to_string(),
        HealthCause::DriverTemperature => {
            format!("driver at {:.0} C", report.driver_temp.0)
        }
        HealthCause::WindingTemperature => {
            format!("motor winding at {:.0} C", report.winding_temp.0)
        }
        HealthCause::NotDriving => {
            "powered but not acting on commands: the joint is limp".to_string()
        }
        HealthCause::Silent => {
            "stopped reporting: its condition is unknown and it may be limp".to_string()
        }
        HealthCause::Fault(kind) => {
            format!("{kind}: the motor cut out and the joint is limp")
        }
    }
}

/// One alert a producer holds active.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Alert {
    pub source: String,
    pub severity: AlertSeverity,
    pub message: String,
}

/// One motor's condition, as the raiser compares it between rounds.
type Condition = Option<(AlertSeverity, HealthCause)>;

/// The set of active alerts a round of reports owes the wire, with the
/// per-motor conditions to commit once it is out.
///
/// The conditions and the alerts come from one pass, and only the raiser
/// that produced the set reads the conditions, so a published set and the
/// state recorded for it cannot drift apart.
#[derive(Debug, PartialEq, Eq)]
pub struct AlertSet<const N: usize> {
    conditions: [Condition; N],
    alerts: Vec<Alert>,
}

impl<const N: usize> AlertSet<N> {
    /// Every alert the producer holds active, to publish as one message.
    pub fn alerts(&self) -> &[Alert] {
        &self.alerts
    }
}

/// What the last published set recorded.
struct Sent<const N: usize> {
    conditions: [Condition; N],
    at: std::time::Instant,
}

/// Plans the alert set a producer publishes from successive health reports:
/// every motor with a condition, published whenever any motor's
/// (severity, cause) changes and again every [`ALERT_FLOOR_PERIOD`], so a
/// message that omits a motor clears it.
///
/// The first round always owes a set, empty while every motor is nominal, so
/// the topic holds a message for a consumer that subscribes later and that
/// consumer can tell a healthy producer from one that has not started.
///
/// Pure planning, two-phase: [`AlertRaiser::due`] proposes the set and the
/// caller commits it with [`AlertRaiser::mark_sent`] only after its publish
/// succeeds, so a failed send is retried next round.
///
/// A motor holds at most one alert, identified by `(source, kind)` where
/// source is that motor's label. Changes are keyed on (severity, cause), so
/// a measurement moving inside a band does not re-trigger the topic at the
/// sample rate; the floor is what refreshes the number it carries.
///
/// `N` is the motor count, so a report slice of the wrong length and a set
/// from another raiser are both compile errors.
pub struct AlertRaiser<const N: usize> {
    /// One operator-facing label per motor, the alert's `source`: an arm
    /// passes "left arm j1".."left arm j7", a gripper its single name.
    sources: [String; N],
    /// The last published set, `None` until one is sent.
    sent: Option<Sent<N>>,
}

impl<const N: usize> AlertRaiser<N> {
    pub fn new(sources: [String; N]) -> Self {
        const { assert!(N > 0, "a raiser without motors raises nothing") };
        Self {
            sources,
            sent: None,
        }
    }

    /// The set these reports owe at `now`: `Some` when any motor's
    /// (severity, cause) differs from the last published set, when the floor
    /// has elapsed since it went out, or when nothing has been published yet.
    /// A motor counts as conditioned only when its level is non-nominal and
    /// it carries a cause, so a severity the contract has no value for
    /// cannot reach the wire.
    pub fn due(&self, reports: &[MotorHealth; N], now: std::time::Instant) -> Option<AlertSet<N>> {
        let conditions: [Condition; N] = std::array::from_fn(|motor| {
            let report = &reports[motor];
            AlertSeverity::of_level(report.level).zip(report.cause)
        });
        if let Some(sent) = &self.sent {
            let floor_elapsed = now.duration_since(sent.at) >= ALERT_FLOOR_PERIOD;
            if sent.conditions == conditions && !floor_elapsed {
                return None;
            }
        }
        let alerts = conditions
            .iter()
            .enumerate()
            .filter_map(|(motor, condition)| {
                condition.map(|(severity, cause)| Alert {
                    source: self.sources[motor].clone(),
                    severity,
                    message: describe(cause, &reports[motor]),
                })
            })
            .collect();
        Some(AlertSet { conditions, alerts })
    }

    /// Records a published set, sent at `now`.
    pub fn mark_sent(&mut self, set: &AlertSet<N>, now: std::time::Instant) {
        self.sent = Some(Sent {
            conditions: set.conditions,
            at: now,
        });
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const DT: f64 = 0.01;
    const RATED: f64 = 20.0;
    const PEAK: f64 = 40.0;

    fn ratings() -> Ratings {
        Ratings::new(RATED, PEAK).expect("valid ratings")
    }

    fn filter() -> MotorHealthFilter {
        MotorHealthFilter::new(ratings())
    }

    fn sample(torque_nm: f64) -> MotorSample {
        MotorSample {
            torque_nm,
            driver_temp: DriverTempC(25.0),
            winding_temp: WindingTempC(25.0),
            condition: MotorCondition::Driving,
        }
    }

    fn at(torque_nm: f64, driver_c: f64, winding_c: f64) -> MotorSample {
        MotorSample {
            driver_temp: DriverTempC(driver_c),
            winding_temp: WindingTempC(winding_c),
            ..sample(torque_nm)
        }
    }

    fn run(f: &mut MotorHealthFilter, s: MotorSample, seconds: f64) -> MotorHealth {
        let ticks = (seconds / DT).round() as usize;
        (0..ticks)
            .map(|_| f.step(s, DT))
            .last()
            .expect("at least one tick")
    }

    #[test]
    fn the_sustained_average_reaches_63_percent_of_a_step_in_one_time_constant() {
        let mut f = filter();
        let report = run(&mut f, sample(RATED), EWMA_TAU_S);
        assert!((report.torque_fraction - (1.0 - (-1.0f64).exp())).abs() < 1e-3);
    }

    #[test]
    fn one_long_gap_cannot_rewrite_the_whole_history() {
        // Six seconds of genuine overload, then a single sample arriving
        // after a long stall. Without the clamp that one sample is weighted
        // as though its value had held for the entire gap, erasing the
        // overload; with it the average ages toward the new value instead.
        let mut f = filter();
        let overloaded = run(&mut f, sample(1.3 * RATED), 6.0);
        assert_eq!(overloaded.level, HealthLevel::Warning);
        let after_gap = f.step(sample(0.2 * RATED), 20.0);
        assert!(
            after_gap.torque_fraction > 0.5,
            "one sample erased the history: {}",
            after_gap.torque_fraction
        );
    }

    #[test]
    fn cold_start_cannot_false_warn_on_an_averaged_threshold() {
        let mut f = filter();
        assert_eq!(f.step(sample(RATED), DT).level, HealthLevel::Nominal);
    }

    #[test]
    fn a_sustained_overload_warns_and_never_escalates_on_the_average() {
        // The payload spec's own 1-minute hold runs near 1.8x continuous,
        // so the average may only warn: red is reserved for the peak, the
        // temperatures, and the faults.
        let mut f = filter();
        assert_eq!(
            run(&mut f, sample(1.3 * RATED), 5.0).level,
            HealthLevel::Nominal
        );
        assert_eq!(
            run(&mut f, sample(1.3 * RATED), 2.0).level,
            HealthLevel::Warning
        );
        let held = run(&mut f, sample(1.3 * RATED), 60.0);
        assert_eq!(held.level, HealthLevel::Warning);
        assert_eq!(held.cause, Some(HealthCause::SustainedTorque));
        assert!(held.torque_fraction > 1.0);
    }

    #[test]
    fn peak_spec_transient_does_not_warn() {
        let mut f = filter();
        let report = run(&mut f, sample(1.7 * RATED), 3.0);
        assert_eq!(report.level, HealthLevel::Nominal);
        assert!(report.torque_fraction < TORQUE_WARN_ON);
    }

    #[test]
    fn warn_clears_only_below_the_release_threshold() {
        let mut f = filter();
        // 6.5 s at 1.3x rated lands y near 0.946: past the 0.90 engage,
        // short of the 1.0 critical crossing at 7.33 s.
        assert_eq!(
            run(&mut f, sample(1.3 * RATED), 6.5).level,
            HealthLevel::Warning
        );
        assert_eq!(
            run(&mut f, sample(0.8 * RATED), 2.0).level,
            HealthLevel::Warning
        );
        assert_eq!(run(&mut f, sample(0.0), 30.0).level, HealthLevel::Nominal);
    }

    #[test]
    fn instantaneous_peak_torque_is_immediately_critical() {
        let mut f = filter();
        assert_eq!(f.step(sample(PEAK), DT).level, HealthLevel::Critical);
        assert_eq!(f.step(sample(-PEAK), DT).level, HealthLevel::Critical);
        assert_eq!(f.step(sample(0.0), DT).level, HealthLevel::Nominal);
    }

    #[test]
    fn torque_dithering_across_the_peak_does_not_flap_the_level() {
        // A bare threshold on a quantized reading retriggers every tick when
        // the load sits on it; the release band is what stops the operator
        // seeing hundreds of transitions per second.
        let mut f = filter();
        let levels: Vec<HealthLevel> = (0..8)
            .map(|i| {
                let torque = if i % 2 == 0 {
                    PEAK + 0.002
                } else {
                    PEAK - 0.002
                };
                f.step(sample(torque), DT).level
            })
            .collect();
        assert!(
            levels.iter().all(|l| *l == HealthLevel::Critical),
            "level flapped across the peak threshold: {levels:?}"
        );
    }

    #[test]
    fn a_peak_during_a_sustained_warning_is_still_named_as_the_peak() {
        // The sustained average sits in the warning band; the only
        // critical-grade evidence is the instantaneous peak, so the cause
        // must not claim a sustained overload that never crossed critical.
        let mut f = filter();
        let warned = run(&mut f, sample(1.3 * RATED), 6.5);
        assert_eq!(warned.level, HealthLevel::Warning);
        let spiked = f.step(sample(PEAK), DT);
        assert_eq!(spiked.level, HealthLevel::Critical);
        assert_eq!(spiked.cause, Some(HealthCause::PeakTorque));
    }

    #[test]
    fn a_released_peak_does_not_leave_a_warning_band_average_critical() {
        // After the peak releases, the level is whatever the sustained
        // average earns on its own: here the warning band, named as the
        // sustained cause.
        let mut f = filter();
        let warned = run(&mut f, sample(1.3 * RATED), 6.5);
        assert_eq!(warned.level, HealthLevel::Warning);
        assert_eq!(f.step(sample(PEAK), DT).level, HealthLevel::Critical);
        let released = f.step(sample(0.5 * PEAK), DT);
        assert_eq!(released.level, HealthLevel::Warning);
        assert_eq!(released.cause, Some(HealthCause::SustainedTorque));
    }

    #[test]
    fn a_peak_from_cold_is_named_as_a_peak() {
        let mut f = filter();
        assert_eq!(
            f.step(sample(PEAK), DT).cause,
            Some(HealthCause::PeakTorque)
        );
    }

    #[test]
    fn a_motor_that_is_powered_but_not_driving_is_not_nominal() {
        // The joint is limp under load, which is the condition this channel
        // exists to surface; torque and temperature both read healthy.
        let mut f = filter();
        let report = f.step(
            MotorSample {
                condition: MotorCondition::Idle,
                ..sample(0.0)
            },
            DT,
        );
        assert_eq!(report.level, HealthLevel::Fault);
        assert_eq!(report.cause, Some(HealthCause::NotDriving));
    }

    #[test]
    fn an_unrecognised_state_fails_loud_rather_than_healthy() {
        let mut f = filter();
        let report = f.step(
            MotorSample {
                condition: MotorCondition::Unrecognised,
                ..sample(0.0)
            },
            DT,
        );
        assert_eq!(report.level, HealthLevel::Fault);
    }

    #[test]
    fn temperatures_latch_with_hysteresis() {
        let mut f = filter();
        assert_eq!(f.step(at(0.0, 86.0, 25.0), DT).level, HealthLevel::Nominal);
        assert_eq!(f.step(at(0.0, 90.0, 25.0), DT).level, HealthLevel::Warning);
        assert_eq!(f.step(at(0.0, 86.0, 25.0), DT).level, HealthLevel::Warning);
        assert_eq!(f.step(at(0.0, 84.0, 25.0), DT).level, HealthLevel::Nominal);
        assert_eq!(f.step(at(0.0, 25.0, 90.0), DT).level, HealthLevel::Critical);
        assert_eq!(f.step(at(0.0, 25.0, 86.0), DT).level, HealthLevel::Critical);
        assert_eq!(f.step(at(0.0, 25.0, 84.0), DT).level, HealthLevel::Warning);
    }

    #[test]
    fn a_fault_latches_and_outranks_every_measurement() {
        let mut f = filter();
        let faulted = MotorSample {
            condition: MotorCondition::Faulted("communication loss"),
            ..sample(0.0)
        };
        assert_eq!(f.step(faulted, DT).level, HealthLevel::Fault);
        let recovered = f.step(sample(0.0), DT);
        assert_eq!(recovered.level, HealthLevel::Fault);
        assert_eq!(
            recovered.cause,
            Some(HealthCause::Fault("communication loss"))
        );
    }

    #[test]
    fn a_silent_motor_reports_the_readings_it_was_last_at() {
        // Publishing a fabricated zero here is what makes a motor last seen
        // near its thermal limit render as cold.
        let mut f = filter();
        f.step(at(0.5 * RATED, 70.0, 96.0), DT);
        let silent = f.silent();
        assert_eq!(silent.level, HealthLevel::NotReporting);
        assert_eq!(silent.cause, Some(HealthCause::Silent));
        assert_eq!(silent.winding_temp, WindingTempC(96.0));
        assert_eq!(silent.driver_temp, DriverTempC(70.0));
    }

    #[test]
    fn a_motor_silent_before_its_first_frame_reports_no_readings() {
        let f = filter();
        let silent = f.silent();
        assert_eq!(silent.level, HealthLevel::NotReporting);
        assert!(silent.driver_temp.0.is_nan());
        assert!(silent.winding_temp.0.is_nan());
    }

    #[test]
    fn silence_outranks_every_condition_a_motor_can_report() {
        assert!(HealthLevel::NotReporting > HealthLevel::Fault);
        assert!(HealthLevel::Fault > HealthLevel::Critical);
        assert!(HealthLevel::Critical > HealthLevel::Warning);
        assert!(HealthLevel::Warning > HealthLevel::Nominal);
    }

    #[test]
    fn the_worst_severity_names_its_own_cause_not_an_earlier_ranked_one() {
        let mut f = filter();
        let report = run(&mut f, at(0.95 * RATED, 25.0, 95.0), 1.0);
        assert_eq!(report.level, HealthLevel::Critical);
        assert_eq!(report.cause, Some(HealthCause::WindingTemperature));
    }

    #[test]
    fn within_a_severity_the_cause_rank_is_stable() {
        let mut f = filter();
        let report = f.step(at(0.0, 91.0, 76.0), DT);
        assert_eq!(report.level, HealthLevel::Warning);
        assert_eq!(report.cause, Some(HealthCause::DriverTemperature));
    }

    #[test]
    fn ratings_reject_values_that_would_poison_the_fraction() {
        assert!(Ratings::new(0.0, 7.0).is_err());
        assert!(Ratings::new(f64::NAN, 7.0).is_err());
        assert!(Ratings::new(-1.0, 7.0).is_err());
        assert!(Ratings::new(3.0, 3.0).is_err());
        assert!(Ratings::new(3.0, f64::INFINITY).is_err());
        assert!(Ratings::new(3.0, 7.0).is_ok());
    }

    #[test]
    fn a_trip_at_or_below_the_continuous_rating_is_refused() {
        // Applying it would leave a critical threshold the motor can never
        // reach, so the operator would be warned never rather than early.
        let r = Ratings::new(3.0, 7.0).expect("valid");
        assert!(r.tightened_to(3.0).is_err());
        assert!(r.tightened_to(2.0).is_err());
        assert_eq!(r.tightened_to(5.0).expect("valid").peak_nm(), 5.0);
    }

    #[test]
    fn tightening_only_ever_lowers_the_peak() {
        let r = Ratings::new(3.0, 7.0).expect("valid");
        assert_eq!(r.tightened_to(9.0).expect("valid").peak_nm(), 7.0);
    }

    #[test]
    fn a_non_finite_trip_is_refused_not_silently_ignored() {
        // f64::min keeps the other argument for NaN, which would turn
        // garbage into a silent no-op on the crate's checked type.
        let r = Ratings::new(3.0, 7.0).expect("valid");
        assert!(r.tightened_to(f64::NAN).is_err());
        assert!(r.tightened_to(f64::INFINITY).is_err());
    }

    #[test]
    #[should_panic(expected = "torque must be finite")]
    fn non_finite_torque_is_rejected() {
        filter().step(sample(f64::NAN), DT);
    }

    #[test]
    #[should_panic(expected = "temperatures must be finite")]
    fn non_finite_temperature_is_rejected() {
        filter().step(at(0.0, f64::NAN, 25.0), DT);
    }

    #[test]
    #[should_panic(expected = "dt_s must be finite")]
    fn non_finite_dt_is_rejected() {
        filter().step(sample(0.0), f64::NAN);
    }

    #[test]
    fn every_level_round_trips_through_the_wire_and_junk_decodes_to_none() {
        for level in [
            HealthLevel::Nominal,
            HealthLevel::Warning,
            HealthLevel::Critical,
            HealthLevel::Fault,
            HealthLevel::NotReporting,
        ] {
            assert_eq!(HealthLevel::from_wire(level.wire()), Some(level));
        }
        for junk in [5, 6, 255] {
            assert_eq!(HealthLevel::from_wire(junk), None);
        }
    }
}

#[cfg(test)]
mod alert_tests {
    use super::*;

    const N: usize = 7;

    fn arm_sources() -> [String; N] {
        std::array::from_fn(|i| format!("left arm j{}", i + 1))
    }

    fn nominal() -> MotorHealth {
        MotorHealth {
            level: HealthLevel::Nominal,
            cause: None,
            torque_fraction: 0.1,
            driver_temp: DriverTempC(30.0),
            winding_temp: WindingTempC(28.0),
        }
    }

    fn silent() -> MotorHealth {
        MotorHealth {
            level: HealthLevel::NotReporting,
            cause: Some(HealthCause::Silent),
            ..nominal()
        }
    }

    fn warned(fraction: f64) -> MotorHealth {
        MotorHealth {
            level: HealthLevel::Warning,
            cause: Some(HealthCause::SustainedTorque),
            torque_fraction: fraction,
            ..nominal()
        }
    }

    fn all_nominal() -> [MotorHealth; N] {
        std::array::from_fn(|_| nominal())
    }

    fn reports(motor: usize, report: MotorHealth) -> [MotorHealth; N] {
        let mut all = all_nominal();
        all[motor] = report;
        all
    }

    /// A fixed instant to measure the floor against.
    fn t0() -> std::time::Instant {
        std::time::Instant::now()
    }

    /// due + mark sent at `now`, as the publisher does on success: the alerts
    /// of the set owed, or `None` when nothing is owed.
    fn step<const M: usize>(
        raiser: &mut AlertRaiser<M>,
        all: &[MotorHealth; M],
        now: std::time::Instant,
    ) -> Option<Vec<Alert>> {
        let set = raiser.due(all, now)?;
        raiser.mark_sent(&set, now);
        Some(set.alerts().to_vec())
    }

    /// A raiser that has published its opening set at `now`, which is where a
    /// running producer spends its life.
    fn started(now: std::time::Instant) -> AlertRaiser<N> {
        let mut raiser = AlertRaiser::new(arm_sources());
        let opening = step(&mut raiser, &all_nominal(), now).expect("the opening set is owed");
        assert!(opening.is_empty(), "a quiet arm opens with an empty set");
        raiser
    }

    #[test]
    fn the_opening_set_goes_out_once_even_when_every_motor_is_quiet() {
        // The topic retains one message, so a consumer that subscribes later
        // reads this one and can tell a quiet arm from an arm that never
        // started.
        let t0 = t0();
        let mut raiser = AlertRaiser::new(arm_sources());
        assert_eq!(
            step(&mut raiser, &all_nominal(), t0),
            Some(Vec::new()),
            "the first round owes an empty set"
        );
        assert!(step(&mut raiser, &all_nominal(), t0).is_none());
        assert!(
            step(&mut raiser, &all_nominal(), t0 + ALERT_FLOOR_PERIOD).is_some(),
            "the floor re-publishes the empty set"
        );
    }

    #[test]
    fn the_unchanged_set_goes_out_again_on_the_floor() {
        // The floor is what lets a consumer age a quiet producer out and what
        // recovers a consumer that lost or refused a message.
        let t0 = t0();
        let mut raiser = started(t0);
        let raised = step(&mut raiser, &reports(2, warned(0.93)), t0).expect("the raise is owed");
        assert_eq!(raised.len(), 1);

        let just_under = t0 + ALERT_FLOOR_PERIOD - std::time::Duration::from_millis(1);
        assert!(
            step(&mut raiser, &reports(2, warned(0.93)), just_under).is_none(),
            "nothing is owed before the floor elapses"
        );
        let again = step(
            &mut raiser,
            &reports(2, warned(0.93)),
            t0 + ALERT_FLOOR_PERIOD,
        )
        .expect("the floor is owed");
        assert_eq!(again, raised, "the same set goes out again");
    }

    #[test]
    fn the_floor_refreshes_the_measurement_the_message_carries() {
        // A number in the text is read by an operator long after the
        // condition began, so the floor bounds how stale it can be.
        let t0 = t0();
        let mut raiser = started(t0);
        let raised = step(&mut raiser, &reports(0, warned(0.91)), t0).expect("the raise is owed");
        assert_eq!(raised[0].message, "holding 91% of rated torque");

        assert!(
            step(&mut raiser, &reports(0, warned(0.99)), t0).is_none(),
            "a moving measurement is not a condition change"
        );
        let refreshed = step(
            &mut raiser,
            &reports(0, warned(0.99)),
            t0 + ALERT_FLOOR_PERIOD,
        )
        .expect("the floor is owed");
        assert_eq!(
            refreshed[0].message, "holding 99% of rated torque",
            "the floor carries the latest reading, not the raise-time one"
        );
    }

    #[test]
    fn a_warning_publishes_the_set_once_and_its_clear_once() {
        let t0 = t0();
        let mut raiser = started(t0);
        let raised = step(&mut raiser, &reports(1, warned(0.93)), t0).expect("the raise is owed");
        assert_eq!(raised.len(), 1);
        assert_eq!(raised[0].source, "left arm j2");
        assert_eq!(raised[0].severity, AlertSeverity::Warning);

        assert!(
            step(&mut raiser, &reports(1, warned(0.94)), t0).is_none(),
            "a condition that holds steady publishes once inside the floor"
        );

        let cleared = step(&mut raiser, &all_nominal(), t0).expect("the clear is owed");
        assert!(
            cleared.is_empty(),
            "a motor with no condition is not listed"
        );
        assert!(step(&mut raiser, &all_nominal(), t0).is_none());
    }

    #[test]
    fn escalation_replaces_the_motors_alert() {
        let t0 = t0();
        let mut raiser = started(t0);
        let warned_set =
            step(&mut raiser, &reports(3, warned(0.93)), t0).expect("the raise is owed");
        assert_eq!(warned_set.len(), 1);
        assert_eq!(warned_set[0].severity, AlertSeverity::Warning);
        assert_eq!(warned_set[0].message, "holding 93% of rated torque");

        let escalated = MotorHealth {
            level: HealthLevel::Critical,
            cause: Some(HealthCause::WindingTemperature),
            winding_temp: WindingTempC(92.0),
            ..nominal()
        };
        let raised = step(&mut raiser, &reports(3, escalated), t0).expect("the escalation is owed");
        assert_eq!(raised.len(), 1, "one entry per motor");
        assert_eq!(raised[0].source, "left arm j4");
        assert_eq!(raised[0].severity, AlertSeverity::Critical);
        assert_eq!(raised[0].message, "motor winding at 92 C");
    }

    #[test]
    fn two_motors_alert_independently() {
        let t0 = t0();
        let mut raiser = started(t0);
        let mut all = all_nominal();
        all[2] = warned(0.93);
        all[5] = warned(0.95);
        let raised = step(&mut raiser, &all, t0).expect("the raises are owed");
        assert_eq!(raised.len(), 2);
        assert_eq!(raised[0].source, "left arm j3");
        assert_eq!(raised[1].source, "left arm j6");

        all[5] = nominal();
        let kept = step(&mut raiser, &all, t0).expect("one clear is owed");
        assert_eq!(kept.len(), 1, "the other motor's alert stays listed");
        assert_eq!(kept[0].source, "left arm j3");
    }

    #[test]
    fn a_silent_motor_raises_its_own_alert_and_does_not_hide_the_others() {
        // A motor that stops answering is the failure this feature exists to
        // catch, and it must not suppress a second motor's fault either.
        let t0 = t0();
        let mut raiser = started(t0);
        let mut all = all_nominal();
        all[4] = silent();
        all[6] = MotorHealth {
            level: HealthLevel::Fault,
            cause: Some(HealthCause::Fault("overload")),
            ..nominal()
        };
        let raised = step(&mut raiser, &all, t0).expect("the raises are owed");
        assert_eq!(
            raised.len(),
            2,
            "the silent motor and the faulted one both alert"
        );
        assert_eq!(raised[0].source, "left arm j5");
        assert_eq!(raised[0].severity, AlertSeverity::Fault);
        assert_eq!(raised[1].source, "left arm j7");
    }

    #[test]
    fn a_motor_that_goes_silent_escalates_rather_than_letting_its_alert_lapse() {
        // Going quiet while warned must not read as a clear: the operator's
        // banner disappearing while the joint is still limp reads as the
        // problem having resolved itself.
        let t0 = t0();
        let mut raiser = started(t0);
        let warned_alert =
            step(&mut raiser, &reports(4, warned(0.93)), t0).expect("the raise is owed");
        assert_eq!(warned_alert[0].severity, AlertSeverity::Warning);

        let gone = step(&mut raiser, &reports(4, silent()), t0).expect("going silent is a change");
        assert_eq!(gone.len(), 1);
        assert_eq!(gone[0].source, "left arm j5");
        assert_eq!(
            gone[0].severity,
            AlertSeverity::Fault,
            "silence is at least as bad as a fault"
        );
        assert!(gone[0].message.contains("stopped reporting"));
    }

    #[test]
    fn the_severity_scale_starts_at_a_warning_and_round_trips() {
        // A listed alert is active, so a nominal motor has no severity at
        // all, and both consumers drop an entry carrying a 0.
        assert_eq!(AlertSeverity::of_level(HealthLevel::Nominal), None);
        assert_eq!(
            AlertSeverity::of_level(HealthLevel::Warning),
            Some(AlertSeverity::Warning)
        );
        assert_eq!(
            AlertSeverity::of_level(HealthLevel::Critical),
            Some(AlertSeverity::Critical)
        );
        assert_eq!(
            AlertSeverity::of_level(HealthLevel::Fault),
            Some(AlertSeverity::Fault)
        );
        assert_eq!(
            AlertSeverity::of_level(HealthLevel::NotReporting),
            Some(AlertSeverity::Fault),
            "silence is at least as bad as a fault"
        );

        for severity in [
            AlertSeverity::Warning,
            AlertSeverity::Critical,
            AlertSeverity::Fault,
        ] {
            assert_eq!(AlertSeverity::from_wire(severity.wire()), Some(severity));
        }
        assert_eq!(AlertSeverity::Warning.wire(), 1);
        assert_eq!(AlertSeverity::Critical.wire(), 2);
        assert_eq!(AlertSeverity::Fault.wire(), 3);
        for outside in [0, 4, 255] {
            assert_eq!(AlertSeverity::from_wire(outside), None, "{outside}");
        }
    }

    #[test]
    fn a_level_without_a_cause_is_not_a_condition() {
        // `verdict` pairs a nominal level with no cause, and the two halves
        // are read together, so a report assembled by hand cannot put a
        // severity the contract has no value for on the wire.
        let t0 = t0();
        let mut raiser = started(t0);
        let nominal_with_cause = MotorHealth {
            level: HealthLevel::Nominal,
            cause: Some(HealthCause::SustainedTorque),
            ..nominal()
        };
        assert!(
            step(&mut raiser, &reports(0, nominal_with_cause), t0).is_none(),
            "no condition, so nothing is owed"
        );
        let level_without_cause = MotorHealth {
            level: HealthLevel::Critical,
            cause: None,
            ..nominal()
        };
        assert!(
            step(&mut raiser, &reports(0, level_without_cause), t0).is_none(),
            "a level with no cause has nothing to describe"
        );
    }

    #[test]
    fn a_fault_names_the_kind_and_says_the_joint_is_limp() {
        let t0 = t0();
        let mut raiser = AlertRaiser::new(["right arm j7".to_string()]);
        step(&mut raiser, &[nominal()], t0).expect("the opening set is owed");
        let faulted = MotorHealth {
            level: HealthLevel::Fault,
            cause: Some(HealthCause::Fault("overload")),
            ..nominal()
        };
        let raised = step(&mut raiser, &[faulted], t0).expect("the fault is owed");
        assert_eq!(raised[0].severity, AlertSeverity::Fault);
        assert_eq!(
            raised[0].message,
            "overload: the motor cut out and the joint is limp"
        );
    }

    #[test]
    fn an_unsent_set_is_owed_until_it_sends() {
        // The publisher only marks what actually went out: a set whose
        // publish failed comes back next round, and so does the clear.
        let t0 = t0();
        let mut raiser = started(t0);
        let first = raiser
            .due(&reports(0, warned(0.93)), t0)
            .expect("the raise is owed");
        let retry = raiser
            .due(&reports(0, warned(0.93)), t0)
            .expect("the unsent raise comes back");
        assert_eq!(retry, first);
        raiser.mark_sent(&retry, t0);

        let clear = raiser.due(&all_nominal(), t0).expect("the clear is owed");
        assert!(clear.alerts().is_empty());
        assert_eq!(
            raiser
                .due(&all_nominal(), t0)
                .expect("the unsent clear comes back"),
            clear
        );
    }

    #[test]
    fn a_single_motor_component_alerts_under_its_own_name() {
        // A gripper is one motor whose label is the whole component name.
        let t0 = t0();
        let mut raiser = AlertRaiser::new(["left gripper".to_string()]);
        step(&mut raiser, &[nominal()], t0).expect("the opening set is owed");
        let raised = step(&mut raiser, &[warned(0.93)], t0).expect("the raise is owed");
        assert_eq!(raised[0].source, "left gripper");
    }
}
