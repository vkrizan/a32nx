#[derive(Debug, PartialEq, Clone, Copy)]
pub enum Mode {
    Speed,
    ThrustClimb,
    ThrustDescent,
    AlphaFloor,
    TOGALock,
}

#[derive(Debug)]
pub struct AutoThrottleInput {
    pub mode: Mode,
    pub throttles: [f64; 2],
    pub airspeed: f64,
    pub airspeed_target: f64,
    pub vls: f64,
    pub alpha_floor: bool,
    pub radio_height: f64,
    pub pushbutton: bool,
    pub instinctive_disconnect: bool,
}

impl AutoThrottleInput {
    fn target_airspeed(&self) -> f64 {
        self.airspeed_target.max(self.vls)
    }
}

#[derive(Debug)]
pub struct AutoThrottleOutput {
    pub mode: Mode,
    pub armed: bool,
    pub active: bool,
    pub commanded: [f64; 2],
}

pub struct Gates {}
impl Gates {
    pub const GATE_SIZE: f64 = 1.0;

    pub const TOGA: f64 = 100.0;
    pub const FLEX_MCT: f64 = 95.0;
    pub const CL: f64 = 90.0;
    pub const IDLE: f64 = 0.0;
    pub const REV_IDLE: f64 = -1.0;
}

#[derive(Debug, PartialEq)]
enum Instinctive {
    Released,
    Pushed(std::time::Instant),
    Lockout,
}

#[derive(Debug)]
pub struct AutoThrottle {
    speed_mode_pid: crate::pid::PID,
    thrust_rate_limiter: crate::rl::RateLimiter,
    last_t: std::time::Instant,
    instinctive: Instinctive,
    toga_lock_throttles: [f64; 2],
    commanded: f64,
    input: AutoThrottleInput,
    output: AutoThrottleOutput,
}

impl AutoThrottle {
    pub fn new() -> Self {
        AutoThrottle {
            speed_mode_pid: crate::pid::PID::new(10.0, 1.0, 0.3, 10.0, 0.0, 100.0),
            thrust_rate_limiter: crate::rl::RateLimiter::new(),
            last_t: std::time::Instant::now(),
            instinctive: Instinctive::Released,
            toga_lock_throttles: [0.0, 0.0],
            commanded: 0.0,
            input: AutoThrottleInput {
                mode: Mode::Speed,
                throttles: [0.0, 0.0],
                airspeed: 0.0,
                airspeed_target: 0.0,
                vls: 0.0,
                alpha_floor: false,
                radio_height: 0.0,
                pushbutton: false,
                instinctive_disconnect: false,
            },
            output: AutoThrottleOutput {
                mode: Mode::Speed,
                armed: false,
                active: false,
                commanded: [0.0, 0.0],
            },
        }
    }

    pub fn update(&mut self) {
        self.engage_logic();

        if self.output.active {
            self.active_logic();
        } else {
            self.commanded = 100.0;
        }

        let m = |i: usize| match self.output.mode {
            Mode::AlphaFloor | Mode::TOGALock => self.commanded,
            _ => self.input.throttles[i].min(self.commanded),
        };
        self.output.commanded = [m(0), m(1)];
    }

    // FIGURE 22-31-00-13000-A SHEET 1
    // A/THR Engage Logic
    fn engage_logic(&mut self) {
        // - two ADIRS must be valid
        // - one LGCIU must be healthy
        // - guidance portion healthy
        // - management portion healthy
        let ap_fd_athr_common_cond = true;

        // - the two ECUs/EECs must be healthy
        // - the FCU must be healthy
        // - no discrepancy between the N1/EPR target computed in the FMGC and the N1/EPR feedback from each ECU/EEC
        // - VLS, VMAX, etc. must be healthy
        // - instinctive disconnect not held for 15s
        let athr_specific_cond = match self.instinctive {
            Instinctive::Released => {
                if self.input.instinctive_disconnect {
                    self.instinctive = Instinctive::Pushed(std::time::Instant::now());
                }
                true
            }
            Instinctive::Pushed(t) => {
                if self.input.instinctive_disconnect {
                    if t.elapsed().as_secs_f64() >= 15.0 {
                        self.instinctive = Instinctive::Lockout;
                    }
                } else {
                    self.instinctive = Instinctive::Released;
                }
                true
            }
            Instinctive::Lockout => false,
        };

        let one_engine_cond = false;

        let athr_common_or_specific = ap_fd_athr_common_cond || athr_specific_cond;
        let s = athr_common_or_specific
            && (
                // Action on A/THR pushbutton switch
                self.input.pushbutton
                // TOGA condition
                || self.input.throttles.iter().all(|t| *t == Gates::FLEX_MCT || *t == Gates::TOGA)
                || self.input.alpha_floor
            );

        let r = !athr_common_or_specific
            // if the A/THR function on the opposite FMGC is disengaged and on condition that this FMGC has priority.
            || false
            // Action on the A/THR pushbutton switch, with the A/THR function already armed.
            || (self.input.pushbutton && self.output.armed)
            // Action on one of the A/THR instinctive disconnect pushbuton switches.
            || self.input.instinctive_disconnect
            // ECU/EEC autothrust control feedback i.e. the A/THR being active at level of the
            // FMGCs, one of the two ECUs/EECs indicates that it is not in autothrust control mode.
            || false
            // AP/FD loss condition i.e. total loss of AP/FD below 100ft with the RETARD mode not engaged.
            || false
            // One engine start on the ground
            || false
            // Go around condition i.e. one throttle control lever is placed in the non active
            // area (> MCT) below 100ft without engagement of the GO AROUND mode on the AP/FD.
            || (false && (self.input.radio_height < 100.0 && self.input.throttles.iter().any(|t| *t > Gates::FLEX_MCT)))
            // Both throttle control levers placed in the IDLE position.
            // Both throttle control levers placed in the REVERSE position.
            || self.input.throttles.iter().all(|t| *t <= Gates::IDLE);

        // SR flip-flop
        self.output.armed = if s {
            !r
        } else if r {
            false
        } else {
            self.output.armed
        };

        // After engagement, A/THR is active if:
        self.output.active = self.output.armed
            && (
                // the Alpha floor protection is active whatever the position of the throttle control levers.
                self.input.alpha_floor
                // one throttle control lever is between IDLE and CL (including CL), and the other
                // is between IDLE and MCT (including MCT) with FLEX TO limit mode not selected.
                || (one_engine_cond && false)
                // The two throttle control levers are between IDLE and CL (CL included).
                || self.input.throttles.iter().all(|t| *t > Gates::IDLE && *t <= Gates::CL)
            );

        if self.output.active {
            self.output.mode = if self.input.alpha_floor {
                Mode::AlphaFloor
            } else {
                match self.output.mode {
                    Mode::AlphaFloor => {
                        self.toga_lock_throttles = self.input.throttles;
                        Mode::TOGALock
                    }
                    Mode::TOGALock => {
                        if self.toga_lock_throttles == self.input.throttles {
                            Mode::TOGALock
                        } else {
                            self.input.mode
                        }
                    }
                    _ => self.input.mode,
                }
            };

            if self.input.mode != self.output.mode && self.output.mode == Mode::Speed {
                self.speed_mode_pid.reset(
                    self.input.target_airspeed(),
                    self.input.airspeed,
                    self.last_t.elapsed().as_secs_f64(),
                    self.commanded,
                );
                self.thrust_rate_limiter.reset(self.commanded);
            }
        }
    }

    fn active_logic(&mut self) {
        let dt = self.last_t.elapsed().as_secs_f64();
        self.last_t = std::time::Instant::now();

        self.commanded = match self.output.mode {
            Mode::Speed => self.thrust_rate_limiter.iterate(
                self.speed_mode_pid
                    .update(self.input.target_airspeed(), self.input.airspeed, dt),
                10.0,
                10.0,
                dt,
            ),
            Mode::ThrustClimb | Mode::ThrustDescent => self.thrust_rate_limiter.iterate(
                if self.output.mode == Mode::ThrustClimb {
                    80.0
                } else {
                    0.0
                },
                4.0,
                4.0,
                dt,
            ),
            Mode::AlphaFloor | Mode::TOGALock => 100.0,
        }
    }

    pub fn input(&mut self) -> &mut AutoThrottleInput {
        &mut self.input
    }

    pub fn output(&self) -> &AutoThrottleOutput {
        &self.output
    }
}
