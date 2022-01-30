use orbiter_rs::Vector3;

/// Constants
pub const VERNIER_PROP_MASS: f64 = 70.98;
pub const VERNIER_ISP: f64 = 3200.0;
pub const VERNIER_THRUST: f64 = 463.0;
pub const VERNIER_RAD: f64 = 0.5;
pub const VERNIER_Z: f64 = -0.65;

pub const RCS_PROP_MASS: f64 = 2.0;
pub const RCS_ISP: f64 = 630.0;
pub const RCS_THRUST: f64 = 0.25;
pub const RCS_RAD: f64 = 1.0;
pub const RCS_Z: f64 = -0.5;
pub const RCS_SPACE: f64 = 0.1;

pub const RETRO_PROP_MASS: f64 = 560.64;
pub const RETRO_THRUST: f64 = 39140.0;
pub const RETRO_BURNTIME: f64 = 40.5;
pub const RETRO_ITOT: f64 = RETRO_THRUST * RETRO_BURNTIME;
pub const RETRO_ISP: f64 = RETRO_ITOT / RETRO_PROP_MASS;
pub const RETRO_Z: f64 = -0.75;

pub const LANDER_EMPTY_MASS: f64 = 289.10; //Basic bus plus payload minus AMR minus retro case
pub const RETRO_EMPTY_MASS: f64 = 64.88;
pub const AMR_MASS: f64 = 3.82;

pub const LEG_RAD: f64 = 1.5;
pub const LEG_Z: f64 = -0.6;

// Controller gains
pub const GRAVITY_TURN_POINTING_GAIN: f64 = 0.3;
pub const PITCH_RATE_GAIN: f64 = 4.0;
pub const YAW_RATE_GAIN: f64 = 2.0;
pub const ROLL_RATE_GAIN: f64 = 10.0;
pub const THRUST_CONTROL_GAIN: f64 = 0.01;
pub const LUNAR_GRAVITY: f64 = 1.625; // m/s^2

// Unit conversions
pub const FT_IN_M: f64 = 0.3048;
pub const MI_IN_M: f64 = 1609.34;

// Significant altitudes
pub const RETRO_IGNITION_ALTITUDE: f64 = 48.0 * MI_IN_M;
pub const DESCENT_CONTOUR_ALTITUDE: f64 = 40000.0 * FT_IN_M;
pub const TERMINAL_DESCENT_ALTITUDE: f64 = 60. * FT_IN_M;
pub const ENGINE_CUTOFF_ALTITUDE: f64 = 14.0 * FT_IN_M;

const SIN_60_DEG: f64 = 0.86602540378;

// Vector constants
/// Mass-Normalized Principal Moments of Inertia
pub const SURVEYOR_PMI: Vector3 = Vector3::new(0.50, 0.50, 0.50);

/// Position of Thruster 1
pub const THRUSTER1_POS: Vector3 = Vector3::new(0.0 * VERNIER_RAD, 1.0 * VERNIER_RAD, VERNIER_Z);
/// Position of Thruster 2
pub const THRUSTER2_POS: Vector3 =
    Vector3::new(SIN_60_DEG * VERNIER_RAD, -0.5 * VERNIER_RAD, VERNIER_Z);
/// Position of Thruster 3
pub const THRUSTER3_POS: Vector3 =
    Vector3::new(-SIN_60_DEG * VERNIER_RAD, -0.5 * VERNIER_RAD, VERNIER_Z);

// Cardinal direction vectors
pub const DIR_X_PLUS: Vector3 = Vector3::new(1., 0., 0.);
pub const DIR_X_MINUS: Vector3 = Vector3::new(-1., 0., 0.);
pub const DIR_Y_PLUS: Vector3 = Vector3::new(0., 1., 0.);
// pub const DIR_Y_MINUS: Vector3 = Vector3::new(0., -1., 0.);
pub const DIR_Z_PLUS: Vector3 = Vector3::new(0., 0., 1.);
pub const DIR_Z_MINUS: Vector3 = Vector3::new(0., 0., 1.);
