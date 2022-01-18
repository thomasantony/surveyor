/// Surveyor spacecraft implementation using orbiter-rs
/// 
/// This is a port of Surveyor.cpp to Rust
/// 
use orbiter_rs::{
    debug_string, oapi_create_vessel, OrbiterVessel, init_vessel, KeyStates, Key, FileHandle, ReferenceFrame,
    PropellantHandle, ThrusterHandle, Vector3, VesselContext, VesselStatus, ThrusterGroupType, V,
};
use lazy_static::lazy_static;

const VERNIER_PROP_MASS: f64 = 70.98;
const VERNIER_ISP: f64 = 3200.0;
const VERNIER_THRUST: f64 = 463.0;
const VERNIER_RAD: f64 = 0.5;
const VERNIER_Z: f64 = -0.65;

const RCS_PROP_MASS: f64 = 2.0;
const RCS_ISP: f64 = 630.0;
const RCS_THRUST: f64 = 0.25;
const RCS_RAD: f64 = 1.0;
const RCS_Z: f64 = -0.5;
const RCS_SPACE: f64 = 0.1;

const RETRO_PROP_MASS: f64 = 560.64;
const RETRO_THRUST: f64 = 39140.0;
const RETRO_BURNTIME: f64 = 40.5;
const RETRO_ITOT: f64 = RETRO_THRUST * RETRO_BURNTIME;
const RETRO_ISP: f64 = RETRO_ITOT / RETRO_PROP_MASS;
const RETRO_Z: f64 = -0.75;

const LANDER_EMPTY_MASS: f64 = 289.10; //Basic bus plus payload minus AMR minus retro case
const RETRO_EMPTY_MASS: f64 = 64.88;
const AMR_MASS: f64 = 3.82;

const LEG_RAD: f64 = 1.5;
const LEG_Z: f64 = -0.6;

// Controller gains
const GRAVITY_TURN_POINTING_GAIN: f64 = 0.3;
const PITCH_RATE_GAIN: f64 = 4.0;
const YAW_RATE_GAIN: f64 = 2.0;
const ROLL_RATE_GAIN: f64 = 10.0;
const THRUST_CONTROL_GAIN: f64 = 0.01;
const CONST_VEL_THRUST_GAIN: f64 = 0.1;
const LUNAR_GRAVITY: f64 = 1.625; // m/s^2

const FT_IN_M: f64 = 0.3048;
const MI_IN_M: f64 = 1609.34;

lazy_static! {
    static ref SURVEYOR_PMI: Vector3 = V!(0.50, 0.50, 0.50);
    static ref THRUSTER1_POS: Vector3 = V!(0.0 * VERNIER_RAD, 1.0 * VERNIER_RAD, VERNIER_Z);
    static ref THRUSTER2_POS: Vector3 = V!(
        (60.0f64).to_radians().sin() * VERNIER_RAD,
        -0.5 * VERNIER_RAD,
        VERNIER_Z
    );
    static ref THRUSTER3_POS: Vector3 = V!(
        -(60.0f64).to_radians().sin() * VERNIER_RAD,
        -0.5 * VERNIER_RAD,
        VERNIER_Z
    );

    static ref DIR_X_PLUS: Vector3 = V!(1., 0., 0.);
    static ref DIR_X_MINUS: Vector3 = V!(-1., 0., 0.);
    static ref DIR_Y_PLUS: Vector3 = V!(0., 1., 0.);
    static ref DIR_Y_MINUS: Vector3 = V!(0., -1., 0.);
    static ref DIR_Z_PLUS: Vector3 = V!(0., 0., 1.);
    static ref DIR_Z_MINUS: Vector3 = V!(0., 0., 1.);
}

#[derive(Debug, PartialEq)]
enum SurveyorState {
    BeforeRetroIgnition,
    RetroFiring,
    AfterRetro,
    TerminalDescent,
}
impl Default for SurveyorState {
    fn default() -> Self {
        Self::BeforeRetroIgnition
    }
}

#[derive(Debug, PartialEq)]
enum AttitudeMode {
    Off,
    GravityTurn,
    InertialLock(Vector3)
}
impl Default for AttitudeMode {
    fn default() -> Self {
        Self::Off
    }
}
#[derive(Default, Debug)]
pub struct Surveyor {
    th_vernier: Vec<ThrusterHandle>,
    th_rcs: Vec<ThrusterHandle>,
    th_retro: ThrusterHandle,
    ph_vernier: PropellantHandle,
    ph_retro: PropellantHandle,
    ph_rcs: PropellantHandle,
    vehicle_state: SurveyorState,

    attitude_mode: AttitudeMode,
    pitchrate_target: f64,
    rollrate_target: f64,
    yawrate_target: f64,

    // Used for inertial attitude hold
    last_thrust_level: f64,
}
impl Surveyor {
    fn setup_meshes(&mut self, context: &VesselContext) {
        context.ClearMeshes();
        let mut meshes = Vec::new();
        meshes.push(("Surveyor-AMR", Vector3::new(0., 0., -0.6)));
        meshes.push(("Surveyor-Retro", Vector3::new(0., 0., -0.5)));
        meshes.push(("Surveyor-Lander", Vector3::new(0., 0.3, 0.)));

        let meshes_used = match self.vehicle_state {
            SurveyorState::BeforeRetroIgnition => &meshes[0..],
            SurveyorState::RetroFiring => &meshes[1..],
            SurveyorState::AfterRetro | SurveyorState::TerminalDescent => &meshes[2..],
        };
        for (mesh, ofs) in meshes_used {
            context.AddMeshWithOffset(mesh.to_string(), &ofs);
        }
    }
    fn setup_thrusters(&mut self, context: &VesselContext)
    {
        // Create Propellant Resources
        self.ph_vernier = context.CreatePropellantResource(VERNIER_PROP_MASS);
        self.ph_rcs = context.CreatePropellantResource(RCS_PROP_MASS);
        self.ph_retro = context.CreatePropellantResource(RETRO_PROP_MASS);

        self.th_vernier.push(context.CreateThruster(
            &THRUSTER1_POS,
            &DIR_Z_PLUS,
            VERNIER_THRUST,
            self.ph_vernier,
            VERNIER_ISP,
        ));
        self.th_vernier.push(context.CreateThruster(
            &THRUSTER2_POS,
            &DIR_Z_PLUS,
            VERNIER_THRUST,
            self.ph_vernier,
            VERNIER_ISP,
        ));
        self.th_vernier.push(context.CreateThruster(
            &THRUSTER3_POS,
            &DIR_Z_PLUS,
            VERNIER_THRUST,
            self.ph_vernier,
            VERNIER_ISP,
        ));
        context.CreateThrusterGroup(&self.th_vernier, ThrusterGroupType::Main);
        for th in self.th_vernier.iter() {
            context.AddExhaust(*th, 1.0, 0.1);
        }

        // Roll (Leg1) jets
        self.th_rcs.push(context.CreateThruster(
            &V!(-RCS_SPACE, RCS_RAD, RCS_Z),
            & DIR_X_PLUS,
            RCS_THRUST,
            self.ph_rcs,
            RCS_ISP,
        ));
        self.th_rcs.push(context.CreateThruster(
            &V!(RCS_SPACE, RCS_RAD, RCS_Z),
            & DIR_X_MINUS,
            RCS_THRUST,
            self.ph_rcs,
            RCS_ISP,
        ));

        // Leg2 jets
        self.th_rcs.push(context.CreateThruster(
            &V!(
                (60.0f64).to_radians().sin() * RCS_RAD,
                -0.5 * RCS_RAD,
                RCS_Z - RCS_SPACE
            ),
            &DIR_Z_PLUS,
            RCS_THRUST,
            self.ph_rcs,
            RCS_ISP,
        ));
        self.th_rcs.push(context.CreateThruster(
            &V!(
                (60.0f64).to_radians().sin() * RCS_RAD,
                -0.5 * RCS_RAD,
                RCS_Z + RCS_SPACE
            ),
            &DIR_Z_MINUS,
            RCS_THRUST,
            self.ph_rcs,
            RCS_ISP,
        ));

        // Leg3 jets
        self.th_rcs.push(context.CreateThruster(
            &V!(
                -(60.0f64).to_radians().sin() * RCS_RAD,
                -0.5 * RCS_RAD,
                RCS_Z - RCS_SPACE
            ),
            &DIR_Z_PLUS,
            RCS_THRUST,
            self.ph_rcs,
            RCS_ISP,
        ));
        self.th_rcs.push(context.CreateThruster(
            &V!(
                -(60.0f64).to_radians().sin() * RCS_RAD,
                -0.5 * RCS_RAD,
                RCS_Z + RCS_SPACE
            ),
            &DIR_Z_MINUS,
            RCS_THRUST,
            self.ph_rcs,
            RCS_ISP,
        ));

        // // Create RCS thruster groups
        // let mut th_group = [ThrusterHandle::default(), ThrusterHandle::default()];

        // th_group[0] = self.th_rcs[3]; // -Z #1
        // th_group[1] = self.th_rcs[5]; // -Z #2
        // context.CreateThrusterGroup(&th_group, ThrusterGroupType::AttPitchdown);

        // th_group[0] = self.th_rcs[2]; // +Z #1
        // th_group[1] = self.th_rcs[4]; // +Z #2
        // context.CreateThrusterGroup(&th_group, ThrusterGroupType::AttPitchup);

        // th_group[0] = self.th_rcs[0]; // +X
        // context.CreateThrusterGroup(&th_group[..1], ThrusterGroupType::AttBankright);

        // th_group[0] = self.th_rcs[1]; // -X
        // context.CreateThrusterGroup(&th_group, ThrusterGroupType::AttBankleft);

        // th_group[0] = self.th_rcs[3]; // -Z #1
        // th_group[1] = self.th_rcs[4]; // +Z #2
        // context.CreateThrusterGroup(&th_group, ThrusterGroupType::AttYawright);

        // th_group[0] = self.th_rcs[2]; // +Z #1
        // th_group[1] = self.th_rcs[5]; // -Z #2
        // context.CreateThrusterGroup(&th_group, ThrusterGroupType::AttYawleft);

        for th in self.th_rcs.iter() {
            context.AddExhaust(*th, 0.1, 0.05);
        }

        self.th_retro = context.CreateThruster(
            &V!(0.0, 0.0, RETRO_Z),
            & DIR_Z_PLUS,
            RETRO_THRUST,
            self.ph_retro,
            RETRO_ISP,
        );
        context.AddExhaust(self.th_retro, 2.0, 0.3);
    }
    fn calc_empty_mass(&self, context: &VesselContext) -> f64 {
        let mut empty_mass = 0.0;
        // Jettison AMR when retro starts firing
        if context.GetPropellantMass(self.ph_retro) > 0.999 * RETRO_PROP_MASS {
            empty_mass += AMR_MASS;
        }
        // Add in retro mass while there is still retro fuel left
        if context.GetPropellantMass(self.ph_retro) > 1. {
            empty_mass += RETRO_EMPTY_MASS;
        }
        empty_mass += LANDER_EMPTY_MASS;
        return empty_mass;
    }
    fn calc_vehicle_mass(&self, context: &VesselContext) -> f64 {
        let mass = self.calc_empty_mass(context);
        return mass + context.GetTotalPropellantMass();
    }
    fn spawn_object(&self, context: &VesselContext, classname: &str, ext: &str, offset: &Vector3) {
        let mut vs = VesselStatus::default();

        context.GetStatus(&mut vs);
        context.Local2Rel(offset, &mut vs.rpos);

        vs.eng_main = 0.0;
        vs.eng_hovr = 0.0;
        vs.status = 0;
        let new_object_name = format!("{}{}", context.GetName(), ext);

        oapi_create_vessel(new_object_name, classname.to_owned(), &vs);
    }
    fn jettison(&mut self, context: &VesselContext) {
        use SurveyorState::*;
        match self.vehicle_state {
            BeforeRetroIgnition => {
                self.vehicle_state = RetroFiring;
                self.spawn_object(context, "Surveyor_AMR", "-AMR", &V!(0., 0., -0.6));
            }
            RetroFiring => {
                self.vehicle_state = AfterRetro;
                self.spawn_object(context, "Surveyor_Retro", "-Retro", &V!(0., 0., -0.5));
            }
            _ => {}
        }
        self.setup_meshes(context);
    }
    fn apply_thrusters(&mut self, context: &VesselContext, f_1: f64, f_2: f64, f_3: f64, theta_1: f64)
    {
        let f_1 = f_1.clamp(0.0, 1.0);
        let f_2 = f_2.clamp(0.0,1.0);
        let f_3 = f_3.clamp(0.0, 1.0);
        let theta_1 = theta_1.clamp(-5f64.to_radians(), 5f64.to_radians());
        context.SetThrusterLevel(self.th_vernier[0], f_1);
        context.SetThrusterLevel(self.th_vernier[1], f_2);
        context.SetThrusterLevel(self.th_vernier[2], f_3);

        let t1_sin = theta_1.sin();
        let t1_cos = theta_1.cos();

        let thruster1_dir = &V!(t1_sin, 0.0, t1_cos);
        context.SetThrusterDir(
            self.th_vernier[0],
            thruster1_dir,
        );
    }
    /// Converts angular acceleration vector into thrust components for vernier thrusters
    /// The `reference_thrust` value is used to compute the "known" value for thruster 1
    /// A minimum of 5% thrust is used if the reference is too low.
    #[allow(non_snake_case)]
    fn compute_thrust_from_ang_acc(&mut self, vehicle_mass:f64, reference_thrust_level: f64, angular_acc: &Vector3, min_thrust_level: f64) -> (f64 ,f64, f64, f64)
    {
        if angular_acc.length() < 0.001
        {
            return (reference_thrust_level, reference_thrust_level, reference_thrust_level, 0.);
        }
        // Fix thruster 1 at 5%
        let F_1 = reference_thrust_level.max(min_thrust_level);

        // Moments of inertia
        let [PMI_x, PMI_y, PMI_z] = SURVEYOR_PMI.0;
        let (I_x, I_y, I_z) = (PMI_x * vehicle_mass, PMI_y * vehicle_mass, PMI_z * vehicle_mass);

        // Thruster positions
        let [_, y1, z1] = THRUSTER1_POS.0;
        let [x2, y2, _] = THRUSTER2_POS.0;
        let [x3, y3, _] = THRUSTER3_POS.0;
        
        // Find limiting roll acceleration
        let SIN_5_DEG:f64 = 5f64.to_radians().sin();
        let max_roll_acc = SIN_5_DEG * y1 / I_z;
        
        // Account for Orbiter's left-handed coordinate system 
        let a_roll = angular_acc.z().clamp(-max_roll_acc, max_roll_acc);
        let a_pitch = -angular_acc.x();
        let a_yaw = angular_acc.y();

        let M_x = I_x * a_pitch;
        let M_y = I_y * a_yaw;
        let M_z = I_z * a_roll;

        let sin_theta_1 = M_z / (F_1 * y1);
        // Limit the value so that we can do a proper arcsin
        let sin_theta_1 = sin_theta_1.clamp(-SIN_5_DEG,SIN_5_DEG);

        let theta_1 = sin_theta_1.asin();
        let (sin_th1, cos_th1) = (theta_1.sin(), theta_1.cos());

        let M1x = F_1 * cos_th1 * y1;
        let M1y = F_1 * sin_th1 * z1;
        
        let F_3 = ((M_x - M1x)/y2 - (M_y - M1y)/x2)/(y3/y2 - x3/x2);
        let F_2 = (M_x - M1x - F_3*y3)/y2;
        
        (F_1, reference_thrust_level+F_2, reference_thrust_level+F_3, theta_1)
    }
    /// Get the desired velocity to be on the descent contour
    fn get_descent_contour_velocity(&mut self, altitude: f64) -> f64
    {
        let (v0, dhdv, h_offset) = if altitude > 12500.0 * FT_IN_M
        {
            (400.0 * FT_IN_M, 90., 12500. * FT_IN_M)
        }else if altitude > 5000.0 * FT_IN_M {
            (100.0 * FT_IN_M, 31.67, 3000. * FT_IN_M)
        }else{
            (20.0 * FT_IN_M, 31.15, 60. * FT_IN_M)
        };
        // Make sure the velocity is negative to match sign convention
        - (v0 + (altitude - h_offset)/dhdv)
    }
    /// Computes requires thrust level to maintain a constant acceleration
    fn const_acc_controller(&mut self, context: &VesselContext, target_acc: f64) -> f64
    {
        let current_acc = self.get_vehicle_acceleration(context);

        let thrust_change = THRUST_CONTROL_GAIN*(target_acc - current_acc);
        thrust_change
    }
    /// Computes requires thrust level to maintain constant velocity
    fn const_velocity_controller(&mut self, context: &VesselContext, target_vel: f64) -> f64
    {
        let current_vel = self.get_surface_approach_vel(context);
        let thrust_change = CONST_VEL_THRUST_GAIN*(target_vel - current_vel);
        thrust_change
    }

    /// Computes angular velocity to be commanded to achieve desired orientation
    fn attitude_controller(&mut self, context: &VesselContext) -> Vector3
    {
        match &self.attitude_mode {
            AttitudeMode::Off => { Vector3::default() },
            AttitudeMode::GravityTurn => {
                let target_orientation = self.compute_target_vector_for_gravity_turn(context);
                let (rotation_axis, rotation_angle) = self.compute_rotation(context, &target_orientation);
                self.pointing_controller(rotation_axis, rotation_angle)
            },
            AttitudeMode::InertialLock(target_orientation_global) => {
                let mut target_orientation = Vector3::default();
                context.Global2Local(&target_orientation_global, &mut target_orientation);
                let (rotation_axis, rotation_angle) = self.compute_rotation(context, &target_orientation);
                self.pointing_controller(rotation_axis, rotation_angle)
            }
        }
    }
    /// Uses a proportional gain to convert a given rotation (in angle+axis form) to
    /// an angular velocity vector
    fn pointing_controller(&mut self, rotation_axis: Vector3, rotation_angle: f64) -> Vector3 
    {
        let ang_vel_magnitude = -rotation_angle * GRAVITY_TURN_POINTING_GAIN;
        // Scale the rotation axis vector by the magnitude to get the angular velocity vector
        let target_ang_vel = rotation_axis * ang_vel_magnitude;

        target_ang_vel
    }
    /// Computes the target vector for gravity turn
    fn compute_target_vector_for_gravity_turn(&mut self, context: &VesselContext) -> Vector3
    {
        let mut airspeed = Vector3::default();
        // Compute velocity vector in vehicle's local frame of reference
        context.GetAirspeedVector(ReferenceFrame::Local, &mut airspeed);
        // We want to rotate the roll axis to point to the opposite of the airspeed vector
        -airspeed.unit()
    }
    /// Computes the rotation in angle/axis form to point the roll axis towards the
    /// retrograde direction
    fn compute_rotation(&mut self, context: &VesselContext, target_orientation: &Vector3) -> (Vector3, f64)
    {
        // Roll axis is the direction of thrust for vernier thrusters
        let roll_axis = V!(0., 0., 1.);

        // We need to use a controller to drive `rotation_angle` to zero
        let rotation_angle = roll_axis.dot(target_orientation).acos();

        // Avoid divide-by-zero
        if  rotation_angle.is_nan() || rotation_angle.abs() < 1e-3
        {
            (V!(0., 1., 0.), 0.)
        }else{
            // We compute the require rotation in angle-axis form
            // Compute axis perpendicular to initial and final orientation of roll-axis
            let rotation_axis = roll_axis.cross(target_orientation).unit();
            (rotation_axis, rotation_angle)
        }
    }
    /// Computes the angular acceleration required to achieve targeted angular velocity
    /// Uses a proportional controller
    fn angular_rate_controller(&mut self, context: &VesselContext, target_ang_vel: &Vector3) -> Vector3
    {
        let mut current_angular_vel = Vector3::default();
        context.GetAngularVel(&mut current_angular_vel);
        let angular_vel_err = target_ang_vel.clone() - current_angular_vel;
        let pitch_rate_err = angular_vel_err.x();
        let yaw_rate_err = angular_vel_err.y();
        let roll_rate_err = angular_vel_err.z();
        V!(
            pitch_rate_err * PITCH_RATE_GAIN,
            yaw_rate_err * YAW_RATE_GAIN,
            roll_rate_err * ROLL_RATE_GAIN
        )
    }
    /// Proxy for an accelerometer
    /// Returns vehicle acceleration in m/s^2
    fn get_vehicle_acceleration(&mut self, context: &VesselContext) -> f64 {
        let mut thrust_vec = Vector3::default();
        context.GetThrustVector(&mut thrust_vec);
        thrust_vec.length() / self.calc_vehicle_mass(context)
    }

    fn get_altitude(&mut self, context: &VesselContext) -> f64 {
        context.GetAltitude() - context.GetSurfaceElevation()
    }
    /// Used in terminal phase after ~10000 ft or ~3km
    fn get_surface_approach_vel(&mut self, context: &VesselContext) -> f64 {
        let href = context.GetSurfaceRef();

        let mut rel_vel = Vector3::default();
        context.GetAirspeedVector(ReferenceFrame::Horizon, &mut rel_vel);

        rel_vel.y() // Vertical speed
    }
}
impl OrbiterVessel for Surveyor {
    fn set_class_caps(&mut self, context: &VesselContext, _cfg: FileHandle) {
        context.SetSize(1.0);
        context.SetPMI(&SURVEYOR_PMI);
        context.SetTouchdownPoints(
            &V!(0.0, LEG_RAD, LEG_Z),
            &V!(
                (60.0f64).to_radians().sin() * LEG_RAD,
                -0.5 * LEG_RAD,
                LEG_Z
            ),
            &V!(
                -(60.0f64).to_radians().sin() * LEG_RAD,
                -0.5 * LEG_RAD,
                LEG_Z
            ),
        );
        self.setup_thrusters(context);
        context.SetEmptyMass(LANDER_EMPTY_MASS);

        // camera parameters
        context.SetCameraOffset(&V!(0.0, 0.8, 0.0));
        self.setup_meshes(context)
    }
    #[allow(non_snake_case)]
    fn on_pre_step(&mut self, context: &VesselContext, _sim_t: f64, _sim_dt: f64, _mjd: f64) {
        context.SetEmptyMass(self.calc_empty_mass(context));
        
        let altitude = self.get_altitude(context);

        // Compute vehicle state
        if self.vehicle_state == SurveyorState::BeforeRetroIgnition
        {
            // let mut target_orientation_global = Vector3::default();
            // context.Local2Global(&V!(0., 0., 1.), &mut target_orientation_global);
            // self.attitude_mode = AttitudeMode::InertialLock(target_orientation_global);
            self.attitude_mode = AttitudeMode::GravityTurn;
            if altitude < 46.0 * MI_IN_M
            {
                debug_string!("Firing retro. altitude = {:.2} mi", altitude / MI_IN_M);
                // Store the current z-axis orientation in global coordinates
                context.SetThrusterLevel(self.th_retro, 1.0);
                // vehicle_state will be set after AMR is jettisoned 
            }else {
                debug_string!("Waiting to fire retro ... altitude = {:.2} mi", altitude / MI_IN_M);
            }
        }else if self.vehicle_state == SurveyorState::AfterRetro {
            if altitude < 20.0 * FT_IN_M
            {
                self.attitude_mode = AttitudeMode::Off;
            }
            else if altitude < 1000. * FT_IN_M
            {
                // Store the current z-axis orientation in global coordinates
                let mut target_orientation_global = Vector3::default();
                context.Local2Global(&V!(0., 0., 1.), &mut target_orientation_global);
                self.attitude_mode = AttitudeMode::InertialLock(target_orientation_global);
            }
        }

        let altitude = self.get_altitude(context);
        // debug_string!("Altitude: {:.2} m", altitude);
        // Get current main thruster level
        let reference_thrust  = if self.vehicle_state == SurveyorState::AfterRetro && altitude > 14.0 * FT_IN_M {
            let delta_thrust = if altitude >= 40000. * FT_IN_M
            // let delta_thrust = if altitude >= 109500.
            {
                debug_string!("Constant Acceleration Mode, Altitude: {:.2} ft", altitude/FT_IN_M);
                self.const_acc_controller(context, 0.9 * LUNAR_GRAVITY)
            }else if altitude > 55. * FT_IN_M {
                // Approximate dsescent contour
                let target_vel = self.get_descent_contour_velocity(altitude);
                debug_string!("Descent Contour, Altitude: {:.2} ft, Target vel: {:.2} ft/s, Current vel: {:.2} ft/s", altitude/FT_IN_M, target_vel/FT_IN_M, self.get_surface_approach_vel(context)/FT_IN_M);
                if altitude < 60. * FT_IN_M 
                {
                    self.vehicle_state = SurveyorState::TerminalDescent;
                }
                self.const_velocity_controller(context, target_vel)
            }else {
                0.
            };
            
            // Clamp to 0.95 to have some control margin
            self.last_thrust_level = (self.last_thrust_level + delta_thrust).clamp(0.0, 0.95);
            self.last_thrust_level
        }else if self.vehicle_state == SurveyorState::TerminalDescent
        {
            let delta_thrust = if altitude >= 14. * FT_IN_M
            {
                // Constant velocity
                let delta_th = self.const_velocity_controller(context, -1.5);
                debug_string!("Terminal Descent, Altitude: {:.2} ft, Current vel: {:.2} ft/s, del_th: {:.2}", altitude/FT_IN_M, self.get_surface_approach_vel(context)/FT_IN_M, delta_th);
                delta_th
            }else{
                debug_string!("Freefall");
                0.
            };
            let ref_acc = VERNIER_THRUST * 3./ self.calc_vehicle_mass(context);
            let ref_thrust_level = ref_acc / LUNAR_GRAVITY;

            self.last_thrust_level = (self.last_thrust_level + delta_thrust).clamp(0.0, 0.95);
            self.last_thrust_level
            // // Clamp to 0.95 to have some control margin
            // (ref_thrust_level + delta_thrust).clamp(0.0, 0.95)
        }else {
            // debug_string!("Thrusters off");
            0.0
        };
       
        // Point spacecraft for gravity turn (or inertial hold)
        let angular_rate_target  = self.attitude_controller(context);
        let angular_acc = self.angular_rate_controller(context, &angular_rate_target);
        
        let vehicle_mass = self.calc_vehicle_mass(context);

        let min_thrust_level = if self.vehicle_state == SurveyorState::TerminalDescent {
            0.01
        }else{
            0.05
        };
        let (F_1, F_2, F_3, th1) = self.compute_thrust_from_ang_acc(vehicle_mass, reference_thrust, & angular_acc, min_thrust_level);
        if altitude > 4.0 && !context.GroundContact()
        {
            self.apply_thrusters(context, F_1, F_2, F_3, th1);
        }else {
            self.apply_thrusters(context, 0., 0., 0., 0.);
        }
        
        if self.vehicle_state == SurveyorState::RetroFiring
            && context.GetPropellantMass(self.ph_retro) < 1.0
        {
            //Jettison the spent main retro
            self.jettison(context);
        }
        if self.vehicle_state == SurveyorState::BeforeRetroIgnition
            && context.GetPropellantMass(self.ph_retro) < 0.999 * RETRO_PROP_MASS
        {
            //Jettison the AMR if the retro has started burning
            self.jettison(context);
            //Relight the retro if needed
            context.SetThrusterLevel(self.th_retro, 1.0);
        }

    }
    fn consume_buffered_key(
        &mut self,
        context: &VesselContext,
        key: Key,
        down: bool,
        kstate: KeyStates,
    ) -> i32 {
        if !down {
            0
        } else if kstate.shift()
        {
            0
        } else {
            // unmodified keys
            if key == Key::L
            {
                // Fire Retro
                context.SetThrusterLevel(self.th_retro, 1.0);
                1
            }else if key == Key::W {
                self.pitchrate_target -= 5f64.to_radians();
                1
            }else if key == Key::S {
                self.pitchrate_target += 5f64.to_radians();
                1
            }else if key == Key::A {
                self.rollrate_target -= 5f64.to_radians();
                1
            }else if key == Key::D {
                self.rollrate_target += 5f64.to_radians();
                1
            }else if key == Key::Q {
                self.yawrate_target += 5f64.to_radians();
                1
            }else if key == Key::E {
                self.yawrate_target -= 5f64.to_radians();
                1
            }else if key == Key::G {
                self.attitude_mode = AttitudeMode::GravityTurn;
                1
            }else if key == Key::Z {
                self.pitchrate_target = 0.;
                self.yawrate_target = 0.;
                self.rollrate_target = 0.;
                1
            }else{
                0
            }
        }
    }
}

init_vessel!(
    fn init(_h_vessel: OBJHANDLE, _flight_model: i32) -> Surveyor 
    {
        Surveyor::default()
    }
    fn exit() {}
);
