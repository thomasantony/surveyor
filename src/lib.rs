/// Surveyor spacecraft implementation using orbiter-rs
/// 
/// This is a port of Surveyor.cpp to Rust
/// 
use orbiter_rs::{
    debug_string, oapi_create_vessel, OrbiterVessel, init_vessel, init_logging, KeyStates, Key, FileHandle, ReferenceFrame,
    PropellantHandle, ThrusterHandle, Vector3, SDKVessel, VesselStatus, ThrusterGroupType, V,
};

mod constants;
use constants::*;
use log::error;
use strum::{EnumString, IntoStaticStr};
use std::str::FromStr;

#[derive(Debug, PartialEq, Clone, EnumString, IntoStaticStr)]
enum DescentPhase {
    BeforeRetroIgnition,
    RetroFiring,
    AfterRetro,
    TerminalDescent,
}
impl Default for DescentPhase {
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
#[derive(Debug)]
pub struct Surveyor {
    th_vernier: Vec<ThrusterHandle>,
    th_rcs: Vec<ThrusterHandle>,
    th_retro: ThrusterHandle,
    ph_vernier: PropellantHandle,
    ph_retro: PropellantHandle,
    ph_rcs: PropellantHandle,
    descent_phase: DescentPhase,

    attitude_mode: AttitudeMode,
    pitchrate_target: f64,
    rollrate_target: f64,
    yawrate_target: f64,

    // Used for inertial attitude hold
    last_thrust_level: f64,

    /// Bridge to C++ Orbiter SDK
    ctx: SDKVessel,
}
impl Surveyor {
    pub fn new(vessel: SDKVessel) -> Self {
        init_logging(log::Level::Info);
        Self {
            th_vernier: Vec::new(),
            th_rcs: Vec::new(),
            th_retro: ThrusterHandle::default(),
            ph_vernier: PropellantHandle::default(),
            ph_retro: PropellantHandle::default(),
            ph_rcs: PropellantHandle::default(),
            descent_phase: DescentPhase::default(),

            attitude_mode: AttitudeMode::Off,
            pitchrate_target: 0.,
            rollrate_target: 0.,
            yawrate_target: 0.,

            last_thrust_level: 0.,
            ctx: vessel
        }
    }
    fn setup_meshes(&self) {
        self.ctx.ClearMeshes();
        let mut meshes = Vec::new();
        meshes.push(("Surveyor-AMR", Vector3::new(0., 0., -0.6)));
        meshes.push(("Surveyor-Retro", Vector3::new(0., 0., -0.5)));
        meshes.push(("Surveyor-Lander", Vector3::new(0., 0.3, 0.)));

        let meshes_used = match self.descent_phase {
            DescentPhase::BeforeRetroIgnition => &meshes[0..],
            DescentPhase::RetroFiring => &meshes[1..],
            DescentPhase::AfterRetro | DescentPhase::TerminalDescent => &meshes[2..],
        };
        for (mesh, ofs) in meshes_used {
            self.ctx.AddMeshWithOffset(mesh.to_string(), &ofs);
        }
    }
    fn setup_thrusters(&mut self)
    {
        // Create Propellant Resources
        self.ph_vernier = self.ctx.CreatePropellantResource(VERNIER_PROP_MASS);
        self.ph_rcs = self.ctx.CreatePropellantResource(RCS_PROP_MASS);
        self.ph_retro = self.ctx.CreatePropellantResource(RETRO_PROP_MASS);

        self.th_vernier.push(self.ctx.CreateThruster(
            &THRUSTER1_POS,
            &DIR_Z_PLUS,
            VERNIER_THRUST,
            self.ph_vernier,
            VERNIER_ISP,
        ));
        self.th_vernier.push(self.ctx.CreateThruster(
            &THRUSTER2_POS,
            &DIR_Z_PLUS,
            VERNIER_THRUST,
            self.ph_vernier,
            VERNIER_ISP,
        ));
        self.th_vernier.push(self.ctx.CreateThruster(
            &THRUSTER3_POS,
            &DIR_Z_PLUS,
            VERNIER_THRUST,
            self.ph_vernier,
            VERNIER_ISP,
        ));
        self.ctx.CreateThrusterGroup(&self.th_vernier, ThrusterGroupType::Main);
        for th in self.th_vernier.iter() {
            self.ctx.AddExhaust(*th, 1.0, 0.1);
        }

        // Roll (Leg1) jets
        self.th_rcs.push(self.ctx.CreateThruster(
            &V!(-RCS_SPACE, RCS_RAD, RCS_Z),
            & DIR_X_PLUS,
            RCS_THRUST,
            self.ph_rcs,
            RCS_ISP,
        ));
        self.th_rcs.push(self.ctx.CreateThruster(
            &V!(RCS_SPACE, RCS_RAD, RCS_Z),
            & DIR_X_MINUS,
            RCS_THRUST,
            self.ph_rcs,
            RCS_ISP,
        ));

        // Leg2 jets
        self.th_rcs.push(self.ctx.CreateThruster(
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
        self.th_rcs.push(self.ctx.CreateThruster(
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
        self.th_rcs.push(self.ctx.CreateThruster(
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
        self.th_rcs.push(self.ctx.CreateThruster(
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
        // self.ctx.CreateThrusterGroup(&th_group, ThrusterGroupType::AttPitchdown);

        // th_group[0] = self.th_rcs[2]; // +Z #1
        // th_group[1] = self.th_rcs[4]; // +Z #2
        // self.ctx.CreateThrusterGroup(&th_group, ThrusterGroupType::AttPitchup);

        // th_group[0] = self.th_rcs[0]; // +X
        // self.ctx.CreateThrusterGroup(&th_group[..1], ThrusterGroupType::AttBankright);

        // th_group[0] = self.th_rcs[1]; // -X
        // self.ctx.CreateThrusterGroup(&th_group, ThrusterGroupType::AttBankleft);

        // th_group[0] = self.th_rcs[3]; // -Z #1
        // th_group[1] = self.th_rcs[4]; // +Z #2
        // self.ctx.CreateThrusterGroup(&th_group, ThrusterGroupType::AttYawright);

        // th_group[0] = self.th_rcs[2]; // +Z #1
        // th_group[1] = self.th_rcs[5]; // -Z #2
        // self.ctx.CreateThrusterGroup(&th_group, ThrusterGroupType::AttYawleft);

        for th in self.th_rcs.iter() {
            self.ctx.AddExhaust(*th, 0.1, 0.05);
        }

        self.th_retro = self.ctx.CreateThruster(
            &V!(0.0, 0.0, RETRO_Z),
            & DIR_Z_PLUS,
            RETRO_THRUST,
            self.ph_retro,
            RETRO_ISP,
        );
        self.ctx.AddExhaust(self.th_retro, 2.0, 0.3);
    }
    fn calc_empty_mass(&self) -> f64 {
        let mut empty_mass = 0.0;
        // Jettison AMR when retro starts firing
        if self.ctx.GetPropellantMass(self.ph_retro) > 0.999 * RETRO_PROP_MASS {
            empty_mass += AMR_MASS;
        }
        // Add in retro mass while there is still retro fuel left
        if self.ctx.GetPropellantMass(self.ph_retro) > 1. {
            empty_mass += RETRO_EMPTY_MASS;
        }
        empty_mass += LANDER_EMPTY_MASS;
        return empty_mass;
    }
    fn calc_vehicle_mass(&self) -> f64 {
        let mass = self.calc_empty_mass();
        return mass + self.ctx.GetTotalPropellantMass();
    }
    fn spawn_object(&self, classname: &str, ext: &str, offset: &Vector3) {
        let mut vs = VesselStatus::default();

        self.ctx.GetStatus(&mut vs);
        self.ctx.Local2Rel(offset, &mut vs.rpos);

        vs.eng_main = 0.0;
        vs.eng_hovr = 0.0;
        vs.status = 0;
        let new_object_name = format!("{}{}", self.ctx.GetName(), ext);

        oapi_create_vessel(new_object_name, classname.to_owned(), &vs);
    }
    fn jettison(&mut self) {
        use DescentPhase::*;
        match self.descent_phase {
            BeforeRetroIgnition => {
                self.descent_phase = RetroFiring;
                self.spawn_object("Surveyor_AMR", "-AMR", &V!(0., 0., -0.6));
            }
            RetroFiring => {
                self.descent_phase = AfterRetro;
                self.spawn_object("Surveyor_Retro", "-Retro", &V!(0., 0., -0.5));
            }
            _ => {}
        }
        self.setup_meshes();
    }
    fn apply_thrusters(&self, f_1: f64, f_2: f64, f_3: f64, theta_1: f64)
    {
        let f_1 = f_1.clamp(0.0, 1.0);
        let f_2 = f_2.clamp(0.0,1.0);
        let f_3 = f_3.clamp(0.0, 1.0);
        let theta_1 = theta_1.clamp(-5f64.to_radians(), 5f64.to_radians());
        self.ctx.SetThrusterLevel(self.th_vernier[0], f_1);
        self.ctx.SetThrusterLevel(self.th_vernier[1], f_2);
        self.ctx.SetThrusterLevel(self.th_vernier[2], f_3);

        let t1_sin = theta_1.sin();
        let t1_cos = theta_1.cos();

        let thruster1_dir = &V!(t1_sin, 0.0, t1_cos);
        self.ctx.SetThrusterDir(
            self.th_vernier[0],
            thruster1_dir,
        );
    }
    /// Converts angular acceleration vector into thrust components for vernier thrusters
    /// The `reference_thrust` value is used to compute the "known" value for thruster 1
    /// A minimum of 5% thrust is used if the reference is too low.
    #[allow(non_snake_case)]
    fn compute_thrust_from_ang_acc(&self, vehicle_mass:f64, reference_thrust_level: f64, angular_acc: &Vector3, min_thrust_level: f64) -> (f64 ,f64, f64, f64)
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
    /// Get the desired velocity and acceleration to be on the descent contour
    fn get_descent_contour_velocity(&self, altitude: f64) -> (f64, f64)
    {
        let descent_contour = [
            (DESCENT_CONTOUR_ALTITUDE, 705. * FT_IN_M),
            (12500.0 * FT_IN_M, 400. * FT_IN_M),
            (5000.0 * FT_IN_M, 100. * FT_IN_M),
            (TERMINAL_DESCENT_ALTITUDE, 5. * FT_IN_M)
        ];

        let mut index = 0;
        for i in 0..descent_contour.len()
        {
            // Find the "band" of the descent contour that we are currently in
            if altitude > descent_contour[i].0
            {
                index = i;
                break;
            }
        }

        if index == 0
        {
            (-descent_contour[0].1, 0.)
        }else{
            let (h0, v0) = descent_contour[index-1];
            let (h1, v1) = descent_contour[index];

            let dhdv = (h0-h1)/(v0-v1);

            let nominal_acc = (v1*v1 - v0*v0)/(2.0*(h0-h1)) + LUNAR_GRAVITY;
            // Make sure the velocity is negative to match sign convention
            (-(v1 + (altitude - h1)/dhdv), nominal_acc)
        }
    }
    /// Computes requires thrust level to maintain a constant acceleration
    fn const_acc_controller(&self, target_acc: f64) -> f64
    {
        let current_acc = self.get_vehicle_acceleration();

        let thrust_change = THRUST_CONTROL_GAIN*(target_acc - current_acc);
        thrust_change
    }
    /// Computes requires thrust level to maintain constant velocity
    fn const_velocity_controller(&self, target_vel: f64, sim_dt: f64) -> f64
    {
        let current_vel = self.get_surface_approach_vel();
        let delta_acc = (target_vel - current_vel)/sim_dt;
        self.const_acc_controller(delta_acc + LUNAR_GRAVITY)
    }

    /// Computes angular velocity to be commanded to achieve desired orientation
    fn attitude_controller(&self) -> Vector3
    {
        match &self.attitude_mode {
            AttitudeMode::Off => { Vector3::default() },
            AttitudeMode::GravityTurn => {
                let target_orientation = self.compute_target_vector_for_gravity_turn();
                let (rotation_axis, rotation_angle) = self.compute_rotation(&target_orientation);
                self.pointing_controller(rotation_axis, rotation_angle)
            },
            AttitudeMode::InertialLock(target_orientation_global) => {
                let mut target_orientation = Vector3::default();
                self.ctx.Global2Local(&target_orientation_global, &mut target_orientation);
                let (rotation_axis, rotation_angle) = self.compute_rotation(&target_orientation);
                self.pointing_controller(rotation_axis, rotation_angle)
            }
        }
    }
    /// Uses a proportional gain to convert a given rotation (in angle+axis form) to
    /// an angular velocity vector
    fn pointing_controller(&self, rotation_axis: Vector3, rotation_angle: f64) -> Vector3 
    {
        let ang_vel_magnitude = -rotation_angle * GRAVITY_TURN_POINTING_GAIN;
        // Scale the rotation axis vector by the magnitude to get the angular velocity vector
        let target_ang_vel = rotation_axis * ang_vel_magnitude;

        target_ang_vel
    }
    /// Computes the target vector for gravity turn
    fn compute_target_vector_for_gravity_turn(&self) -> Vector3
    {
        let mut airspeed = Vector3::default();
        // Compute velocity vector in vehicle's local frame of reference
        self.ctx.GetAirspeedVector(ReferenceFrame::Local, &mut airspeed);
        // We want to rotate the roll axis to point to the opposite of the airspeed vector
        -airspeed.unit()
    }
    /// Computes the rotation in angle/axis form to point the roll axis towards the
    /// desired orientation
    fn compute_rotation(&self, target_orientation: &Vector3) -> (Vector3, f64)
    {
        // Roll axis is the direction of thrust for vernier thrusters
        let roll_axis = DIR_Z_PLUS;

        // We need to use a controller to drive `rotation_angle` to zero
        let rotation_angle = roll_axis.dot(target_orientation).acos();

        // Avoid divide-by-zero
        if  rotation_angle.is_nan() || rotation_angle.abs() < 1e-3
        {
            (DIR_Y_PLUS, 0.)
        }else{
            // We compute the require rotation in angle-axis form
            // Compute axis perpendicular to initial and final orientation of roll-axis
            let rotation_axis = roll_axis.cross(target_orientation).unit();
            (rotation_axis, rotation_angle)
        }
    }
    /// Computes the angular acceleration required to achieve targeted angular velocity
    /// Uses a proportional controller
    fn angular_rate_controller(&self, target_ang_vel: &Vector3) -> Vector3
    {
        let mut current_angular_vel = Vector3::default();
        self.ctx.GetAngularVel(&mut current_angular_vel);
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
    fn get_vehicle_acceleration(&self) -> f64 {
        let mut thrust_vec = Vector3::default();
        self.ctx.GetThrustVector(&mut thrust_vec);
        thrust_vec.length() / self.calc_vehicle_mass()
    }

    fn get_altitude(&self) -> f64 {
        self.ctx.GetAltitude() - self.ctx.GetSurfaceElevation()
    }
    /// Used in terminal phase after ~10000 ft or ~3km
    fn get_surface_approach_vel(&self) -> f64 {
        let mut rel_vel = Vector3::default();
        self.ctx.GetAirspeedVector(ReferenceFrame::Horizon, &mut rel_vel);

        rel_vel.y() // Vertical speed
    }
}
impl OrbiterVessel for Surveyor {
    fn set_class_caps(&mut self, _cfg: &FileHandle) {
        self.ctx.SetSize(1.0);
        self.ctx.SetPMI(&SURVEYOR_PMI);
        self.ctx.SetTouchdownPoints(
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
        self.setup_thrusters();
        self.ctx.SetEmptyMass(LANDER_EMPTY_MASS);

        // camera parameters
        self.ctx.SetCameraOffset(&V!(0.0, 0.8, 0.0));
        self.setup_meshes()
    }
    #[allow(non_snake_case)]
    fn on_pre_step(&mut self, _sim_t: f64, sim_dt: f64, _mjd: f64) {
        self.ctx.SetEmptyMass(self.calc_empty_mass());
        
        let altitude = self.get_altitude();

        if altitude < 1.0 {
            return;
        }

        // Compute vehicle and attitude control state
        if self.descent_phase == DescentPhase::BeforeRetroIgnition
        {
            self.attitude_mode = AttitudeMode::GravityTurn;
            if altitude < RETRO_IGNITION_ALTITUDE
            {
                debug_string!("Firing retro. altitude = {:.2} mi", altitude / MI_IN_M);
                // Store the current z-axis orientation in global coordinates
                self.ctx.SetThrusterLevel(self.th_retro, 1.0);
                // descent_phase will be set after AMR is jettisoned 
            }else {
                debug_string!("Waiting to fire retro ... altitude = {:.2} mi", altitude / MI_IN_M);
            }
        }else if self.descent_phase == DescentPhase::AfterRetro {
            self.attitude_mode = AttitudeMode::GravityTurn;
            if altitude < ENGINE_CUTOFF_ALTITUDE
            {
                self.attitude_mode = AttitudeMode::Off;
            }
            else if altitude < TERMINAL_DESCENT_ALTITUDE
            {
                // Store the current z-axis orientation in global coordinates
                let mut target_orientation_global = Vector3::default();
                self.ctx.Local2Global(&DIR_Z_PLUS, &mut target_orientation_global);
                self.attitude_mode = AttitudeMode::InertialLock(target_orientation_global);
            }
        }else if self.descent_phase == DescentPhase::RetroFiring
        {
            self.attitude_mode = AttitudeMode::GravityTurn;
            debug_string!("Altitude: {:.2} ft", altitude/FT_IN_M);
        }

        // Get current main thruster level
        let reference_thrust  = if self.descent_phase == DescentPhase::AfterRetro && altitude > ENGINE_CUTOFF_ALTITUDE {
            let delta_thrust = if altitude >= DESCENT_CONTOUR_ALTITUDE
            {
                debug_string!("Constant Acceleration Mode, Altitude: {:.2} ft", altitude/FT_IN_M);
                self.const_acc_controller(0.9 * LUNAR_GRAVITY)
            }else if altitude > TERMINAL_DESCENT_ALTITUDE - 5.0 {
                // Approximate descent contour
                let (target_vel, target_acc) = self.get_descent_contour_velocity(altitude);
                let current_vel = self.get_surface_approach_vel();
                
                debug_string!("Descent Contour, Altitude: {:.2} ft, Target vel: {:.2} ft/s, Current vel: {:.2} ft/s", altitude/FT_IN_M, target_vel/FT_IN_M, self.get_surface_approach_vel()/FT_IN_M);
                if altitude < TERMINAL_DESCENT_ALTITUDE
                {
                    self.descent_phase = DescentPhase::TerminalDescent;
                }
                let delta_acc = (target_vel - current_vel)/sim_dt;
                self.const_acc_controller(target_acc + delta_acc)
            }else {
                0.
            };
            
            // Clamp to 0.95 to have some control margin
            self.last_thrust_level = (self.last_thrust_level + delta_thrust).clamp(0.0, 0.95);
            self.last_thrust_level
        }else if self.descent_phase == DescentPhase::TerminalDescent
        {
            let delta_thrust = if altitude >= ENGINE_CUTOFF_ALTITUDE
            {
                // Constant velocity
                let delta_th = self.const_velocity_controller(-1.5, sim_dt);
                debug_string!("Terminal Descent, Altitude: {:.2} ft, Current vel: {:.2} ft/s", altitude/FT_IN_M, self.get_surface_approach_vel()/FT_IN_M);
                delta_th
            }else{
                // Turn off engines
                if self.ctx.GroundContact()
                {
                    debug_string!("Touchdown!");
                }else{
                    debug_string!("Altitude: {:.2} ft", altitude/FT_IN_M);
                }
                self.last_thrust_level = 0.;
                0.
            };

            // Clamp to 0.95 to have some control margin
            self.last_thrust_level = (self.last_thrust_level + delta_thrust).clamp(0.0, 0.95);
            self.last_thrust_level
        }else {
            0.0
        };
       
        // Point spacecraft for gravity turn (or inertial hold)
        let angular_rate_target  = self.attitude_controller();
        let angular_acc = self.angular_rate_controller(&angular_rate_target);
        
        let vehicle_mass = self.calc_vehicle_mass();

        // Compute thrust values required to satisfy acceleration and attitude-control targets
        let (F_1, F_2, F_3, th1) = self.compute_thrust_from_ang_acc(vehicle_mass, reference_thrust, & angular_acc, 0.05);
        if altitude > ENGINE_CUTOFF_ALTITUDE && !self.ctx.GroundContact()
        {
            self.apply_thrusters(F_1, F_2, F_3, th1);
        }else {
            self.apply_thrusters(0., 0., 0., 0.);
        }
        
        // Stage jettison logic
        if self.descent_phase == DescentPhase::RetroFiring
            && self.ctx.GetPropellantMass(self.ph_retro) < 1.0
        {
            // Jettison the spent main retro
            self.jettison();
        }
        if self.descent_phase == DescentPhase::BeforeRetroIgnition
            && self.ctx.GetPropellantMass(self.ph_retro) < 0.999 * RETRO_PROP_MASS
        {
            // Jettison the AMR if the retro has started burning
            self.jettison();
            // Relight the retro if needed
            self.ctx.SetThrusterLevel(self.th_retro, 1.0);
        }

    }
    fn consume_buffered_key(
        &mut self,
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
                self.ctx.SetThrusterLevel(self.th_retro, 1.0);
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
    fn on_load_param(&mut self, param_data: &str) -> bool {
        if param_data.starts_with("DESCENT_PHASE")
        {
            let descent_phase_str = &param_data[14..];
            if let Ok(descent_phase) = DescentPhase::from_str(descent_phase_str)
            {
                self.descent_phase = descent_phase;
            }else{
                error!("Failed to parse DESCENT_PHASE from scenario: {}", descent_phase_str);
            }
            true
        }else{
            false
        }
    }
    fn on_save_state(&mut self, scn: &FileHandle) {
        let descent_phase_str: &'static str = self.descent_phase.clone().into();
        scn.write_scenario_string("DESCENT_PHASE", descent_phase_str);
    }
}

init_vessel!(
    fn init(vessel) 
    {
        Surveyor::new(vessel)
    }
    fn exit() {}
);
