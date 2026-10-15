use crate::constants::data_deriver_constants::DataDeriverConstants;

const SPECIFIC_HEAT_RATIO_AIR: f32 = DataDeriverConstants::SPECIFIC_HEAT_RATIO_AIR;
const SPECIFIC_GAS_CONSTANT_AIR_J_PER_KG_K: f32 =
    DataDeriverConstants::SPECIFIC_GAS_CONSTANT_AIR_J_PER_KG_K;
const CELSIUS_TO_KELVIN_OFFSET: f32 = DataDeriverConstants::CELSIUS_TO_KELVIN_OFFSET;
const RAD_TO_DEG: f32 = DataDeriverConstants::RAD_TO_DEG;
const IMU_Z_AXIS_CCW_ROTATION_DEGREES: f32 = DataDeriverConstants::IMU_Z_AXIS_CCW_ROTATION_DEGREES;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MountAxis {
    PosX,
    NegX,
    PosY,
    NegY,
    PosZ,
    NegZ,
}

#[derive(Debug, Clone)]
pub struct DataProcessor {
    mount_axis: Option<MountAxis>,
}

impl DataProcessor {
    fn mount_axis_unit_vector(mount_axis: MountAxis) -> (f32, f32, f32) {
        match mount_axis {
            MountAxis::PosX => (1.0, 0.0, 0.0),
            MountAxis::NegX => (-1.0, 0.0, 0.0),
            MountAxis::PosY => (0.0, 1.0, 0.0),
            MountAxis::NegY => (0.0, -1.0, 0.0),
            MountAxis::PosZ => (0.0, 0.0, 1.0),
            MountAxis::NegZ => (0.0, 0.0, -1.0),
        }
    }

    fn rotate_acceleration_about_z_ccw(
        &self,
        acceleration_x_gs: f32,
        acceleration_y_gs: f32,
        acceleration_z_gs: f32,
    ) -> (f32, f32, f32) {
        let angle_radians = IMU_Z_AXIS_CCW_ROTATION_DEGREES.to_radians();
        let cos_angle = angle_radians.cos();
        let sin_angle = angle_radians.sin();

        let rotated_x = acceleration_x_gs * cos_angle - acceleration_y_gs * sin_angle;
        let rotated_y = acceleration_x_gs * sin_angle + acceleration_y_gs * cos_angle;

        (rotated_x, rotated_y, acceleration_z_gs)
    }

    pub fn new() -> Self {
        Self { mount_axis: None }
    }

    pub fn mount_axis(&self) -> Option<MountAxis> {
        self.mount_axis
    }

    pub fn reset_mount_axis(&mut self) {
        self.mount_axis = None;
    }

    /// Computes tilt angle (degrees) from the rocket body axis and orientation quaternion.
    ///
    /// Acceleration is only used to infer the rocket axis in body frame once.
    /// The tilt itself is then computed from quaternion-rotated rocket axis to world +Z.
    /// TODO: unsure if this works or not, FIRM might just have the wrong orientations for sensors.
    #[allow(clippy::too_many_arguments)]
    pub fn derive_tilt_angle_degrees(
        &mut self,
        raw_acceleration_x_gs: f32,
        raw_acceleration_y_gs: f32,
        raw_acceleration_z_gs: f32,
        est_quaternion_w: f32,
        est_quaternion_x: f32,
        est_quaternion_y: f32,
        est_quaternion_z: f32,
    ) -> f32 {
        if !raw_acceleration_x_gs.is_finite()
            || !raw_acceleration_y_gs.is_finite()
            || !raw_acceleration_z_gs.is_finite()
            || !est_quaternion_w.is_finite()
            || !est_quaternion_x.is_finite()
            || !est_quaternion_y.is_finite()
            || !est_quaternion_z.is_finite()
        {
            return 0.0;
        }

        let (corrected_acceleration_x_gs, corrected_acceleration_y_gs, corrected_acceleration_z_gs) =
            self.rotate_acceleration_about_z_ccw(
                raw_acceleration_x_gs,
                raw_acceleration_y_gs,
                raw_acceleration_z_gs,
            );

        if self.mount_axis.is_none() {
            self.mount_axis = Some(self.derive_mount_axis_from_boot_acceleration(
                corrected_acceleration_x_gs,
                corrected_acceleration_y_gs,
                corrected_acceleration_z_gs,
            ));
        }

        let quaternion_norm = (est_quaternion_w * est_quaternion_w
            + est_quaternion_x * est_quaternion_x
            + est_quaternion_y * est_quaternion_y
            + est_quaternion_z * est_quaternion_z)
            .sqrt();
        if quaternion_norm <= f32::EPSILON {
            return 0.0;
        }

        let qw = est_quaternion_w / quaternion_norm;
        let qx = est_quaternion_x / quaternion_norm;
        let qy = est_quaternion_y / quaternion_norm;
        let qz = est_quaternion_z / quaternion_norm;

        let (axis_x, axis_y, axis_z) =
            Self::mount_axis_unit_vector(self.mount_axis.unwrap_or(MountAxis::PosZ));

        let tx = 2.0 * (qy * axis_z - qz * axis_y);
        let ty = 2.0 * (qz * axis_x - qx * axis_z);
        let tz = 2.0 * (qx * axis_y - qy * axis_x);

        let world_axis_x = axis_x + qw * tx + (qy * tz - qz * ty);
        let world_axis_y = axis_y + qw * ty + (qz * tx - qx * tz);
        let world_axis_z = axis_z + qw * tz + (qx * ty - qy * tx);

        let world_axis_magnitude = (world_axis_x * world_axis_x
            + world_axis_y * world_axis_y
            + world_axis_z * world_axis_z)
            .sqrt();
        if world_axis_magnitude <= f32::EPSILON {
            return 0.0;
        }

        let cos_theta = (world_axis_z / world_axis_magnitude).clamp(-1.0, 1.0);
        cos_theta.acos() * RAD_TO_DEG
    }

    /// Rotates a body-frame acceleration vector using an orientation quaternion.
    ///
    /// The quaternion is normalized internally and treated as a vehicle-to-world
    /// rotation estimate, so it is applied directly to map body vectors to world.
    #[allow(clippy::too_many_arguments)]
    pub fn derive_rotated_raw_acceleration(
        &self,
        raw_acceleration_x_gs: f32,
        raw_acceleration_y_gs: f32,
        raw_acceleration_z_gs: f32,
        est_quaternion_w: f32,
        est_quaternion_x: f32,
        est_quaternion_y: f32,
        est_quaternion_z: f32,
    ) -> (f32, f32, f32) {
        if !raw_acceleration_x_gs.is_finite()
            || !raw_acceleration_y_gs.is_finite()
            || !raw_acceleration_z_gs.is_finite()
            || !est_quaternion_w.is_finite()
            || !est_quaternion_x.is_finite()
            || !est_quaternion_y.is_finite()
            || !est_quaternion_z.is_finite()
        {
            return (0.0, 0.0, 0.0);
        }

        let (raw_acceleration_x_gs, raw_acceleration_y_gs, raw_acceleration_z_gs) = self
            .rotate_acceleration_about_z_ccw(
                raw_acceleration_x_gs,
                raw_acceleration_y_gs,
                raw_acceleration_z_gs,
            );

        let quaternion_norm = (est_quaternion_w * est_quaternion_w
            + est_quaternion_x * est_quaternion_x
            + est_quaternion_y * est_quaternion_y
            + est_quaternion_z * est_quaternion_z)
            .sqrt();

        if quaternion_norm <= f32::EPSILON {
            return (
                raw_acceleration_x_gs,
                raw_acceleration_y_gs,
                raw_acceleration_z_gs,
            );
        }

        let qw = est_quaternion_w / quaternion_norm;
        let qx = est_quaternion_x / quaternion_norm;
        let qy = est_quaternion_y / quaternion_norm;
        let qz = est_quaternion_z / quaternion_norm;

        let vx = raw_acceleration_x_gs;
        let vy = raw_acceleration_y_gs;
        let vz = raw_acceleration_z_gs;

        // Optimized quaternion-vector rotation:
        // t = 2 * cross(q.xyz, v)
        // v' = v + qw * t + cross(q.xyz, t)
        let tx = 2.0 * (qy * vz - qz * vy);
        let ty = 2.0 * (qz * vx - qx * vz);
        let tz = 2.0 * (qx * vy - qy * vx);

        let rotated_x = vx + qw * tx + (qy * tz - qz * ty);
        let rotated_y = vy + qw * ty + (qz * tx - qx * tz);
        let rotated_z = vz + qw * tz + (qx * ty - qy * tx);

        (rotated_x, rotated_y, rotated_z)
    }

    /// Chooses the nearest signed body axis (+/-X, +/-Y, +/-Z) to a boot-time
    /// acceleration sample. This is useful for one-time mounting-axis detection.
    pub fn derive_mount_axis_from_boot_acceleration(
        &self,
        raw_acceleration_x_gs: f32,
        raw_acceleration_y_gs: f32,
        raw_acceleration_z_gs: f32,
    ) -> MountAxis {
        if !raw_acceleration_x_gs.is_finite()
            || !raw_acceleration_y_gs.is_finite()
            || !raw_acceleration_z_gs.is_finite()
        {
            return MountAxis::PosZ;
        }

        let magnitude = (raw_acceleration_x_gs * raw_acceleration_x_gs
            + raw_acceleration_y_gs * raw_acceleration_y_gs
            + raw_acceleration_z_gs * raw_acceleration_z_gs)
            .sqrt();

        if magnitude <= f32::EPSILON {
            return MountAxis::PosZ;
        }

        let nx = raw_acceleration_x_gs / magnitude;
        let ny = raw_acceleration_y_gs / magnitude;
        let nz = raw_acceleration_z_gs / magnitude;

        let candidates = [
            (MountAxis::PosX, (1.0f32, 0.0f32, 0.0f32)),
            (MountAxis::NegX, (-1.0f32, 0.0f32, 0.0f32)),
            (MountAxis::PosY, (0.0f32, 1.0f32, 0.0f32)),
            (MountAxis::NegY, (0.0f32, -1.0f32, 0.0f32)),
            (MountAxis::PosZ, (0.0f32, 0.0f32, 1.0f32)),
            (MountAxis::NegZ, (0.0f32, 0.0f32, -1.0f32)),
        ];

        let mut best_axis = MountAxis::PosZ;
        let mut best_dot = f32::NEG_INFINITY;
        for (axis, vector) in candidates {
            let dot = nx * vector.0 + ny * vector.1 + nz * vector.2;
            if dot > best_dot {
                best_dot = dot;
                best_axis = axis;
            }
        }

        best_axis
    }

    /// Computes Mach number from estimated velocity magnitude and ambient temperature.
    pub fn derive_mach_number(
        &self,
        est_velocity_z_meters_per_s: f32,
        temperature_celsius: f32,
    ) -> f32 {
        if !est_velocity_z_meters_per_s.is_finite() || !temperature_celsius.is_finite() {
            return 0.0;
        }

        let temperature_kelvin = temperature_celsius + CELSIUS_TO_KELVIN_OFFSET;
        if temperature_kelvin <= 0.0 {
            return 0.0;
        }

        let speed_of_sound_m_per_s =
            (SPECIFIC_HEAT_RATIO_AIR * SPECIFIC_GAS_CONSTANT_AIR_J_PER_KG_K * temperature_kelvin)
                .sqrt();
        if speed_of_sound_m_per_s <= f32::EPSILON {
            return 0.0;
        }

        let speed_m_per_s = (est_velocity_z_meters_per_s * est_velocity_z_meters_per_s).sqrt();

        speed_m_per_s / speed_of_sound_m_per_s
    }
}

impl Default for DataProcessor {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::{DataProcessor, MountAxis};
    use crate::constants::data_deriver_constants::DataDeriverConstants;

    #[test]
    fn rotated_accel_identity() {
        let deriver = DataProcessor::new();
        let (x, y, z) = deriver.derive_rotated_raw_acceleration(0.0, 0.0, 1.0, 1.0, 0.0, 0.0, 0.0);
        assert!(x.abs() < 1e-6);
        assert!(y.abs() < 1e-6);
        assert!((z - 1.0).abs() < 1e-6);
    }

    #[test]
    fn accel_mount_rotation_applied() {
        let deriver = DataProcessor::new();
        let (x, y, z) = deriver.derive_rotated_raw_acceleration(1.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0);

        let angle_radians = DataDeriverConstants::IMU_Z_AXIS_CCW_ROTATION_DEGREES.to_radians();
        assert!((x - angle_radians.cos()).abs() < 1e-6);
        assert!((y - angle_radians.sin()).abs() < 1e-6);
        assert!(z.abs() < 1e-6);
    }

    #[test]
    fn rotated_accel_vehicle_to_world_quaternion() {
        let deriver = DataProcessor::new();
        // Simulate gravity in body frame and rotate it into world using a vehicle->world quaternion.
        let body_gravity = (0.0f32, 0.0f32, 1.0f32);
        let half_angle = core::f32::consts::FRAC_PI_4 / 2.0;
        let qw = half_angle.cos();
        let qx = 0.0;
        let qy = half_angle.sin();
        let qz = 0.0;

        // Deriver should rotate body reading into world frame.
        let (world_x, world_y, world_z) = deriver.derive_rotated_raw_acceleration(
            body_gravity.0,
            body_gravity.1,
            body_gravity.2,
            qw,
            qx,
            qy,
            qz,
        );

        assert!((world_x - core::f32::consts::FRAC_1_SQRT_2).abs() < 1e-5);
        assert!(world_y.abs() < 1e-5);
        assert!((world_z - core::f32::consts::FRAC_1_SQRT_2).abs() < 1e-5);
    }

    #[test]
    fn tilt_from_quaternion_aligned_and_perpendicular() {
        let mut deriver = DataProcessor::new();

        let aligned = deriver.derive_tilt_angle_degrees(0.0, 0.0, 1.0, 1.0, 0.0, 0.0, 0.0);
        assert!(aligned.abs() < 1e-6);

        deriver.reset_mount_axis();
        let perpendicular = deriver.derive_tilt_angle_degrees(1.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0);
        assert!((perpendicular - 90.0).abs() < 1e-5);
    }

    #[test]
    fn tilt_uses_quaternion_after_axis_latched() {
        let mut deriver = DataProcessor::new();

        let aligned = deriver.derive_tilt_angle_degrees(0.0, 0.0, 1.0, 1.0, 0.0, 0.0, 0.0);
        assert!(aligned.abs() < 1e-6);

        // Once axis is latched, noisy acceleration should not change tilt when quaternion is unchanged.
        let noisy = deriver.derive_tilt_angle_degrees(0.3, -0.7, 0.2, 1.0, 0.0, 0.0, 0.0);
        assert!(noisy.abs() < 1e-6);
    }

    #[test]
    fn mount_axis_snap_chooses_closest_axis() {
        let deriver = DataProcessor::new();
        let axis = deriver.derive_mount_axis_from_boot_acceleration(0.05, -0.97, 0.14);
        assert_eq!(axis, MountAxis::NegY);
    }

    #[test]
    fn test_mach_number() {
        let deriver = DataProcessor::new();
        let mach = deriver.derive_mach_number(0.0, 20.0);
        assert!(mach.abs() < 1e-6);

        let mach = deriver.derive_mach_number(50.0, 20.0);
        assert!((mach - 0.14568).abs() < 1e-3);

        let mach = deriver.derive_mach_number(360.0, 20.0);
        assert!((mach - 1.049).abs() < 1e-3);
    }
}
