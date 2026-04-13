use crate::firm_packets::ProcessedFIRMData;
use nalgebra::{Matrix3, Vector3};
use std::vec::Vec;

/// Stores the result of a magnetometer calibration.
#[derive(Debug, Clone, Copy)]
pub struct MagnetometerCalibration {
    /// Hard Iron bias (offsets to subtract).
    pub hard_iron_bias: Vector3<f32>,
    /// Soft Iron matrix (transformation to apply after subtraction).
    pub soft_iron_matrix: Matrix3<f32>,
    /// The expected field strength (radius of the fitted sphere).
    pub field_strength: f32,
}

impl MagnetometerCalibration {
    /// Applies the calibration to a raw data point.
    /// Returns the corrected magnetic field vector.
    pub fn apply(&self, x: f32, y: f32, z: f32) -> Vector3<f32> {
        let raw = Vector3::new(x, y, z);
        // Formula: M * (raw - bias)
        self.soft_iron_matrix * (raw - self.hard_iron_bias)
    }

    /// Returns an identity calibration (no change to data).
    pub fn identity() -> Self {
        Self {
            hard_iron_bias: Vector3::zeros(),
            soft_iron_matrix: Matrix3::identity(),
            field_strength: 0.0,
        }
    }

    /// Exports the calibration parameters as flat arrays suitable for
    /// serialization or firmware configuration.
    ///
    /// # Returns
    ///
    /// A tuple containing:
    /// - `[f32; 3]`: The hard iron offsets [x, y, z].
    /// - `[f32; 9]`: The soft iron matrix in row-major order [m11, m12, m13, m21...].
    pub fn to_arrays(&self) -> ([f32; 3], [f32; 9]) {
        let offsets = [
            self.hard_iron_bias.x,
            self.hard_iron_bias.y,
            self.hard_iron_bias.z,
        ];

        // Puts the matrix in row-major order
        let matrix: [f32; 9] = [
            self.soft_iron_matrix[(0, 0)],
            self.soft_iron_matrix[(0, 1)],
            self.soft_iron_matrix[(0, 2)],
            self.soft_iron_matrix[(1, 0)],
            self.soft_iron_matrix[(1, 1)],
            self.soft_iron_matrix[(1, 2)],
            self.soft_iron_matrix[(2, 0)],
            self.soft_iron_matrix[(2, 1)],
            self.soft_iron_matrix[(2, 2)],
        ];

        (offsets, matrix)
    }
}

/// Accumulates processed telemetry packets and calculates magnetometer calibration parameters
/// using least squares ellipsoid fitting (similar to MATLAB's magcal).
pub struct MagnetometerCalibrator {
    /// Buffer of collected points (x, y, z).
    samples: Vec<Vector3<f32>>,
    /// Whether we are currently accepting new data points.
    is_collecting: bool,
}

impl Default for MagnetometerCalibrator {
    fn default() -> Self {
        Self::new()
    }
}

impl MagnetometerCalibrator {
    /// Creates a new calibrator instance.
    pub fn new() -> Self {
        Self {
            samples: Vec::new(),
            is_collecting: false,
        }
    }

    /// Starts the calibration process. Clears previous data.
    pub fn start(&mut self) {
        self.samples.clear();
        self.is_collecting = true;
    }

    /// Stops collecting data.
    pub fn stop(&mut self) {
        self.is_collecting = false;
    }

    /// Adds a data packet to the calibration buffer if collecting.
    pub fn add_sample(&mut self, data: &ProcessedFIRMData) {
        if self.is_collecting {
            self.samples.push(Vector3::new(
                data.magnetic_field_x_microteslas,
                data.magnetic_field_y_microteslas,
                data.magnetic_field_z_microteslas,
            ));
        }
    }

    /// Adds a raw magnetometer sample (x, y, z) to the calibration buffer if collecting.
    pub fn add_sample_xyz(&mut self, x: f32, y: f32, z: f32) {
        if self.is_collecting {
            self.samples.push(Vector3::new(x, y, z));
        }
    }

    /// Returns the number of samples currently collected.
    pub fn sample_count(&self) -> usize {
        self.samples.len()
    }

    /// Performs the math to solve for Hard Iron and Soft Iron parameters.
    ///
    /// This fits the equation: (x-c)' A (x-c) = 1
    /// Returns `None` if there is insufficient data or the solver fails.
    pub fn calculate(&self) -> Option<MagnetometerCalibration> {
        let n = self.samples.len();
        if n < 10 {
            // Need at least 9 points to fit an ellipsoid, but more is better for noise.
            return None;
        }

        // Construct the Design Matrix D (N x 9) for:
        // ax^2 + by^2 + cz^2 + 2dxy + 2exz + 2fyz + 2gx + 2hy + 2iz = 1
        let mut d_matrix = nalgebra::DMatrix::<f32>::zeros(n, 9);
        let ones = nalgebra::DVector::<f32>::from_element(n, 1.0);

        for (i, p) in self.samples.iter().enumerate() {
            let x = p.x;
            let y = p.y;
            let z = p.z;

            // Columns: [x^2, y^2, z^2, xy, xz, yz, x, y, z]
            d_matrix[(i, 0)] = x * x;
            d_matrix[(i, 1)] = y * y;
            d_matrix[(i, 2)] = z * z;
            d_matrix[(i, 3)] = 2.0 * x * y;
            d_matrix[(i, 4)] = 2.0 * x * z;
            d_matrix[(i, 5)] = 2.0 * y * z;
            d_matrix[(i, 6)] = 2.0 * x;
            d_matrix[(i, 7)] = 2.0 * y;
            d_matrix[(i, 8)] = 2.0 * z;
        }

        // 2. Solve D * v = 1 for parameter vector v
        let solution = d_matrix.svd(true, true).solve(&ones, 1e-6).ok()?;

        // 3. Unpack parameters into Algebraic Matrix Q and Vector U
        // Q = [a d e; d b f; e f c]
        let a = solution[0];
        let b = solution[1];
        let c = solution[2];
        let d = solution[3];
        let e = solution[4];
        let f = solution[5];
        let g = solution[6];
        let h = solution[7];
        let sol_i = solution[8];

        let q_matrix = Matrix3::new(a, d, e, d, b, f, e, f, c);

        let u_vec = Vector3::new(g, h, sol_i);

        // 4. Calculate Center (Hard Iron Bias)
        // center = - Q^-1 * U
        let q_inv = q_matrix.try_inverse()?;
        let center = -q_inv * u_vec;

        // 5. Calculate Soft Iron Matrix
        // We transform the fitted ellipsoid into a sphere.
        // T_matrix = sqrt(Q)

        // Eigen decomposition of the shape matrix Q
        // Since Q is symmetric, we can use SymmetricEigen
        let eigen = q_matrix.symmetric_eigen();

        // Reconstruct the scaling matrix.
        // We want to map the ellipsoid to a sphere of radius 'B'.
        // The equation at the center is (x-c)' Q (x-c) = 1 + c' Q c
        // Let radius_sq = 1 + c' Q c.
        // Effective shape matrix M = Q / radius_sq.

        // center' * Q * center yields a 1x1 matrix; extract scalar
        let term = center.transpose() * q_matrix * center;
        let term_scalar = term[(0, 0)];
        let radius_sq = 1.0 + term_scalar;
        if radius_sq <= 0.0 {
            return None;
        }
        let estimated_field_strength = radius_sq.sqrt();

        // To get the Soft Iron matrix that normalizes data to a sphere:
        // S = V * D^0.5 * V^T
        // We iterate over eigenvalues to sqrt them.
        let mut d_sqrt = Matrix3::zeros();
        for idx in 0..3 {
            if eigen.eigenvalues[idx] < 0.0 {
                // If eigenvalues are negative, the fit failed (hyperboloid, not ellipsoid).
                return None;
            }
            d_sqrt[(idx, idx)] = eigen.eigenvalues[idx].sqrt();
        }

        // This matrix maps the fitted ellipsoid to a unit sphere.
        let soft_iron_unit = (eigen.eigenvectors * d_sqrt * eigen.eigenvectors.transpose())
            * (1.0 / estimated_field_strength);

        let mut sum_norm = 0.0f32;
        for p in &self.samples {
            let v = p - center;
            sum_norm += v.norm();
        }
        let mean_norm = sum_norm / (n as f32);
        let scale = if mean_norm.is_finite() && mean_norm > 0.0 {
            mean_norm
        } else {
            1.0
        };

        let soft_iron = soft_iron_unit * scale;

        Some(MagnetometerCalibration {
            hard_iron_bias: center,
            soft_iron_matrix: soft_iron,
            field_strength: mean_norm,
        })
    }
}
