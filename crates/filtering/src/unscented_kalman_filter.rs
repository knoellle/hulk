use color_eyre::{eyre::ContextCompat, Result};
use nalgebra::{SMatrix, SVector};
use types::multivariate_normal_distribution::MultivariateNormalDistribution;

pub trait UnscentedKalmanFilter<const STATE_DIMENSION: usize> {
    fn predict_nonlinear<F>(
        &mut self,
        state_prediction: F,
        process_noise: SMatrix<f32, STATE_DIMENSION, STATE_DIMENSION>,
        mean_weight: f32,
    ) -> Result<()>
    where
        F: FnMut(SVector<f32, STATE_DIMENSION>) -> SVector<f32, STATE_DIMENSION>;
    fn generate_sigma_points(&self, mean_weight: f32)
        -> Result<Vec<SVector<f32, STATE_DIMENSION>>>;
}
pub fn calculate_mean<const DIMENSION: usize>(
    elements: &[SVector<f32, DIMENSION>],
    weights: &Vec<f32>,
) -> SVector<f32, DIMENSION> {
    weights
        .iter()
        .zip(elements.iter())
        .map(|(&weight, element)| weight * element)
        .sum()
}

pub fn calculate_covariance<const DIMENSION: usize>(
    elements: &[SVector<f32, DIMENSION>],
    mean: &SVector<f32, DIMENSION>,
    weights: &Vec<f32>,
) -> SMatrix<f32, DIMENSION, DIMENSION> {
    weights
        .iter()
        .zip(elements.iter().map(|element| element - mean))
        .map(|(weight, normalized_element)| {
            *weight * normalized_element * normalized_element.transpose()
        })
        .sum()
}

pub fn into_symmetric<const DIMENSION: usize>(
    matrix: SMatrix<f32, DIMENSION, DIMENSION>,
) -> SMatrix<f32, DIMENSION, DIMENSION> {
    0.5 * (matrix + matrix.transpose())
}

impl<const STATE_DIMENSION: usize> UnscentedKalmanFilter<STATE_DIMENSION>
    for MultivariateNormalDistribution<STATE_DIMENSION>
{
    fn predict_nonlinear<F>(
        &mut self,
        state_prediction: F,
        process_noise: SMatrix<f32, STATE_DIMENSION, STATE_DIMENSION>,
        mean_weight: f32,
    ) -> Result<()>
    where
        F: FnMut(SVector<f32, STATE_DIMENSION>) -> SVector<f32, STATE_DIMENSION>,
    {
        let sigma_points = self.generate_sigma_points(mean_weight)?;
        let predicted_sigma_points: Vec<SVector<f32, STATE_DIMENSION>> =
            sigma_points.into_iter().map(state_prediction).collect();
        let mut weights = vec![mean_weight];
        weights.resize(
            2 * STATE_DIMENSION + 1,
            (1.0 - mean_weight) / (2.0 * STATE_DIMENSION as f32),
        );
        let state_mean = calculate_mean(&predicted_sigma_points, &weights);
        let state_covariance = calculate_covariance(&predicted_sigma_points, &state_mean, &weights);
        self.mean = state_mean;
        // TODO: Check whether the symmetric operation is necessary
        self.covariance = into_symmetric(state_covariance + process_noise);

        Ok(())
    }

    fn generate_sigma_points(
        &self,
        mean_weight: f32,
    ) -> Result<Vec<SVector<f32, STATE_DIMENSION>>> {
        let covariance_cholesky = self
            .covariance
            .cholesky()
            .context("Failed to decompose covariance matrix via Cholesky decomposition")?;
        let covariance_square_root =
            ((STATE_DIMENSION as f32) / (1.0 - mean_weight)).sqrt() * covariance_cholesky.l();

        let mut sigma_points = vec![self.mean];
        for i in 0..STATE_DIMENSION {
            sigma_points.push(self.mean - covariance_square_root.column(i));
            sigma_points.push(self.mean + covariance_square_root.column(i));
        }
        Ok(sigma_points)
    }
}
