use csv;
use kalmanfilters::vanillafilter::KalmanFilter;
use nalgebra as na;
use nalgebra::{dmatrix, dvector, DMatrix, DVector};
use rand::{distributions::Distribution, thread_rng, Rng};
use rand_distr::StandardNormal;
use serde::Serialize;
use std::fs::File;
use std::{error::Error, fs, io, path::Path};
#[test]
fn test_serialize() -> Result<(), Box<dyn Error>> {
    let t = na::dmatrix![1,2,3;4,5,6;7,8,9];
    let path = Path::new("tests/fixtures/init_state_1.csv");
    let file_writer = fs::OpenOptions::new()
        .write(true)
        .create_new(true)
        .open(&path)?;
    let mut writer = csv::Writer::from_writer(file_writer);
    writer.serialize(t)?;
    writer.flush()?;
    Ok(())
}

#[test]
fn test_noisy_1d() {
    // Blatant copy from https://github.com/rlabbe/filterpy/blob/master/filterpy/kalman/tests/test_kf.py
    let init_state = dvector![2.0, 0.0];
    let state_transition_matrix = dmatrix![1.0,1.0;0.0,1.0];
    let measurement_matrix = dmatrix![1.0, 0.0];
    let state_size = 2;
    let state_covariance_matrix = 1000.0 * DMatrix::identity(state_size, state_size);
    let process_noise_covariance = 0.0001 * DMatrix::identity(state_size, state_size);
    let measurement_noise_covariance = 2.0 * DMatrix::identity(1, 1);
    let control_matrix = DMatrix::zeros(state_size, 1);
    let mut kf = KalmanFilter::new(
        init_state,
        state_transition_matrix,
        measurement_matrix,
        control_matrix,
        state_covariance_matrix,
        measurement_noise_covariance,
        process_noise_covariance,
    )
    .unwrap();
    let mut rng = rand::thread_rng();
    let num_steps = 99;
    let measurements: Vec<DVector<f64>> = (0..num_steps)
        .map(|x| dvector![(x as f64) + rng.sample::<f64, StandardNormal>(StandardNormal)])
        .collect();
    let dummy_input = dvector![0.0];
    let mut state_track_vec: Vec<DVector<f64>> = Vec::new();
    let mut state_cov_track: Vec<DMatrix<f64>> = Vec::new();
    for i in 0..num_steps {
        kf.predict(&dummy_input);
        state_track_vec.push(kf.curr_state.clone());
        kf.process_measurement(&measurements[i]);
        state_cov_track.push(kf.state_covariance.clone());
        println!("{:?}", state_track_vec[i]);
    }
}

fn test_constant_acceleration(){
    let state_vector: DVector<f64> = dvector![0.0, 0.0, 0.0, 0.0, 1.0, 1.0];
    let delta_t = 0.1; // Example time step
    let state_transition_matrix: DMatrix<f64> = dmatrix![
        1.0, 0.0, delta_t, 0.0, 0.5 * delta_t * delta_t, 0.0;
        0.0, 1.0, 0.0, delta_t, 0.0, 0.5 * delta_t * delta_t;
        0.0, 0.0, 1.0, 0.0, delta_t, 0.0;
        0.0, 0.0, 0.0, 1.0, 0.0, delta_t;
        0.0, 0.0, 0.0, 0.0, 1.0, 0.0;
        0.0, 0.0, 0.0, 0.0, 0.0, 1.0
    ];
    let measurement_noise_covariance = 0.1 * DMatrix::identity(2, 2);
    let measurement_matrix: DMatrix<f64> = dmatrix![
        1.0, 0.0, 0.0, 0.0, 0.0, 0.0;
        0.0, 1.0, 0.0, 0.0, 0.0, 0.0
    ];
    let dummy_input = dvector![0.0];
    let init_state=dvector![0.0, 0.0, 1.0, 2.0, 0.2, -0.1];
    let control_matrix: DMatrix<f64> = DMatrix::zeros(6, 1);
    let state_covariance_matrix: DMatrix<f64> = DMatrix::identity(6, 6);
    let process_noise_covariance: DMatrix<f64> = 0.1 * DMatrix::identity(6, 6);
    // Create the Kalman filter
    let mut kf = KalmanFilter::new(
        init_state,
        state_transition_matrix,
        measurement_matrix,
        control_matrix,
        state_covariance_matrix,
        measurement_noise_covariance,
        process_noise_covariance,
    ).unwrap();
    let measurement = dvector![0.1, 0.2];
}