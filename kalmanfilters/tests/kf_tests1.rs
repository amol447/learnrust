use kalmanfilters::vanillafilter::KalmanFilter;
use nalgebra as na;
use serde::Serialize;
use csv;
use std::{io, error::Error, fs, path::Path};
use std::fs::File;
use nalgebra::{dmatrix, dvector, DMatrix, DVector};
use rand::{thread_rng, distributions::Distribution, Rng};
use rand_distr::{StandardNormal, };
#[test]
fn test_serialize() -> Result<(), Box<dyn Error>>
{
    let t = na::dmatrix![1,2,3;4,5,6;7,8,9];
    let path = Path::new("tests/fixtures/init_state_1.csv");
    let file_writer = fs::OpenOptions::new().write(true).create_new(true).open(&path)?;
    let mut writer = csv::Writer::from_writer(file_writer);
    writer.serialize(t)?;
    writer.flush()?;
    Ok(())
}

#[test]
fn test_noisy_1d() {
    // Blatant copy from https://github.com/rlabbe/filterpy/blob/master/filterpy/kalman/tests/test_kf.py
    let init_state = dvector![2.0,0.0];
    let state_transition_matrix = dmatrix![1.0,1.0;0.0,1.0];
    let measurement_matrix = dmatrix![1.0, 0.0];
    let state_covariance_matrix = 1000 * DMatrix::identity(2, 2);
    let process_noise_covariance = 0.0001 * DMatrix::identity(2, 2);
    let measurement_noise_covariance = 2.0 * DMatrix::identity(1, 1);
    let control_matrix = DMatrix::zeros(1, 1);
    let kf = KalmanFilter::new(init_state, state_transition_matrix, measurement_matrix, control_matrix, state_covariance_matrix, measurement_noise_covariance, process_noise_covariance).unwrap();
    let mut rng = rand::thread_rng();
    let num_steps = 99;
    let  measurements:Vec<DVector<f64>> = (0..num_steps).map(|x| dvector![x+rng.sample(StandardNormal)]).collect();
    let dummy_input = dvector![0.0];
    let mu
    for i in 0..num_steps{
        kf.predict(&dummy_input);
        kf.process_measurement(&measurements[i])
        
        
        
    }
}