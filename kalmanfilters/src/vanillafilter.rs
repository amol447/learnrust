extern crate nalgebra as na;
use na::{DMatrix, DVector};

pub struct KalmanFilter {
    pub curr_state: DVector<f64>,                   //x_0
    state_transition: DMatrix<f64>,             //F
    measurement_matrix: DMatrix<f64>,           //H
    control_matrix: DMatrix<f64>,               //B
    pub state_covariance: DMatrix<f64>,             //P
    measurement_noise_covariance: DMatrix<f64>, //Q
    process_noise_covariance: DMatrix<f64>,     //R
}

impl KalmanFilter {
    pub fn new(
        curr_state: DVector<f64>,
        state_transition: DMatrix<f64>,
        measurement_matrix: DMatrix<f64>,
        control_matrix: DMatrix<f64>,
        state_covariance: DMatrix<f64>,
        measurement_noise_covariance: DMatrix<f64>,
        process_noise_covariance: DMatrix<f64>,
    ) -> Option<KalmanFilter> {
        let state_dim = curr_state.len();
        let (num_rows_meas, num_cols_meas) = measurement_matrix.shape();
        if num_cols_meas != state_dim {
            println!("state vector must have the same length as num columns in measurement matrix. state vector length={state_dim}, measurement_matrix dimensions are ({num_rows_meas},{num_cols_meas})");
            return None;
        }
        if check_square_matrix_size(&state_transition, state_dim, "state transition")
            || check_square_matrix_size(&state_covariance, state_dim, "state covariance")
            || check_square_matrix_size(&process_noise_covariance, state_dim, "process noise")
        {
            return None;
        }
        let (num_rows_control_matrix, num_cols_control_matrix) = control_matrix.shape();
        if num_rows_control_matrix != state_dim {
            println!("control matrix must be a  matrix of same num rows as state vector");
            return None;
        }
        if check_square_matrix_size(&measurement_noise_covariance, num_rows_meas, "measurement noise covariance") {
            return None;
        }


        Some(Self {
            curr_state,
            state_transition,
            measurement_matrix,
            control_matrix,
            state_covariance,
            measurement_noise_covariance,
            process_noise_covariance,
        })
    }
    pub fn process_measurement(self: &mut Self, measurement: &DVector<f64>) {
        let residual = measurement-(&self.measurement_matrix) * (&self.curr_state)  ;
        let innovation_covariance_inv = ((&self.measurement_matrix * &self.state_covariance) * self.measurement_matrix.transpose() + &self.measurement_noise_covariance).try_inverse().expect("innovation covaraince matrix not invertible. Something is wrong with the setup of Kalman filter");
        let kalman_gain = &self.state_covariance * &self.measurement_matrix.transpose() *innovation_covariance_inv;
        self.curr_state += &kalman_gain* &residual;
        let n =self.curr_state.len();
        self.state_covariance=(DMatrix::identity(n,n)-kalman_gain*&self.measurement_matrix)*&self.state_covariance;
        
    }
    pub fn predict(self: &mut Self, control_input:&DVector<f64>)  {
        self.curr_state=&self.state_transition*&self.curr_state+&self.control_matrix*control_input;
        self.state_covariance=&self.state_transition* &self.state_covariance*self.state_transition.transpose()+&self.process_noise_covariance;
        
    }
}

fn check_square_matrix_size(m: &DMatrix<f64>, s: usize, message: &str) -> bool {
    let (num_rows, num_cols) = m.shape();
    let ans = num_rows != s || num_cols != s;
    if ans {
        println!(
            "{} must be a square matrix with same rows and cols as state vector",
            message
        );
        println!("size of matrix is {num_rows}x{num_cols}");
        println!("size of state vector is {s}");
    }
    ans
}