
#[macro_use]
extern crate approx; // For the macro assert_relative_eq!
extern crate nalgebra as na;
extern crate derive_more;


use na::{DMatrix, DVector};
struct KalmanFilter{
    init_state:DVector<f64>,//x_0
    state_transition:DMatrix<f64>, //F
    measurement_matrix:DMatrix<f64>,//H
    control_matrix:DMatrix<f64>,//B
    state_covariance:DMatrix<f64>,//P
    measurement_noise_covariance:DMatrix<f64>,//Q
    process_noise_covariance:DMatrix<f64>,//R
}
fn check_square_matrix_size(m:&DMatrix<f64>, s:usize, message:&str) -> bool{
    let (num_rows, num_cols) = m.shape();
    let ans = num_rows != s || num_cols != s;
    if ans {
        println!("{} must be a square matrix with same rows and cols as state vector", message);
        println!("size of matrix is {num_rows}x{num_cols}");
        println!("size of state vector is {s}");
    }
    ans
}

impl KalmanFilter {
    pub fn new(init_state:DVector<f64>, state_transition:DMatrix<f64>,measurement_matrix:DMatrix<f64>, control_matrix:DMatrix<f64>,state_covariance:DMatrix<f64>, measurement_noise_covariance:DMatrix<f64>, process_noise_covariance:DMatrix<f64>) -> Option<KalmanFilter> {
        let state_dim = init_state.len();
        let (num_rows_meas, num_cols_meas) = measurement_matrix.shape();
        if num_cols_meas !=state_dim{
            println!("state vector must have the same length as num columns in measurement matrix. state vector length={state_dim}, measurement_matrix dimensions are ({num_rows_meas},{num_cols_meas})");
            return None
        }
        if check_square_matrix_size(&state_transition,state_dim,"state transition") || check_square_matrix_size(&state_covariance,state_dim,"state covariance") || check_square_matrix_size(&process_noise_covariance,state_dim,"process noise") {
            println!("state covariance must be a square matrix of same dimension as state vector");
            return None
        }
        let (num_rows_control_matrix,num_cols_control_matrix) = control_matrix.shape();
        if num_rows_control_matrix!=state_dim {
            println!("control matrix must be a  matrix of same num rows as state vector");
            return None
        }
        let(num_rows_process_noise_covariance,num_cols_process_noise_covariance) = process_noise_covariance.shape();
        
       
        
        Some(Self{init_state,state_transition,measurement_matrix,control_matrix, state_covariance, measurement_noise_covariance,process_noise_covariance})
    }

}
#[derive(Debug)]
struct Test{
    x:isize,
    y:String
}
impl Test {
    fn new(x:isize,y:String) -> Test {
        Self{x,y}
    }
}
fn main() {
    println!("Hello, world!");
    let s=Test::new(1,"1".to_string());
    println!("{:?}", s);
}