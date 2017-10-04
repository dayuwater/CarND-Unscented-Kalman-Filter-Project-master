#include "ukf.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

// The code is currently working on sensor fusion mode with parameters tuned to Dataset 1

// Dataset 1 ( The one in the project folder ):
// lidar only : [0.17, 0.15, 0.65, 0.35]   EKF: [0.12, 0.10, 0.58, 0.45]
// radar only : [0.22, 0.31, 0.41, 0.38]   EKF: [0.19, 0.3, 0.55, 0.66]
// sensor fusion: [0.07, 0.08, 0.35, 0.25] EKF : [0.09, 0.08, 0.45, 0.44]

// Dataset 2: ( I used the same parameters as in Dataset 1 )
// lidar only : [0.16, 0.14, 0.55, 0.37]   EKF: [0.10, 0.10, 0.54, 0.46]
// radar only : [0.34, 0.32, 0.76, 0.44]   EKF: [0.22, 0.29, 0.59, 0.73]
// sensor fusion: [0.10, 0.07, 0.70, 0.33] EKF : [0.07, 0.09, 0.46, 0.5]

// Under the same parameters and conditions, UKF performs better than EKF for sensor fusion if UKF has a good estimation about the process noise of the objects it measures. If the estimation of process noise is not ideal, EKF outperforms UKF

// However, EKF outperforms UKF for position estimations if we only use a single type of sensor. UKF is still better at velocity estimations in this case

/**
 * Initializes Unscented Kalman filter
 */
UKF::UKF() {
    // if this is false, laser measurements will be ignored (except during init)
    use_laser_ = true;
    
    // if this is false, radar measurements will be ignored (except during init)
    use_radar_ = true;
    
    // initial state vector
    x_ = VectorXd(5);
    
    // initial covariance matrix
    P_ = MatrixXd(5, 5);
    
    // Process noise standard deviation longitudinal acceleration in m/s^2
    std_a_ = 2;
    
    // Process noise standard deviation yaw acceleration in rad/s^2
    std_yawdd_ = 2; 
    
    // Laser measurement noise standard deviation position1 in m
    std_laspx_ = 0.15;
    
    // Laser measurement noise standard deviation position2 in m
    std_laspy_ = 0.15;
    
    // Radar measurement noise standard deviation radius in m
    std_radr_ = 0.3;
    
    // Radar measurement noise standard deviation angle in rad
    std_radphi_ = 0.03;
    
    // Radar measurement noise standard deviation radius change in m/s
    std_radrd_ = 0.3;
    
    /**
     TODO:
     
     Complete the initialization. See ukf.h for other member properties.
     
     Hint: one or more values initialized above might be wildly off...
     */
    
    n_x_ = 5;
    n_aug_ = 7;
    lambda_ = 3 - n_aug_;
    Xsig_pred_ = Eigen::MatrixXd(n_x_, 2 * n_aug_ + 1);
    weights_ = Eigen::VectorXd(2*n_aug_+1);
    
}

UKF::~UKF() {}

/**
 * @param {MeasurementPackage} meas_package The latest measurement data of
 * either radar or laser.
 */
void UKF::ProcessMeasurement(MeasurementPackage meas_package) {
    /**
     TODO:
     
     Complete this function! Make sure you switch between lidar and radar
     measurements.
     */
    // if the UKF is not initialized
    if(!is_initialized_){
        
        double px, py;
        // Try to decide the value to initialize based on the type of the first input
        if(meas_package.sensor_type_ == meas_package.LASER){
            // initialize x, convert lidar space to CTRV space
            px = meas_package.raw_measurements_(0);
            py = meas_package.raw_measurements_(1);
            
        }
        else if(meas_package.sensor_type_ == meas_package.RADAR){
            // initialize x, convert radar space to CTRV space, DO NOT USE RADAR VELOCITY!!
            double rho = meas_package.raw_measurements_(0);
            double phi = meas_package.raw_measurements_(1);
            
            px = rho * sin(phi);
            py = rho * cos(phi);
        }
        else{
            cout << "Detected an unknown sensor" << endl;
            return;
        }
        
        // initalize x
        x_(0) = px;
        x_(1) = py;
        x_(2) = 0;
        x_(3) = 0;
        x_(4) = 0;
        
        // initialize P
        P_(0,0) = 1;  // px
        P_(1,1) = 1;  // py
        P_(2,2) = 1;  // v
        P_(3,3) = 1;  // yaw
        P_(4,4) = 1;  // yawdot

        
        is_initialized_ = true;
        time_us_ = meas_package.timestamp_;
    }
    
    
    // if the UKF is initialized
    else{
        if(meas_package.sensor_type_ == meas_package.LASER and use_laser_){
            double delta_t = (meas_package.timestamp_ - time_us_) / 1000000.0;
            time_us_ = meas_package.timestamp_;
            // predict
            Prediction(delta_t);
            UpdateLidar(meas_package);
            
        }
        else if(meas_package.sensor_type_ == meas_package.RADAR and use_radar_){
            double delta_t = (meas_package.timestamp_ - time_us_) / 1000000.0;
            time_us_ = meas_package.timestamp_;
            // predict
            Prediction(delta_t);
            UpdateRadar(meas_package);

        }
        
    }
    
}

/**
 * Predicts sigma points, the state, and the state covariance matrix.
 * @param {double} delta_t the change in time (in seconds) between the last
 * measurement and this one.
 */
void UKF::Prediction(double delta_t) {
    /**
     TODO:
     
     Complete this function! Estimate the object's location. Modify the state
     vector, x_. Predict sigma points, the state, and the state covariance matrix.
     */
    
    // Step 1: Initialize Variables
    
    //create sigma point matrix
    VectorXd x_aug = VectorXd(7);
    MatrixXd P_aug = MatrixXd(7, 7);
    MatrixXd Xsig_aug = MatrixXd(n_aug_, 2 * n_aug_ + 1);
    
    
    // Step 2: Generate Augmented Sigma Points
    lambda_ = 3 - n_aug_;
    //create augmented mean state
    x_aug.head(5) = x_;
    x_aug(5) = 0;
    x_aug(6) = 0;
    
    //create augmented covariance matrix
    P_aug.fill(0.0);
    P_aug.topLeftCorner(5,5) = P_;
    P_aug(5,5) = std_a_*std_a_;
    P_aug(6,6) = std_yawdd_*std_yawdd_;
    
    //create square root matrix
    MatrixXd L = P_aug.llt().matrixL();
    
    //create augmented sigma points
    Xsig_aug.col(0)  = x_aug;
    for (int i = 0; i< n_aug_; i++)
    {
        Xsig_aug.col(i+1)       = x_aug + sqrt(lambda_+n_aug_) * L.col(i);
        Xsig_aug.col(i+1+n_aug_) = x_aug - sqrt(lambda_+n_aug_) * L.col(i);
    }
    
    
    // Step 3: Predict Sigma Points
    
    for (int i = 0; i< 2*n_aug_+1; i++)
    {
        //extract values for better readability
        double p_x = Xsig_aug(0,i);
        double p_y = Xsig_aug(1,i);
        double v = Xsig_aug(2,i);
        double yaw = Xsig_aug(3,i);
        double yawd = Xsig_aug(4,i);
        double nu_a = Xsig_aug(5,i);
        double nu_yawdd = Xsig_aug(6,i);
        
        //predicted state values
        double px_p, py_p;
        
        //avoid division by zero
        if (fabs(yawd) > 0.001) {
            px_p = p_x + v/yawd * ( sin (yaw + yawd*delta_t) - sin(yaw));
            py_p = p_y + v/yawd * ( cos(yaw) - cos(yaw+yawd*delta_t) );
        }
        else {
            px_p = p_x + v*delta_t*cos(yaw);
            py_p = p_y + v*delta_t*sin(yaw);
        }
        
        double v_p = v;
        double yaw_p = yaw + yawd*delta_t;
        double yawd_p = yawd;
        
        //add noise
        px_p = px_p + 0.5*nu_a*delta_t*delta_t * cos(yaw);
        py_p = py_p + 0.5*nu_a*delta_t*delta_t * sin(yaw);
        v_p = v_p + nu_a*delta_t;
        
        yaw_p = yaw_p + 0.5*nu_yawdd*delta_t*delta_t;
        yawd_p = yawd_p + nu_yawdd*delta_t;
        
        //write predicted sigma point into right column
        Xsig_pred_(0,i) = px_p;
        Xsig_pred_(1,i) = py_p;
        Xsig_pred_(2,i) = v_p;
        Xsig_pred_(3,i) = yaw_p;
        Xsig_pred_(4,i) = yawd_p;
    }
    
    // Step 4: Get the predicted state (x) and covariance (P) from those sigma points
    
    // set weights
    double weight_0 = lambda_/(lambda_+n_aug_);
    weights_(0) = weight_0;
    for (int i=1; i<2*n_aug_+1; i++) {  //2n+1 weights
        double weight = 0.5/(n_aug_+lambda_);
        weights_(i) = weight;
    }
    
    x_.fill(0.0);
    //predicted state mean
    for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //iterate over sigma points
        x_ = x_+ weights_(i) * Xsig_pred_.col(i);
    }
    
    P_.fill(0.0);
    //predicted state covariance matrix
    for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //iterate over sigma points
        
        // state difference
        VectorXd x_diff = Xsig_pred_.col(i) - x_;
        //angle normalization
        while (x_diff(3)> M_PI) x_diff(3)-=2.*M_PI;
        while (x_diff(3)<-M_PI) x_diff(3)+=2.*M_PI;
        
        P_ = P_ + weights_(i) * x_diff * x_diff.transpose() ;
    }
    
    
    
}

/**
 * Updates the state and the state covariance matrix using a laser measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateLidar(MeasurementPackage meas_package) {
    /**
     TODO:
     
     Complete this function! Use lidar data to update the belief about the object's
     position. Modify the state vector, x_, and covariance, P_.
     
     You'll also need to calculate the lidar NIS.
     */
    int n_z = 2;  // we only have position x and position y for lidar measurements
    
    MatrixXd Zsig = MatrixXd(n_z, 2 * n_aug_ + 1);
    
    
    // Step 1: Transform predicted state from CTRV space to lidar space
    //transform sigma points into measurement space
    for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //2n+1 simga points
        
        // extract values for better readibility
        double p_x = Xsig_pred_(0,i);
        double p_y = Xsig_pred_(1,i);
        
        // measurement model
        Zsig(0,i) = p_x;                                //px
        Zsig(1,i) = p_y;                                //py
        
    }
    
    //mean predicted measurement
    VectorXd z_pred = VectorXd(n_z);
    z_pred.fill(0.0);
    for (int i=0; i < 2*n_aug_+1; i++) {
        z_pred = z_pred + weights_(i) * Zsig.col(i);
    }
    
    //measurement covariance matrix S
    MatrixXd S = MatrixXd(n_z,n_z);
    S.fill(0.0);
    for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //2n+1 simga points
        //residual
        VectorXd z_diff = Zsig.col(i) - z_pred;
        
        // there is no normalization going on for lidar measurements
        
        S = S + weights_(i) * z_diff * z_diff.transpose();
    }
    
    //add measurement noise covariance matrix
    MatrixXd R = MatrixXd(n_z,n_z);
    R <<    std_laspx_ * std_laspx_, 0,
    0, std_laspy_ * std_laspy_;
    
    S = S + R;
    
    
    
    
    // Step 2: Update the UKF using the lidar measurement
    MatrixXd Tc = MatrixXd(n_x_, n_z);
    //calculate cross correlation matrix
    Tc.fill(0.0);
    for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //2n+1 simga points
        
        //residual
        VectorXd z_diff = Zsig.col(i) - z_pred;
        // no angle normalization
        
        
        // state difference
        VectorXd x_diff = Xsig_pred_.col(i) - x_;
        // no angle normalization
        
        
        Tc = Tc + weights_(i) * x_diff * z_diff.transpose();
    }
    
    //Kalman gain K;
    MatrixXd K = Tc * S.inverse();
    
    // get the measurement
    VectorXd z = meas_package.raw_measurements_;
    
    //residual
    VectorXd z_diff = z - z_pred;
    
    // no angle normalization
    
    
    //update state mean and covariance matrix
    x_ = x_ + K * z_diff;
    P_ = P_ - K*S*K.transpose();
    

    
}

/**
 * Updates the state and the state covariance matrix using a radar measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateRadar(MeasurementPackage meas_package) {
    /**
     TODO:
     
     Complete this function! Use radar data to update the belief about the object's
     position. Modify the state vector, x_, and covariance, P_.
     
     You'll also need to calculate the radar NIS.
     */
    int n_z = 3;
    MatrixXd Zsig = MatrixXd(n_z, 2 * n_aug_ + 1);
    
    // Step 1: Transform predicted state from CTRV space to radar space
    //transform sigma points into measurement space
    for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //2n+1 simga points
        
        // extract values for better readibility
        double p_x = Xsig_pred_(0,i);
        double p_y = Xsig_pred_(1,i);
        double v  = Xsig_pred_(2,i);
        double yaw = Xsig_pred_(3,i);
        
        double v1 = cos(yaw)*v;
        double v2 = sin(yaw)*v;
        
        // measurement model
        Zsig(0,i) = sqrt(p_x*p_x + p_y*p_y);                        //r
        Zsig(1,i) = atan2(p_y,p_x);                                 //phi
        Zsig(2,i) = (p_x*v1 + p_y*v2 ) / sqrt(p_x*p_x + p_y*p_y);   //r_dot
    }
    
    //mean predicted measurement
    VectorXd z_pred = VectorXd(n_z);
    z_pred.fill(0.0);
    for (int i=0; i < 2*n_aug_+1; i++) {
        z_pred = z_pred + weights_(i) * Zsig.col(i);
    }
    
    //measurement covariance matrix S
    MatrixXd S = MatrixXd(n_z,n_z);
    S.fill(0.0);
    for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //2n+1 simga points
        //residual
        VectorXd z_diff = Zsig.col(i) - z_pred;
        
        //angle normalization
        while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
        while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;
        
        S = S + weights_(i) * z_diff * z_diff.transpose();
    }
    
    //add measurement noise covariance matrix
    MatrixXd R = MatrixXd(n_z,n_z);
    R <<    std_radr_*std_radr_, 0, 0,
    0, std_radphi_*std_radphi_, 0,
    0, 0,std_radrd_*std_radrd_;
    S = S + R;
    
    // Step 2: Update the UKF using the radar measurement
    MatrixXd Tc = MatrixXd(n_x_, n_z);
    //calculate cross correlation matrix
    Tc.fill(0.0);
    for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //2n+1 simga points
        
        //residual
        VectorXd z_diff = Zsig.col(i) - z_pred;
        //angle normalization
        while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
        while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;
        
        // state difference
        VectorXd x_diff = Xsig_pred_.col(i) - x_;
        //angle normalization
        while (x_diff(3)> M_PI) x_diff(3)-=2.*M_PI;
        while (x_diff(3)<-M_PI) x_diff(3)+=2.*M_PI;
        
        Tc = Tc + weights_(i) * x_diff * z_diff.transpose();
    }
    
    //Kalman gain K;
    MatrixXd K = Tc * S.inverse();
    
    // get the measurement
    VectorXd z = meas_package.raw_measurements_;
    
    //residual
    VectorXd z_diff = z - z_pred;
    
    //angle normalization
    while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
    while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;
    
    //update state mean and covariance matrix
    x_ = x_ + K * z_diff;
    P_ = P_ - K*S*K.transpose();
    
    
    
}
