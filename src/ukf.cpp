#include "ukf.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/**
 * Initializes Unscented Kalman filter
 * This is scaffolding, do not modify
 */
UKF::UKF() {
  // if this is false, laser measurements will be ignored (except during init)
  use_laser_ = false;

  // if this is false, radar measurements will be ignored (except during init)
  use_radar_ = true;

  // initial state vector
  x_ = VectorXd(5);

  // initial covariance matrix
  P_ = MatrixXd(5, 5);

  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ = 30;

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 30;
  
  //DO NOT MODIFY measurement noise values below these are provided by the sensor manufacturer.
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
  //DO NOT MODIFY measurement noise values above these are provided by the sensor manufacturer.
  
  /**
  TODO:

  Complete the initialization. See ukf.h for other member properties.

  Hint: one or more values initialized above might be wildly off...
  */
  is_initialized_ = false;

  n_x_ = 5;

  n_aug_ = 7;

  lambda_ = 3 - n_aug_;

  Xsig_pred_ = MatrixXd(5, 2*n_aug_);

  weights_ = VectorXd(2*n_aug_+1);

  // Build weights
  weights_(0) = lambda_/(lambda_ + n_aug_);
  double dummy = 0.5/(lambda_ + n_aug_);
  for(int i = 1; i < 2*n_aug_+1; ++i)
  {
    weights_(i) = dummy;
  }
  
  // Build R_lidar_
  R_lidar_ = MatrixXd(2, 2);
  R_lidar_ << std_laspx_*std_laspx_, 0,
              0, std_laspy_*std_laspy_;

  // Build R_radar_
  R_radar_ = MatrixXd(3, 3);
  R_radar_ <<  std_radr_*std_radr_, 0, 0,
                0, std_radphi_*std_radphi_, 0,
                0, 0, std_radrd_*std_radrd_;     

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
  if(!is_initialized_)
  {
    cout<<"init"<<endl;
    if(use_laser_ && (meas_package.sensor_type_= MeasurementPackage::LASER))
    {
        cout<<"laser init"<<endl;
        x_ << meas_package.raw_measurements_(0),
              meas_package.raw_measurements_(1),
              0,
              0,
              0;

    }
    if(use_radar_ && (meas_package.sensor_type_ = MeasurementPackage::RADAR))
    {
        cout<<"radar init"<<endl;
        double rho = meas_package.raw_measurements_(0);
        double phi = meas_package.raw_measurements_(1);
        x_ << rho * cos(phi),
             rho * sin(phi),
             0,
             0,
             0;
    }

    is_initialized_ = true;
    time_us_ = meas_package.timestamp_;
    cout<<"init is completed"<<endl;
  }
  else
  {
    if(use_laser_ && (meas_package.sensor_type_ = MeasurementPackage::LASER))
    {
        cout<<"laser measurement"<<endl;
        double dt = meas_package.timestamp_ - time_us_;
        dt /= 1000000;
        time_us_ = meas_package.timestamp_;      // update time right away
        cout<< "dt="<<dt<<endl;

        Prediction(dt);
        UpdateLidar(meas_package);
    }
    if(use_radar_ && (meas_package.sensor_type_ = MeasurementPackage::RADAR))
    {
        cout<<"radar measurement"<<endl;
        double dt = meas_package.timestamp_ - time_us_;
        dt /= 1000000;
        time_us_ = meas_package.timestamp_;      // update time right away
        cout<< "dt="<<dt<<endl;

        Prediction(dt);
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
  
  MatrixXd Xsig_aug = MatrixXd(7, 2*n_aug_+1);
  GenerateAugSigmaPoints(Xsig_aug);
  PredictSigmaPonts(Xsig_aug, delta_t);
  cout<<"prediction: Xsig_pred_ =\n"<<Xsig_pred_<<endl;

  //Predict the mean and covariance with sigma points
  x_.fill(0.0);
  for(int i = 0; i < 2*n_aug_+1; ++i)
  {
    x_ += weights_(i) * Xsig_pred_.col(i);
  }
  cout<< "prediction: x_ =\n"<<x_<<endl;

  P_.fill(0.0);
  for(int i = 0; i < 2*n_aug_+1; ++i)
  {
    VectorXd x_diff = Xsig_pred_.col(i) - x_;
    // Normalized the yaw angle
    while(x_diff(3) > M_PI) x_diff(3) -= 2.0 * M_PI;
    while(x_diff(3) < -M_PI) x_diff(3) += 2.0 * M_PI;
  
    P_ += weights_(i) * x_diff * x_diff.transpose();
  }
  cout<< "prediction: P_ =\n"<<P_<<endl;
}


/////////////////////////////////////////////////////////////////////////////////////////

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
}

/////////////////////////////////////////////////////////////////////////////////////////

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
  // BUild Zxig
  VectorXd z_pred = VectorXd(3);
  MatrixXd Zxig_pred = MatrixXd(3, 2*n_aug_+1); // radar measurement dim = 3 
  MatrixXd S = MatrixXd(3, 3);
  MatrixXd T = MatrixXd(5, 3); // cross-correlation between sigma points in state space and measurement space
  MatrixXd K = MatrixXd(5, 3); // Kalman gain

  // Calculate Zxig_pred
  // construct from Xsig_pred_
  for(int i = 0; i < 2*n_aug_+1; ++i)
  {

    double px = Xsig_pred_(0, i);
    double py = Xsig_pred_(1, i);
    double v = Xsig_pred_(2, i);
    double yaw = Xsig_pred_(3, i);

    double rho = sqrt(px*px + py*py);
    double phi = atan2(py, px);
    double rho_rate(0);
    if(rho < 0.001)
    {
      rho_rate = numeric_limits<double>::infinity();
    }
    else
    {
      rho_rate = (px*cos(yaw)*v + py*sin(yaw)*v) / rho;
    }

    Zxig_pred.col(i) << rho,
                   phi, 
                   rho_rate;
  }
  cout<< "UpdateRadar: Zxig_pred=\n"<<Zxig_pred<<endl;

  // Calculate z_pred
  z_pred.fill(0.0);
  for(int i = 0; i < 2*n_aug_+1; ++i)
  {
    z_pred += weights_(i) * Zxig_pred.col(i);
  }
  cout<< "UpdateRadar: z_pred=\n"<<z_pred<<endl;

  // Calculate S
  S.fill(0.0);
  for(int i = 0; i< 2*n_aug_+1; ++i)
  {
    VectorXd z_diff = Zxig_pred.col(i) - z_pred;

    // Normalized the angle
    while(z_diff(2) > M_PI) z_diff(2) -= 2 * M_PI;
    while(z_diff(2) < -M_PI) z_diff(2) += 2 * M_PI;


    S += weights_(i) * z_diff * z_diff.transpose();
  }
  S += R_radar_;
  cout<< "UpdateRadar: S=\n"<<S<<endl;

  // Calculate T
  T.fill(0.0);
  for(int i = 0; i < 2*n_aug_+1; ++i)
  {
    VectorXd x_diff = Xsig_pred_.col(i) - x_;

    // Normalized
    while(x_diff(3) > M_PI) x_diff(3) -= 2.0 * M_PI;
    while(x_diff(3) < -M_PI) x_diff(3) += 2.0 * M_PI;

    VectorXd z_diff = Zxig_pred.col(i) - z_pred;
    // Normalized the angle
    while(z_diff(2) > M_PI) z_diff(2) -= 2 * M_PI;
    while(z_diff(2) < -M_PI) z_diff(2) += 2 * M_PI;

    T += weights_(i) * x_diff * z_diff.transpose();
  }
  cout<< "UpdateRadar: T=\n"<<T<<endl;

  // Calculate K
  K = T * S.inverse();
  cout<< "UpdateRadar: K=\n"<<K<<endl;

  // Update
  x_ = x_ + K * (meas_package.raw_measurements_ - z_pred);
  P_ = P_ - K * S * K.transpose();

  cout<< "UpdateRadar: complete x=\n"<<x_<<endl;
  cout<< "UpdateRadar: complete P=\n"<<P_<<endl;

}
/////////////////////////////////////////////////////////////////////////////////////////
void UKF::GenerateAugSigmaPoints(Eigen::MatrixXd& Xsig_aug)
{
  cout<<"construct the augmented state"<<endl;
  VectorXd x_aug = VectorXd(7); 
  x_aug.head(5) = x_; 
  x_aug(6) = 0;
  x_aug(7) = 0;

  cout<<"construct Q"<<endl;
  MatrixXd Q = MatrixXd(2,2);
  Q << std_a_*std_a_, 0,
       0, std_yawdd_*std_yawdd_;
    
  cout<<"construct P_aug"<<endl;
  MatrixXd P_aug = MatrixXd(7, 2*n_aug_+1);
  P_aug.topLeftCorner(5,5) = P_;
  P_aug.bottomRightCorner(2,2) = Q;
  cout<<"P_aug \n"<<P_aug<<endl;

  MatrixXd A = P_aug.llt().matrixL();
  cout<<"A \n"<<A<<endl;
  double s = sqrt(lambda_ + n_aug_);

  cout<<"construct Xsig_aug"<<endl;
  Xsig_aug.col(0) = x_aug;
  for(int i = 1; i <= n_aug_; ++i)
  {
    Xsig_aug.col(i) = x_aug + s * A.col(i-1);
    Xsig_aug.col(i+n_aug_-1) = x_aug - s * A.col(i-1);
  }
  cout<<"Xsig_aug \n"<<Xsig_aug<<endl;
}

/////////////////////////////////////////////////////////////////////////////////////////
void UKF::PredictSigmaPonts(const Eigen::MatrixXd& Xsig_aug, const double& delta_t)
{
  for(int i = 0; i < 2*n_aug_+1 ; ++i)
  {
    VectorXd T = VectorXd(5); 
    VectorXd V = VectorXd(5);
    VectorXd x = Xsig_aug.col(i);
    BuildTransitionVec(x, delta_t, T);
    BUildNoiseVec(x, delta_t, V);
    Xsig_pred_.col(i) = x.head(5) + T + V; // write to the member variable
  }
}
/////////////////////////////////////////////////////////////////////////////////////////
void UKF::BuildTransitionVec(const VectorXd& in, const double& delta_t, VectorXd& out)
{
  double px       = in(0);
  double py       = in(1);
  double v        = in(2);
  double yaw      = in(3);
  double yaw_rate = in(4);

  // check yaw_rate
  if(yaw_rate < 0.001)
  {
    cout<<"[Warning]: yaw_rate is nearly 0"<<endl;
    out << v * cos(yaw) * delta_t,
           v * sin(yaw) * delta_t,
           0,
           yaw_rate * delta_t,
           0;
  }
  else
  {
    double angle = yaw + yaw_rate * delta_t;
    out << (v / yaw_rate) * (sin(angle) - sin(yaw)),
           (v / yaw_rate) * (-cos(angle) + cos(yaw)),
           0,
           yaw_rate * delta_t,
           0;
  }
}

/////////////////////////////////////////////////////////////////////////////////////////
void UKF::BUildNoiseVec(const VectorXd& in, const double& delta_t, VectorXd& out)
{
  double px       = in(0);
  double py       = in(1);
  double v        = in(2);
  double yaw      = in(3);
  double yaw_rate = in(4);
  double acc_noise = in(5);
  double yaw_rate_noise = in(6);
  double dt2 = delta_t * delta_t;

  out << 0.5 * dt2 * cos(yaw) * acc_noise,
         0.5 * dt2 * sin(yaw) * acc_noise,
         delta_t * acc_noise,
         0.5 * dt2 * yaw_rate_noise,
         delta_t * yaw_rate_noise;
}
