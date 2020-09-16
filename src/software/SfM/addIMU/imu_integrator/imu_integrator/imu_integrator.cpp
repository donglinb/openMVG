#include <iostream>

#include "imu_integrator.h"

namespace IMUDynamics
{

IMUIntegrator::IMUIntegrator()
{
    std::cout<<"IMU Integrator created with default constructor."<<std::endl;
    imu_queue_.clear();
    imu_calibrated_ = false;
}

IMUIntegrator::IMUIntegrator(imuState<float>& init_imu_state_)
{
    std::cout<<"IMU Integrator created with initial state."<<std::endl;
    this->init_imu_state_ = init_imu_state_;
    setup_msckf();
    imu_queue_.clear();
    imu_calibrated_ = true;
}

void IMUIntegrator::push_back(imuReading<float>& current_imu, double cur_imu_time)
{
    if(imu_calibrated_)
    {
        msckf_.propagate(current_imu);
    }
    else
    {
        imu_queue_.emplace_back(cur_imu_time, current_imu);
        if(imu_queue_.size()>=200)
        {
            initialize_imu();
            setup_msckf();
            imu_queue_.clear();
            imu_calibrated_ = true;
        }
    }
}

void IMUIntegrator::initialize_imu()
{
    Eigen::Vector3f accel_accum;
    Eigen::Vector3f gyro_accum;
    int num_readings = 0;

    accel_accum.setZero();
    gyro_accum.setZero();

    for(const auto& entry : imu_queue_)
    {
        auto imu_time = std::get<0>(entry);
        auto imu_reading = std::get<1>(entry);

        accel_accum += imu_reading.a;
        gyro_accum += imu_reading.omega;
        num_readings++;
    }

    Eigen::Vector3f accel_mean = accel_accum / num_readings;
    Eigen::Vector3f gyro_mean = gyro_accum / num_readings;

    init_imu_state_.b_g = gyro_mean;
    init_imu_state_.g << 0.0, 0.0, -9.81;
    init_imu_state_.q_IG = msckf_mono::Quaternion<float>::FromTwoVectors(
        -init_imu_state_.g, accel_mean);

    init_imu_state_.b_a = init_imu_state_.q_IG*init_imu_state_.g + accel_mean;

    init_imu_state_.p_I_G.setZero();
    init_imu_state_.v_I_G.setZero();
    
    const auto q = init_imu_state_.q_IG;

    std::cout<<"\nInitial IMU State" <<
      "\n--p_I_G " << init_imu_state_.p_I_G.transpose() <<
      "\n--q_IG " << q.w() << "," << q.x() << "," << q.y() << "," << q.x() <<
      "\n--v_I_G " << init_imu_state_.v_I_G.transpose() <<
      "\n--b_a " << init_imu_state_.b_a.transpose() <<
      "\n--b_g " << init_imu_state_.b_g.transpose() <<
      "\n--g " << init_imu_state_.g.transpose()<<std::endl;
}

void IMUIntegrator::setup_msckf()
{
    // TODO: reading the parameters from external yaml config file, or remove this part.
    
    camera_.f_u = 457.587;
    camera_.f_v = 456.134;
    camera_.c_u = 379.999;
    camera_.c_v = 255.238;
    camera_.q_CI.setIdentity();
    camera_.p_C_I.setZero();
    
    float feature_cov = 2.0;

    Eigen::Matrix<float,12,1> Q_imu_vars;
    float w_var = 1e-4, dbg_var = 3.6733e-5, a_var = 1e-2, dba_var = 7e-2;
    Q_imu_vars << w_var, 	w_var, 	w_var,
                  dbg_var,dbg_var,dbg_var,
                  a_var,	a_var,	a_var,
                  dba_var,dba_var,dba_var;

    Eigen::Matrix<float,15,1> IMUCovar_vars;
    float q_var_init = 1e-5, bg_var_init = 1e-2, v_var_init = 1e-2, ba_var_init = 1e-2, p_var_init = 1e-12;
    IMUCovar_vars << q_var_init, q_var_init, q_var_init,
                     bg_var_init,bg_var_init,bg_var_init,
                     v_var_init, v_var_init, v_var_init,
                     ba_var_init,ba_var_init,ba_var_init,
                     p_var_init, p_var_init, p_var_init;
                     
    // Setup noise parameters
    noise_params_.initial_imu_covar = IMUCovar_vars.asDiagonal();
    noise_params_.Q_imu = Q_imu_vars.asDiagonal();
    noise_params_.u_var_prime = pow(feature_cov/camera_.f_u,2);
    noise_params_.v_var_prime = pow(feature_cov/camera_.f_v,2);
    
    msckf_params_.max_gn_cost_norm = 7;
    msckf_params_.max_gn_cost_norm = pow(msckf_params_.max_gn_cost_norm/camera_.f_u, 2);
    msckf_params_.translation_threshold = 0.1;
    msckf_params_.min_rcond = 3e-12;
    msckf_params_.redundancy_angle_thresh = 0.005;
    msckf_params_.redundancy_distance_thresh = 0.05;
    msckf_params_.max_track_length = 50;
    msckf_params_.min_track_length = 5;
    msckf_params_.max_cam_states = 30;
    
    msckf_.initialize(camera_, noise_params_, msckf_params_, init_imu_state_);
}

Eigen::Vector3f IMUIntegrator::getPosition()
{
    imuState<float> state = msckf_.getImuState();
    return state.p_I_G;
}

Eigen::Vector4f IMUIntegrator::getOrientation()
{
    imuState<float> state = msckf_.getImuState();
    return state.q_IG.coeffs();
}

imuState<float> IMUIntegrator::getIMUState()
{
    return msckf_.getImuState();
}

}  // namespace msckf_mono
