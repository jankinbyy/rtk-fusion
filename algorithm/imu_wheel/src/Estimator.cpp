#include "Estimator.h"

Estimator::Estimator(const double gyro_noise, const double gyro_bias_noise, const double acc_noise)
    : status_(Status::kInvalid),
      last_timestamp_(-1.),
      initializer_(std::make_unique<Initializer>()),
      propagator_(std::make_unique<Propagator>(gyro_noise, gyro_bias_noise))
{
	acc_noise_mat_ = Eigen::Matrix3d::Identity() * acc_noise;
}

void Estimator::clear_state_ori()
{
	acc_buffer_.clear();
	status_ = Status::kInvalid;
	G_R_I_ = Eigen::Matrix3d::Identity();
	last_timestamp_ = -1;
	bg_.setZero();
	cov_.setZero();
	cov_.topLeftCorner<3, 3>() = Eigen::Matrix3d::Identity() * 0.5 * 0.5 * kDeg2Rad * kDeg2Rad;
	cov_.bottomRightCorner<3, 3>() = Eigen::Matrix3d::Identity() * 0.1 * 0.1 * kDeg2Rad * kDeg2Rad;
}

bool Initializer::Initialize(const Eigen::Vector3d &acc, Eigen::Matrix3d *G_R_I)
{
	acc_buffer_.push_back(acc);
	if (acc_buffer_.size() <= config_.acc_buffer_size) {
		return false;
	}
	acc_buffer_.pop_front();
	Eigen::Vector3d mean_acc(0., 0., 0.);
	for (const Eigen::Vector3d &one_acc : acc_buffer_) {
		mean_acc += one_acc;
	}
	mean_acc = mean_acc / static_cast<double>(acc_buffer_.size());
	// Compute std acc.
	Eigen::Vector3d std_acc(0., 0., 0.);
	for (const Eigen::Vector3d &one_acc : acc_buffer_) {
		std_acc += (one_acc - mean_acc).cwiseAbs2();
	}
	std_acc = (std_acc / static_cast<double>(acc_buffer_.size())).cwiseSqrt();
	if (std_acc.norm() > config_.max_acc_std) {
		std::cout << "[Initialize]: Initializaion failed. Too big acc std: " << std::fixed
			  << std_acc.transpose();
		return false;
	}
	// Get initial orientaion.
	const Eigen::Vector3d z_axis = mean_acc.normalized();
	const Eigen::Vector3d x_axis =
	    (Eigen::Vector3d::UnitX() - z_axis * z_axis.transpose() * Eigen::Vector3d::UnitX()).normalized();
	const Eigen::Vector3d y_axis = (z_axis.cross(x_axis)).normalized();

	Eigen::Matrix3d I_R_G;
	I_R_G.block<3, 1>(0, 0) = x_axis;
	I_R_G.block<3, 1>(0, 1) = y_axis;
	I_R_G.block<3, 1>(0, 2) = z_axis;

	*G_R_I = I_R_G.transpose();

	return true;
}
Propagator::Propagator(const double &gyro_noise, const double &gyro_bias_noise)
    : gyro_noise_(gyro_noise), gyro_bias_noise_(gyro_bias_noise)
{
}

void Propagator::PropagateMean(const Eigen::Matrix3d &begin_G_R_I, const Eigen::Vector3d &gyro, const double delta_t,
			       Eigen::Matrix3d &end_G_R_I)
{
	const Eigen::Vector3d unbiased_gyro = gyro - Eigen::Vector3d::Zero();
	const Eigen::Vector3d angle_vec = unbiased_gyro * delta_t;
	Eigen::Matrix3d delta_rot;
	if (angle_vec.norm() < 1e-12) {
		delta_rot = Eigen::Quaterniond(Eigen::Matrix3d::Identity() + SkewMat(angle_vec))
				.normalized()
				.toRotationMatrix();
	} else {
		const double angle = angle_vec.norm();
		const Eigen::Vector3d axis = angle_vec / angle;
		delta_rot = Eigen::AngleAxisd(angle, axis).toRotationMatrix();
	}
	end_G_R_I = begin_G_R_I * delta_rot;
}

void Propagator::EstimateByAcc(const Eigen::Vector3d &acc, Eigen::Matrix3d &end_G_R_I)
{

	double roll = atan2(acc(1), acc(2));
	double pitch = -atan2(acc(0), sqrt(pow(acc(1), 2) + pow(acc(2), 2)));
	double yaw = 0.0;
	Eigen::AngleAxisd yawAngle(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()));
	Eigen::AngleAxisd pitchAngle(Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()));
	Eigen::AngleAxisd rollAngle(Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()));
	end_G_R_I = yawAngle * pitchAngle * rollAngle;
}

//龙格库塔
void Propagator::PropagateRK(const Eigen::Matrix3d &begin_G_R_I, const Eigen::Vector3d &gyro,
			     const Eigen::Vector3d &last_gyro, const double delta_t, Eigen::Matrix3d &end_G_R_I)
{
	Eigen::Quaterniond quaternion_self_last(begin_G_R_I);
	double dif_quarterion_f[4];
	double dif_quarterion_l[4];
	double med_quarterion[4];

	dif_quarterion_f[0] = (-quaternion_self_last.x() * last_gyro[0] - quaternion_self_last.y() * last_gyro[1] -
			       quaternion_self_last.z() * last_gyro[2]) *
			      0.5f;
	dif_quarterion_f[1] = (quaternion_self_last.w() * last_gyro[0] + quaternion_self_last.y() * last_gyro[2] -
			       quaternion_self_last.z() * last_gyro[1]) *
			      0.5f;
	dif_quarterion_f[2] = (quaternion_self_last.w() * last_gyro[1] - quaternion_self_last.x() * last_gyro[2] +
			       quaternion_self_last.z() * last_gyro[0]) *
			      0.5f;
	dif_quarterion_f[3] = (quaternion_self_last.w() * last_gyro[2] + quaternion_self_last.x() * last_gyro[1] -
			       quaternion_self_last.y() * last_gyro[0]) *
			      0.5f;

	med_quarterion[0] = quaternion_self_last.w() + dif_quarterion_f[0] * delta_t;
	med_quarterion[1] = quaternion_self_last.x() + dif_quarterion_f[1] * delta_t;
	med_quarterion[2] = quaternion_self_last.y() + dif_quarterion_f[2] * delta_t;
	med_quarterion[3] = quaternion_self_last.z() + dif_quarterion_f[3] * delta_t;

	dif_quarterion_l[0] = (-med_quarterion[1] * gyro[0] - med_quarterion[2] * gyro[1] -
			       med_quarterion[3] * gyro[2]) *
			      0.5f;
	dif_quarterion_l[1] = (med_quarterion[0] * gyro[0] + med_quarterion[2] * gyro[2] -
			       med_quarterion[3] * gyro[1]) *
			      0.5f;
	dif_quarterion_l[2] = (med_quarterion[0] * gyro[1] - med_quarterion[1] * gyro[2] +
			       med_quarterion[3] * gyro[0]) *
			      0.5f;
	dif_quarterion_l[3] = (med_quarterion[0] * gyro[2] + med_quarterion[1] * gyro[1] -
			       med_quarterion[2] * gyro[0]) *
			      0.5f;

	Eigen::Quaterniond quaternion;
	quaternion.w() = quaternion_self_last.w() + 0.5f * (dif_quarterion_f[0] + dif_quarterion_l[0]) * delta_t;
	quaternion.x() = quaternion_self_last.x() + 0.5f * (dif_quarterion_f[1] + dif_quarterion_l[1]) * delta_t;
	quaternion.y() = quaternion_self_last.y() + 0.5f * (dif_quarterion_f[2] + dif_quarterion_l[2]) * delta_t;
	quaternion.z() = quaternion_self_last.z() + 0.5f * (dif_quarterion_f[3] + dif_quarterion_l[3]) * delta_t;
	end_G_R_I = quaternion.matrix();
}

IMUData preIntegrate(const IMUData &imu_in, const std::shared_ptr<Estimator> &orientation_estimator)
{
	IMUData imuSelf = imu_in;
	Eigen::Matrix3d G_R_I;
	const double timestamp = imu_in.time;
	Eigen::Vector3d acc(imu_in.linear_acceleration.x, imu_in.linear_acceleration.y, imu_in.linear_acceleration.z);
	Eigen::Vector3d gyro(imu_in.angular_velocity.x, imu_in.angular_velocity.y, imu_in.angular_velocity.z);
	imuSelf.angular_velocity.x = gyro(0);
	imuSelf.angular_velocity.y = gyro(1);
	imuSelf.angular_velocity.z = gyro(2);
	imuSelf.linear_acceleration.x = acc(0);
	imuSelf.linear_acceleration.y = acc(1);
	imuSelf.linear_acceleration.z = acc(2);
	Status status = orientation_estimator->Estimate(timestamp, gyro, acc, &G_R_I);
	Eigen::Quaterniond quat(G_R_I);
	imuSelf.orientation.Construct(quat);
	return imuSelf;
}
Status Estimator::Estimate(double timestamp, const Eigen::Vector3d &gyro, const Eigen::Vector3d &acc,
			   Eigen::Matrix3d *G_R_I)
{
	if (status_ == Status::kInvalid) {
		if (!initializer_->Initialize(acc, &G_R_I_)) {
			G_R_I->setIdentity();
			return Status::kInvalid;
		}

		bg_.setZero();
		cov_.setZero();
		cov_.topLeftCorner<3, 3>() = Eigen::Matrix3d::Identity() * 0.5 * 0.5 * kDeg2Rad * kDeg2Rad;
		cov_.bottomRightCorner<3, 3>() = Eigen::Matrix3d::Identity() * 0.1 * 0.1 * kDeg2Rad * kDeg2Rad;
		last_timestamp_ = timestamp;

		// Send out.
		*G_R_I = G_R_I_;

		status_ = Status::kValid;
		return Status::kValid;
	}

	double delta_t = timestamp - last_timestamp_;
	last_timestamp_ = timestamp;

	// Propagation.
	Eigen::Matrix3d prior_G_R_I;
	Eigen::Vector3d prior_bg;
	Eigen::Matrix<double, 6, 6> prior_cov;
	propagator_->PropagateMeanAndCov(G_R_I_, bg_, cov_, gyro, delta_t, &prior_G_R_I, &prior_bg,
					 &prior_cov);  //根据输入的角速度，估计

	acc_buffer_.push_back(acc);
	if (acc_buffer_.size() > config_.acc_buffer_size) {
		acc_buffer_.pop_front();
	}
	// compute mean.
	Eigen::Vector3d mean_acc(0., 0., 0.);
	for (const Eigen::Vector3d &one_acc : acc_buffer_) {
		mean_acc += one_acc;
	}
	mean_acc = mean_acc / static_cast<double>(acc_buffer_.size());

	// Update
	Update(prior_G_R_I, prior_bg, prior_cov, mean_acc, acc_noise_mat_, &G_R_I_, &bg_, &cov_);  //根据加速度更新

	// Send out.
	*G_R_I = G_R_I_;

	return Status::kValid;
}
// https://github.com/yuzhou42/ESKF-Attitude-Estimation/blob/master/ESKF%20Attitude%20Algorithm.pdf
void Propagator::PropagateMeanAndCov(const Eigen::Matrix3d &begin_G_R_I, const Eigen::Vector3d &begin_bg,
				     const Eigen::Matrix<double, 6, 6> &begin_cov, const Eigen::Vector3d &gyro,
				     const double delta_t, Eigen::Matrix3d *end_G_R_I, Eigen::Vector3d *end_bg,
				     Eigen::Matrix<double, 6, 6> *end_cov)
{
	// Mean propagation.
	const Eigen::Vector3d unbiased_gyro = gyro - begin_bg;
	const Eigen::Vector3d angle_vec =
	    unbiased_gyro * delta_t;  //（wm-wb）*delta_t quaternion kinematics for the error-state KF p60 261c
	Eigen::Matrix3d delta_rot;
	if (angle_vec.norm() < 1e-12) {
		delta_rot = Eigen::Quaterniond(Eigen::Matrix3d::Identity() + SkewMat(angle_vec))
				.normalized()
				.toRotationMatrix();
	} else {
		const double angle = angle_vec.norm();
		const Eigen::Vector3d axis = angle_vec / angle;
		delta_rot = Eigen::AngleAxisd(angle, axis).toRotationMatrix();  // Rt
	}
	*end_G_R_I = begin_G_R_I * delta_rot;
	*end_bg = begin_bg;

	// Jacobian.  [角度的误差状态 角速度的误差状态]
	Eigen::Matrix<double, 6, 6> Fx;
	Fx.topLeftCorner<3, 3>() = delta_rot.transpose();		     // Rt{(wm-wb)deltaT}
	Fx.topRightCorner<3, 3>() = -Eigen::Matrix3d::Identity() * delta_t;  // deltaT
	Fx.bottomLeftCorner<3, 3>() = Eigen::Matrix3d::Zero();
	Fx.bottomRightCorner<3, 3>() = Eigen::Matrix3d::Identity();

	Eigen::Matrix<double, 6, 6> Q = Eigen::Matrix<double, 6, 6>::Zero();
	Q.topLeftCorner<3, 3>() = Eigen::Matrix3d::Identity() * gyro_noise_ * delta_t * delta_t;
	Q.bottomRightCorner<3, 3>() = Eigen::Matrix3d::Identity() * gyro_bias_noise_ * delta_t;

	*end_cov = Fx * begin_cov * Fx.transpose() + Q;
}
void Update(const Eigen::Matrix3d &prior_G_R_I, const Eigen::Vector3d &prior_bg,
	    const Eigen::Matrix<double, 6, 6> &prior_cov, const Eigen::Vector3d &acc, const Eigen::Matrix3d &acc_noise,
	    Eigen::Matrix3d *posterior_G_R_I, Eigen::Vector3d *posterior_bg, Eigen::Matrix<double, 6, 6> *posterior_cov)
{
	// Residual
	Eigen::Vector3d gravity_vec(0., 0., 1.);
	Eigen::Vector3d residual =
	    acc.normalized() -
	    prior_G_R_I.transpose() *
		gravity_vec;  // v=y-h(xt) imu加速度姿态校正，假设无机动加速度，只有重力加速度，y=am=an-R*g

	// Jacobian.
	Eigen::Matrix<double, 3, 6> H;  // R|w
	H.setZero();
	H.block<3, 3>(0, 0) = SkewMat(prior_G_R_I.transpose() * gravity_vec);

	// Kalman gain.
	Eigen::Matrix<double, 6, 3> K = prior_cov * H.transpose() *
					(H * prior_cov * H.transpose() + acc_noise).inverse();

	// Delta x.
	Eigen::Matrix<double, 6, 1> delta_x = K * residual;  // delta_x=K*(y-h(xt))

	// Update state.
	Eigen::Matrix3d delta_Rot;
	if (delta_x.topRows<3>().norm() < 1e-12) {
		delta_Rot = Eigen::Quaterniond(Eigen::Matrix3d::Identity() + SkewMat(delta_x.topRows<3>()))
				.normalized()
				.toRotationMatrix();
	} else {
		const double angle = delta_x.topRows<3>().norm();
		const Eigen::Vector3d axis = delta_x.topRows<3>() / angle;      // delta x单位化
		delta_Rot = Eigen::AngleAxisd(angle, axis).toRotationMatrix();  //绕轴旋转
	}
	*posterior_G_R_I = prior_G_R_I * delta_Rot;	  //求旋转
	*posterior_bg = prior_bg + delta_x.bottomRows<3>();  //及bg

	// Update covariance.
	const Eigen::Matrix<double, 6, 6> I_mins_KH = Eigen::Matrix<double, 6, 6>::Identity() - K * H;
	*posterior_cov = I_mins_KH * prior_cov * I_mins_KH.transpose() +
			 K * acc_noise *
			     K.transpose();  // p=(1-k*h)*p*(1-kh)t+kvkt P = (I-KH)P(I-KH)' + KRK'比P = (I-KH)P更稳定
}
