/****************************************************************************
 *
 *   Copyright (c) 2017 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 *
 * This module is a modification of the fixed wing module and it is designed for ground rovers.
 * It has been developed starting from the fw module, simplified and improved with dedicated items.
 *
 * All the acknowledgments and credits for the fw wing app are reported in those files.
 *
 * @author Marco Zorzi <mzorzi@student.ethz.ch>
 */

#include "tilt_body_by_vectoring.hpp"

/**
 * GroundRover attitude control app start / stop handling function
 *
 * @ingroup apps
 */
extern "C" __EXPORT int tilt_body_by_vectoring_main(int argc, char *argv[]);

namespace TBBV_control
{
	TiltBodyByVectoring	*g_control = nullptr;
}

TiltBodyByVectoring::TiltBodyByVectoring() :
	_mavlink_log_pub(nullptr),
	/* performance counters */
	_loop_perf(perf_alloc(PC_ELAPSED, "gnda_dt")),

	_nonfinite_input_perf(perf_alloc(PC_COUNT, "gnda_nani")),
	_nonfinite_output_perf(perf_alloc(PC_COUNT, "gnda_nano")),	
	_actuators_0_circuit_breaker_enabled(false)	
{
	_opt_recovery = new TailsitterRecovery();
	_parameter_handles.bat_scale_en = param_find("TBBV_BAT_SCA_EN");
	_parameter_handles.servo_re_1 = param_find("TBBV_SERVO_RE_1");
	_parameter_handles.servo_re_2 = param_find("TBBV_SERVO_RE_2");
	_parameter_handles.servo_re_3 = param_find("TBBV_SERVO_RE_3");
	_parameter_handles.servo_re_4 = param_find("TBBV_SERVO_RE_4");

	_parameter_handles.kmx = param_find("TBBV_KMX");
	_parameter_handles.kmy = param_find("TBBV_KMY");
	_parameter_handles.kmz = param_find("TBBV_KMZ");

	_parameter_handles.roll_p = param_find("TBBV_ROLL_P");
	_parameter_handles.pitch_p = param_find("TBBV_PITCH_P");
	_parameter_handles.yaw_p = param_find("TBBV_YAW_P");

	_parameter_handles.feed_fx = param_find("TBBV_FEED_FX");
	_parameter_handles.feed_fy = param_find("TBBV_FEED_FY");

	_parameter_handles.att_roll = param_find("TBBV_ATT_ROLL");
	_parameter_handles.att_pitch = param_find("TBBV_ATT_PITCH");
	_parameter_handles.att_yaw = param_find("TBBV_ATT_YAW");
	/* fetch initial parameter values */

	_I.identity();
	parameters_update();
}

TiltBodyByVectoring::~TiltBodyByVectoring()
{
	if (_control_task != -1) {

		/* task wakes up every 100ms or so at the longest */
		_task_should_exit = true;

		/* wait for a second for the task to quit at our request */
		unsigned i = 0;

		do {
			/* wait 20ms */
			usleep(20000);

			/* if we have given up, kill it */
			if (++i > 50) {
				px4_task_delete(_control_task);
				break;
			}
		} while (_control_task != -1);
	}

	delete _opt_recovery;

	perf_free(_loop_perf);
	perf_free(_nonfinite_input_perf);
	perf_free(_nonfinite_output_perf);

	TBBV_control::g_control = nullptr;
}

void
TiltBodyByVectoring::parameters_update()
{
	param_get(_parameter_handles.bat_scale_en, &_parameters.bat_scale_en);
	param_get(_parameter_handles.servo_re_1, &_parameters.servo_re_1);
	param_get(_parameter_handles.servo_re_2, &_parameters.servo_re_2);
	param_get(_parameter_handles.servo_re_3, &_parameters.servo_re_3);
	param_get(_parameter_handles.servo_re_4, &_parameters.servo_re_4);

	param_get(_parameter_handles.kmx, &_parameters.kmx);
	param_get(_parameter_handles.kmy, &_parameters.kmy);
	param_get(_parameter_handles.kmz, &_parameters.kmz);

	param_get(_parameter_handles.roll_p, &_parameters.roll_p);
	param_get(_parameter_handles.pitch_p, &_parameters.pitch_p);
	param_get(_parameter_handles.yaw_p, &_parameters.yaw_p);

	param_get(_parameter_handles.feed_fx, &_parameters.feed_fx);
	param_get(_parameter_handles.feed_fy, &_parameters.feed_fy);

	param_get(_parameter_handles.att_roll, &_parameters.att_roll);
	param_get(_parameter_handles.att_pitch, &_parameters.att_pitch);
	param_get(_parameter_handles.att_yaw, &_parameters.att_yaw);

	_actuators_0_circuit_breaker_enabled = circuit_breaker_enabled("CBRK_RATE_CTRL", CBRK_RATE_CTRL_KEY);

	/* pid parameters*/
	pid_init(&_roll_rate_ctrl, PID_MODE_DERIVATIV_SET, 0.01f);
	pid_set_parameters(&_roll_rate_ctrl,
			   1.0f,
			   0.0f,
			   0.0f,
			   0.0f,
			   1.0f);
}

void
TiltBodyByVectoring::vehicle_control_mode_poll()
{
	bool updated = false;
	orb_check(_vcontrol_mode_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(vehicle_control_mode), _vcontrol_mode_sub, &_vcontrol_mode);
	}
}

void
TiltBodyByVectoring::manual_control_setpoint_poll()
{
	bool updated = false;
	orb_check(_manual_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(manual_control_setpoint), _manual_sub, &_manual);
	}
}

void
TiltBodyByVectoring::battery_status_poll()
{
	/* check if there is a new message */
	bool updated;
	orb_check(_battery_status_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(battery_status), _battery_status_sub, &_battery_status);
	}
}

void
TiltBodyByVectoring::actuator_armed_poll()
{
	/* check if there is a new message */
	bool updated;
	orb_check(_actuator_armed_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(actuator_armed), _actuator_armed_sub, &_actuator_armed);
	}
}

void 
TiltBodyByVectoring::force_vector_generator(float dt)
{
	fx = _manual.x;
	fy = _manual.y;
	fz = -10.0f*_manual.z; 
}

void 
TiltBodyByVectoring::quaternions_setpoint_generator(float dt)
{
	yaw = yaw +( 0.5f *_manual.r * dt);
	q_setpoint.from_yaw(yaw);
}

void 
TiltBodyByVectoring::attitude_control(float dt)
{	
	/* get current rotation matrix from control state quaternions */
	//math::Quaternion q_att(_vehicle_attitude.q[0], _vehicle_attitude.q[1], _vehicle_attitude.q[2], _vehicle_attitude.q[3]); // form 1 to 2 
	
	//math::Quaternion q_sp(1.0f,0.0f,0.0f,0.0f);
	/*math::Quaternion q_sp = q_setpoint;

	math::Vector<3> att_p(8.0f,8.0f,8.0f) ;

	math::Vector<3> _rates_sp;
	_rates_sp.zero();
	
	_opt_recovery->setAttGains(att_p, 0.0f);
	_opt_recovery->calcOptimalRates(q_att, q_sp, 0.0f, _rates_sp);
    */
    math::Vector<3> att_p(_parameters.att_roll,_parameters.att_pitch,_parameters.att_yaw) ;
    math::Vector<3> _rates_sp;
	/* construct attitude setpoint rotation matrix */
	math::Quaternion q_sp = q_setpoint;
	math::Matrix<3, 3> R_sp = q_sp.to_dcm();
	/* get current rotation matrix from control state quaternions */
	math::Quaternion q_att(_vehicle_attitude.q[0], _vehicle_attitude.q[1], _vehicle_attitude.q[2], _vehicle_attitude.q[3]); // form 1 to 2 
	math::Matrix<3, 3> R = q_att.to_dcm();

	/* all input data is ready, run controller itself */

	/* try to move thrust vector shortest way, because yaw response is slower than roll/pitch */
	math::Vector<3> R_z(R(0, 2), R(1, 2), R(2, 2));
	math::Vector<3> R_sp_z(R_sp(0, 2), R_sp(1, 2), R_sp(2, 2));

	/* axis and sin(angle) of desired rotation (indexes: 0=pitch, 1=roll, 2=yaw).
	 * This is for roll/pitch only (tilt), e_R(2) is 0 */
	math::Vector<3> e_R = R.transposed() * (R_z % R_sp_z);

	/* calculate angle error */
	float e_R_z_sin = e_R.length(); // == sin(tilt angle error)
	float e_R_z_cos = R_z * R_sp_z; // == cos(tilt angle error) == (R.transposed() * R_sp)(2, 2)

	/* calculate rotation matrix after roll/pitch only rotation */
	math::Matrix<3, 3> R_rp;

	if (e_R_z_sin > 0.0f) {
		/* get axis-angle representation */
		float e_R_z_angle = atan2f(e_R_z_sin, e_R_z_cos);
		math::Vector<3> e_R_z_axis = e_R / e_R_z_sin;

		e_R = e_R_z_axis * e_R_z_angle;

		/* cross product matrix for e_R_axis */
		math::Matrix<3, 3> e_R_cp;
		e_R_cp.zero();
		e_R_cp(0, 1) = -e_R_z_axis(2);
		e_R_cp(0, 2) = e_R_z_axis(1);
		e_R_cp(1, 0) = e_R_z_axis(2);
		e_R_cp(1, 2) = -e_R_z_axis(0);
		e_R_cp(2, 0) = -e_R_z_axis(1);
		e_R_cp(2, 1) = e_R_z_axis(0);

		/* rotation matrix for roll/pitch only rotation */
		R_rp = R * (_I + e_R_cp * e_R_z_sin + e_R_cp * e_R_cp * (1.0f - e_R_z_cos));

	} else {
		/* zero roll/pitch rotation */
		R_rp = R;
	}

	/* R_rp and R_sp have the same Z axis, calculate yaw error */
	math::Vector<3> R_sp_x(R_sp(0, 0), R_sp(1, 0), R_sp(2, 0));
	math::Vector<3> R_rp_x(R_rp(0, 0), R_rp(1, 0), R_rp(2, 0));

	/* calculate the weight for yaw control
	 * Make the weight depend on the tilt angle error: the higher the error of roll and/or pitch, the lower
	 * the weight that we use to control the yaw. This gives precedence to roll & pitch correction.
	 * The weight is 1 if there is no tilt error.
	 */
	float yaw_w = e_R_z_cos * e_R_z_cos;

	/* calculate the angle between R_rp_x and R_sp_x (yaw angle error), and apply the yaw weight */
	e_R(2) = atan2f((R_rp_x % R_sp_x) * R_sp_z, R_rp_x * R_sp_x) * yaw_w;

	if (e_R_z_cos < 0.0f) {
		/* for large thrust vector rotations use another rotation method:
		 * calculate angle and axis for R -> R_sp rotation directly */
		math::Quaternion q_error;
		q_error.from_dcm(R.transposed() * R_sp);
		math::Vector<3> e_R_d = q_error(0) >= 0.0f ? q_error.imag()  * 2.0f : -q_error.imag() * 2.0f;

		/* use fusion of Z axis based rotation and direct rotation */
		float direct_w = e_R_z_cos * e_R_z_cos * yaw_w;
		e_R = e_R * (1.0f - direct_w) + e_R_d * direct_w;
	}

	/* calculate angular rates setpoint */
	_rates_sp = att_p.emult(e_R);

	_v_rates_sp.roll = _rates_sp(0) ;
	_v_rates_sp.pitch = _rates_sp(1) ;
	_v_rates_sp.yaw = _rates_sp(2) ;
}

void
TiltBodyByVectoring::body_rates_control(float dt)
{
	math::Vector<3> rate_mea(_vehicle_attitude.rollspeed, _vehicle_attitude.pitchspeed, _vehicle_attitude.yawspeed);
	
	math::Vector<3> rate_sp(_v_rates_sp.roll, _v_rates_sp.pitch, _v_rates_sp.yaw);

	math::Vector<3> rate_error = rate_sp - rate_mea ;	

	/* time constant (inverse of P gain ) */
	float roll_p = 1.0f;
	float pitch_p = 1.0f;
	float yaw_p = 1.0f;

	roll_p = _parameters.roll_p;
	pitch_p = _parameters.pitch_p;
	yaw_p = _parameters.yaw_p;

	{
		/* Future PID controller */
	}

	math::Vector<3> rate_p(roll_p,pitch_p,yaw_p);
	math::Vector<3> torque_des = rate_p.emult(rate_error) ;

	//torque_des += ( rate_mea % (J * rate_mea) ) ;

	mx = torque_des(0);
	my = torque_des(1);
	mz = torque_des(2);
}

void 
TiltBodyByVectoring::actuator_mixer(float dt)
{
	float Kmx = 0.0f;
	float Kmy = 1.0f;
	float Kmz = 0.0f;

	Kmx = _parameters.kmx;
	Kmy = _parameters.kmy;
	Kmz = _parameters.kmz;

	{
		fx = fx*_parameters.feed_fx;
		fy = fy*_parameters.feed_fy;
	}

	//fz = -10.0f; /* Newton */

	fz = math::max( -fz , 0.0f );
	/*
	int sign_my ;
	if(my >=0.0f){
		sign_my = 1;
	}else{
		sign_my = -1;
	}

	my = math::min( fabsf(Kmy*my) , fz/4 )*sign_my ;
	*/
	float A1_long = fz/4 - Kmx*mx ;
	float A1_lat  = fx/4 - Kmy*my + Kmz*mz ;

	float A2_long = fz/4 + Kmx*mx ;
	float A2_lat  = fx/4 - Kmy*my - Kmz*mz ;

	float A3_long = fz/4 - Kmx*mx ;
	float A3_lat  = fx/4 + Kmy*my + Kmz*mz ;

	float A4_long = fz/4 + Kmx*mx ;
	float A4_lat  = fx/4 + Kmy*my - Kmz*mz ;

	A1_long = math::constrain(A1_long, 0.5f, 6.0f);
	A2_long = math::constrain(A2_long, 0.5f, 6.0f);
	A3_long = math::constrain(A3_long, 0.5f, 6.0f);
	A4_long = math::constrain(A4_long, 0.5f, 6.0f);

	t1 = sqrtf(A1_long*A1_long + A1_lat*A1_lat) ;
	t2 = sqrtf(A2_long*A2_long + A2_lat*A2_lat) ;
	t3 = sqrtf(A3_long*A3_long + A3_lat*A3_lat) ;
	t4 = sqrtf(A4_long*A4_long + A4_lat*A4_lat) ;

	z1 = atanf(A1_lat / (A1_long)) ;
	z2 = atanf(A2_lat / (A2_long)) ;
	z3 = atanf(A3_lat / (A3_long)) ;
	z4 = atanf(A4_lat / (A4_long)) ;
}

void
TiltBodyByVectoring::actuator_normalize()
{
	float Thrust_max = 6.0f ; /* Newton */

	t1 = t1/Thrust_max ;
	t2 = t2/Thrust_max ;
	t3 = t3/Thrust_max ;
	t4 = t4/Thrust_max ;

	float lower_lim = 0.0f ;
	float upper_lim = 1.0f ;
	t1 = math::constrain(t1 , lower_lim , upper_lim);
	t2 = math::constrain(t2 , lower_lim , upper_lim);
	t3 = math::constrain(t3 , lower_lim , upper_lim);
	t4 = math::constrain(t4 , lower_lim , upper_lim);

	float Tilt_max = math::radians(40.0f); /* radian */
	z1 = z1/Tilt_max ;
	z2 = z2/Tilt_max ;
	z3 = z3/Tilt_max ;
	z4 = z4/Tilt_max ;

	lower_lim = -1.0f;
	z1 = math::constrain(z1 , lower_lim , upper_lim);
	z2 = math::constrain(z2 , lower_lim  , upper_lim);
	z3 = math::constrain(z3 , lower_lim  , upper_lim);
	z4 = math::constrain(z4 , lower_lim  , upper_lim);

} 
void 
TiltBodyByVectoring::actuator_set()
{
	/** 
	actuator list
	control group 0
		channel 3 : mean throttle
		channel 4 : differrent throttle 1
		channel 5 : differrent throttle 2
		channel 6 : differrent throttle 3
		channel 7 : differrent throttle 4
	
	control group 1
		channel 4 : servo 1
		channel 5 : servo 2
		channel 6 : servo 3
		channel 7 : servo 4
	*/
	float mean_thr = 0.0f;
	mean_thr = ((t1+t2+t3+t4)/((float)number_of_motor));
	
	/* motor */
	_actuators_0.control[3] = mean_thr ;

	_actuators_0.control[4] = t1 - mean_thr ;
	_actuators_0.control[5] = t2 - mean_thr ;
	_actuators_0.control[6] = t3 - mean_thr ;
	_actuators_0.control[7] = t4 - mean_thr ;

	/* tilt servo */
	_actuators_1.control[4] = z1*( 1-2*(int)_parameters.servo_re_1) ;
	_actuators_1.control[5] = z2*( 1-2*(int)_parameters.servo_re_2) ;
	_actuators_1.control[6] = z3*( 1-2*(int)_parameters.servo_re_3) ;
	_actuators_1.control[7] = z4*( 1-2*(int)_parameters.servo_re_4) ;
}

void
TiltBodyByVectoring::actuator_set_zero()
{
	/* motor */
	_actuators_0.control[3] = 0.0f ;

	_actuators_0.control[4] = 0.0f ;
	_actuators_0.control[5] = 0.0f ;
	_actuators_0.control[6] = 0.0f ;
	_actuators_0.control[7] = 0.0f ;

	/* tilt servo */
	_actuators_1.control[4] = 0.0f ;
	_actuators_1.control[5] = 0.0f ;
	_actuators_1.control[6] = 0.0f ;
	_actuators_1.control[7] = 0.0f ;
}

void
TiltBodyByVectoring::actuator_disarm_rotor()
{
	/* motor */
	_actuators_0.control[3] = 0.0f ;

	_actuators_0.control[4] = 0.0f ;
	_actuators_0.control[5] = 0.0f ;
	_actuators_0.control[6] = 0.0f ;
	_actuators_0.control[7] = 0.0f ;
}

void
TiltBodyByVectoring::publish_vehicle_rates_setpoint()
{
	if (_rates_sp_pub != nullptr) {
		orb_publish(ORB_ID(vehicle_rates_setpoint), _rates_sp_pub, &_v_rates_sp);
	} else {
		_rates_sp_pub = orb_advertise(ORB_ID(vehicle_rates_setpoint), &_v_rates_sp);
	}
}

void 
TiltBodyByVectoring::force_enable_motor()
{
	int pwm_value = PWM_DEFAULT_MAX;
	int ret;
	unsigned servo_count;
	const char *dev = PWM_OUTPUT0_DEVICE_PATH;
	int fd = px4_open(dev, 0);

	if (fd < 0) {
		PX4_WARN("can't open %s", dev);
	}

	ret = px4_ioctl(fd, PWM_SERVO_GET_COUNT, (unsigned long)&servo_count);
	struct pwm_output_values pwm_max_values;
	memset(&pwm_max_values, 0, sizeof(pwm_max_values));

	for (int i = 0; i < 4 ; i++) {
		pwm_max_values.values[i] = pwm_value;
		pwm_max_values.channel_count = 4;
	}

	ret = px4_ioctl(fd, PWM_SERVO_SET_MAX_PWM, (long unsigned int)&pwm_max_values);

	if (ret != OK) {
		PX4_WARN("failed setting max values");
	}

	px4_close(fd);
}

void 
TiltBodyByVectoring::force_disable_motor()
{
	int pwm_value = PWM_MOTOR_OFF;
	int ret;
	unsigned servo_count;
	const char *dev = PWM_OUTPUT0_DEVICE_PATH;
	int fd = px4_open(dev, 0);

	if (fd < 0) {
		PX4_WARN("can't open %s", dev);
	}

	ret = px4_ioctl(fd, PWM_SERVO_GET_COUNT, (unsigned long)&servo_count);
	struct pwm_output_values pwm_max_values;
	memset(&pwm_max_values, 0, sizeof(pwm_max_values));

	for (int i = 0; i < 4 ; i++) {
		pwm_max_values.values[i] = pwm_value;
		pwm_max_values.channel_count = 4;
	}

	ret = px4_ioctl(fd, PWM_SERVO_SET_MAX_PWM, (long unsigned int)&pwm_max_values);

	for (int i = 0; i < 4 ; i++) {
		ret = px4_ioctl(fd, PWM_SERVO_SET(i), (unsigned long)pwm_value);
	}

	if (ret != OK) {
		PX4_WARN("failed setting max values");
	}

	px4_close(fd);
}

void
TiltBodyByVectoring::task_main_trampoline(int argc, char *argv[])
{
	TBBV_control::g_control->task_main();
}

void
TiltBodyByVectoring::task_main()
{	
	/* Wait */
	usleep(1000*1000);
	mavlink_log_info(&_mavlink_log_pub, "[TBBV] started");

	J(0,0) = 1.0f;
	J(0,1) = 0.0f;
	J(0,2) = 0.0f;

	J(1,0) = 0.0f;
	J(1,1) = 1.0f;
	J(1,2) = 0.0f;

	J(2,0) = 0.0f;
	J(2,1) = 0.0f;
	J(2,2) = 1.0f;

	_vehicle_attitude_sub = orb_subscribe(ORB_ID(vehicle_attitude));
	_vcontrol_mode_sub = orb_subscribe(ORB_ID(vehicle_control_mode));
	_params_sub = orb_subscribe(ORB_ID(parameter_update));
	_manual_sub = orb_subscribe(ORB_ID(manual_control_setpoint));
	_battery_status_sub = orb_subscribe(ORB_ID(battery_status));
	_actuator_armed_sub = orb_subscribe(ORB_ID(actuator_armed));
	parameters_update();

	/* get an initial update for all sensor and status data */
	vehicle_control_mode_poll();
	manual_control_setpoint_poll();
	battery_status_poll();
	actuator_armed_poll();
			
	/* wakeup source */
	struct pollfd fds[2];
	/* Setup of loop */
	fds[0].fd = _params_sub;
	fds[0].events = POLLIN;	
	fds[1].fd = _vehicle_attitude_sub;
	fds[1].events = POLLIN;

	_task_running = true;

	while (!_task_should_exit) {
		
		/* wait for up to 100ms for data */
		int pret = poll(fds, 2, 500);
		/* timed out - periodic check for _task_should_exit, etc. */
		if (pret == 0) {
			continue;
		}

		/* this is undesirable but not much we can do - might want to flag unhappy status */
		if (pret < 0) {
			warn("poll error %d, %d", pret, errno);
			continue;
		}

		perf_begin(_loop_perf);

		/* only update parameters if they changed */
		if (fds[0].revents & POLLIN) {
			/* read from param to clear updated flag (uORB API requirement) */
			struct parameter_update_s update;
			orb_copy(ORB_ID(parameter_update), _params_sub, &update);
			/* if a param update occured, re-read our parameters */
			parameters_update();
		}
		/* only run controller if attitude changed */
		if (fds[1].revents & POLLIN) {
			static uint64_t last_run = 0;
			float deltaT = (hrt_absolute_time() - last_run) / 1000000.0f;
			last_run = hrt_absolute_time();

			/* guard against too large deltaT's */
			if (deltaT > 1.0f ||
			    fabsf(deltaT) < 0.00001f ||
			    !PX4_ISFINITE(deltaT)) {
				deltaT = 0.01f;
			}

			/* load local copies */
			orb_copy(ORB_ID(vehicle_attitude), _vehicle_attitude_sub, &_vehicle_attitude);

			vehicle_control_mode_poll();
			manual_control_setpoint_poll();
			battery_status_poll();
			actuator_armed_poll();

			/* Body rate control */
			float dt = deltaT ; 

			if (_vcontrol_mode.flag_control_attitude_enabled){
				force_vector_generator(dt);
				quaternions_setpoint_generator(dt);
				attitude_control(dt);
			}
			if (_vcontrol_mode.flag_control_rates_enabled) {
				/* run rates controller */

				body_rates_control(dt);
				
				actuator_mixer(dt);
				actuator_normalize();

				actuator_set();				
			} else {
				actuator_set_zero();
				/* off */
			}


			/*
			if (_parameters.bat_scale_en && _battery_status.scale > 0.0f)
			{
				for (int i = 0; i < number_of_motor; ++i)
				{
					_actuators.control[i] *= _battery_status.scale;					
				}
			}*/

			if(!_actuator_armed.armed || true ){
				/* reset yaw */
				math::Quaternion q_att(_vehicle_attitude.q[0], _vehicle_attitude.q[1], _vehicle_attitude.q[2], _vehicle_attitude.q[3]); // form 1 to 2 
				math::Vector<3> Euler= q_att.to_euler();
				yaw = Euler(3);
				//actuator_disarm_rotor();					
			}

			/* lazily publish the setpoint only once available */
			_actuators_0.timestamp = hrt_absolute_time();
			_actuators_0.timestamp_sample = _vehicle_attitude.timestamp;
			
			_actuators_1.timestamp = hrt_absolute_time();
			_actuators_1.timestamp_sample = _vehicle_attitude.timestamp;
			/* calling publish function */
			publish_vehicle_rates_setpoint();		
			
			/* Only publish if any of the proper modes are enabled */
			
			if ( !_actuators_0_circuit_breaker_enabled || true) {
				if (_vcontrol_mode.flag_control_rates_enabled) {

					/* publish the actuator controls */
					if (_actuators_0_pub != nullptr) {
						orb_publish(ORB_ID(actuator_controls_0), _actuators_0_pub, &_actuators_0);

					} else {
						_actuators_0_pub = orb_advertise(ORB_ID(actuator_controls_0), &_actuators_0);
					}

					/* publish the actuator controls */
					if (_actuators_1_pub != nullptr) {
						orb_publish(ORB_ID(actuator_controls_1), _actuators_1_pub, &_actuators_1);

					} else {
						_actuators_1_pub = orb_advertise(ORB_ID(actuator_controls_1), &_actuators_1);
					}
				}
			}
			
		}
		
		perf_end(_loop_perf);
	}

	warnx("exiting.\n");

	_control_task = -1;
	_task_running = false;
}

int
TiltBodyByVectoring::start()
{
	ASSERT(_control_task == -1);

	/* start the task */
	_control_task = px4_task_spawn_cmd("tilt_body_by_vectoring",
					   SCHED_DEFAULT,
					   SCHED_PRIORITY_MAX - 5,
					   1500,
					   (px4_main_t)&TiltBodyByVectoring::task_main_trampoline,
					   nullptr);

	if (_control_task < 0) {
		warn("task start failed");
		return -errno;
	}

	return PX4_OK;
}

int tilt_body_by_vectoring_main(int argc, char *argv[])
{
	if (argc < 2) {
		warnx("usage: gnd_att_control {start|stop|status}");
		return 1;
	}

	if (!strcmp(argv[1], "start")) {

		if (TBBV_control::g_control != nullptr) {
			warnx("already running");
			return 1;
		}

		TBBV_control::g_control = new TiltBodyByVectoring;

		if (TBBV_control::g_control == nullptr) {
			warnx("alloc failed");
			return 1;
		}

		if (PX4_OK != TBBV_control::g_control->start()) {
			delete TBBV_control::g_control;
			TBBV_control::g_control = nullptr;
			warn("start failed");
			return 1;
		}

		/* check if the waiting is necessary at all */
		if (TBBV_control::g_control == nullptr || !TBBV_control::g_control->task_running()) {

			/* avoid memory fragmentation by not exiting start handler until the task has fully started */
			while (TBBV_control::g_control == nullptr || !TBBV_control::g_control->task_running()) {
				usleep(50000);
				printf(".");
				fflush(stdout);
			}

			printf("\n");
		}

		return 0;
	}

	if (!strcmp(argv[1], "stop")) {
		if (TBBV_control::g_control == nullptr) {
			warnx("not running");
			return 1;
		}

		delete TBBV_control::g_control;
		TBBV_control::g_control = nullptr;
		return 0;
	}

	if (!strcmp(argv[1], "status")) {
		if (TBBV_control::g_control) {
			warnx("running");
			return 0;

		} else {
			warnx("not running");
			return 1;
		}
	}

	warnx("unrecognized command");
	return 1;
}