
/****************************************************************************
 *
 *   Copyright (c) 2013, 2014 PX4 Development Team. All rights reserved.
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
 * @file bottle_drop.cpp
 *
 * Bottle drop module for Outback Challenge 2014, Team Swiss Fang
 *
 * @author Dominik Juchli <juchlid@ethz.ch>
 * @author Julian Oes <joes@student.ethz.ch>
 */

#include <px4_config.h>
#include <px4_tasks.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <math.h>
#include <poll.h>
#include <time.h>
#include <sys/ioctl.h>
#include <drivers/device/device.h>
#include <drivers/drv_hrt.h>
#include <arch/board/board.h>
#include <uORB/uORB.h>
#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/vehicle_command_ack.h>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/wind_estimate.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/debug_key_value.h>
#include <systemlib/param/param.h>
#include <systemlib/err.h>
#include <systemlib/mavlink_log.h>
#include <geo/geo.h>
#include <dataman/dataman.h>
#include <mathlib/mathlib.h>

enum state
{
	xy_position_change = 1,
	inject_position_change,
	z_position_change
};

/**
 * humming app start / stop handling function
 *
 * @ingroup apps
 */
extern "C" __EXPORT int humming_main(int argc, char *argv[]);

class Humming
{
public:
	/**
	 * Constructor
	 */
	Humming();

	/**
	 * Destructor, also kills task.
	 */
	~Humming();

	/**
	 * Start the task.
	 *
	 * @return		OK on success.
	 */
	int		start();

	/**
	 * Display status.
	 */
	void		status();

	void		open_bay();
	void		close_bay();
	void		drop();
	void		lock_release();

	
private:
	bool		_task_should_exit;		/**< if true, task should exit */
	int		_main_task;			/**< handle for task */
	orb_advert_t	_mavlink_log_pub;

	int		_command_sub;
	struct vehicle_command_s	_command;
	
	int 	_control_mode_sub;
	struct vehicle_control_mode_s _control_mode;

	int 	_manual_control_setpoint_sub;
	struct manual_control_setpoint_s _manual_control_setpoint;

	int		_params_sub;			/**< parameter updates subscription */
	
	bool 	_humming_sys_start;

	struct actuator_controls_s _actuators;
	orb_advert_t	_actuator_pub;

	struct debug_key_value_s dbg;
	orb_advert_t pub_dbg;

	float actuators_setpoint[4];
	int actuators_error[4];
	bool task_1;
	int task_point;
	int state ;
	int counter;

	bool servo_push = false;
	int servo_loop_cnt = 11;
	struct {

		param_t act_1_length;
		param_t act_2_length;
		param_t act_3_length;
		param_t act_4_length;
		param_t act_5_length;
		param_t act_6_length;

		param_t act_1_speed;
		param_t act_2_speed;
		param_t act_3_speed;
		param_t act_4_speed;
		param_t act_5_speed;
		param_t act_6_speed;

	}	_params_handles;		/**< handles for interesting parameters */

	struct {

		float act_length[6];
		float act_speed[6];
	}	_params{};

	/**
	 * Update our local parameter cache.
	 */
	int			parameters_update();

	/**
	 * Check for parameter update and handle it.
	 */
	void		parameter_update_poll(bool force);

	void		task_main();

	void		handle_command(struct vehicle_command_s *cmd);

	void		answer_command(struct vehicle_command_s *cmd, unsigned result);

	/**
	 * Set the actuators
	 */
	int		actuators_publish();

	/**
	 * Shim for calling task_main from task_create.
	 */
	static void	task_main_trampoline(int argc, char *argv[]);

	void ack_vehicle_command(orb_advert_t &vehicle_command_ack_pub, struct vehicle_command_s &cmd, uint32_t result);
};

namespace humming
{
Humming	*g_humming;
}

Humming::Humming() :

	_task_should_exit(false),
	_main_task(-1),
	_mavlink_log_pub(nullptr),
	_command_sub(-1),
	_command {},
	_control_mode_sub(-1),
	_control_mode {},
	_manual_control_setpoint_sub(-1),
	_manual_control_setpoint {},
	_params_sub(-1),
	_humming_sys_start(false),	
	_actuators {},
	_actuator_pub(nullptr)
{	
	for (int i = 0; i < 4; ++i)
	{
		actuators_setpoint[i] = 0.0f;
	}
	for (int i = 0; i < 4; ++i)
	{
		actuators_error[i] = -1.0f;
	}
	task_1 = false;
	state = 0;
	counter = 0;
	task_point = -1;


	_params_handles.act_1_length = param_find("ACT_1_LEN");
	_params_handles.act_2_length = param_find("ACT_2_LEN");
	_params_handles.act_3_length = param_find("ACT_3_LEN");
	_params_handles.act_4_length = param_find("ACT_4_LEN");
	_params_handles.act_5_length = param_find("ACT_5_LEN");
	_params_handles.act_6_length = param_find("ACT_6_LEN");

	_params_handles.act_1_speed = param_find("ACT_1_SPE");
	_params_handles.act_2_speed = param_find("ACT_2_SPE");
	_params_handles.act_3_speed = param_find("ACT_3_SPE");
	_params_handles.act_4_speed = param_find("ACT_4_SPE");
	_params_handles.act_5_speed = param_find("ACT_5_SPE");
	_params_handles.act_6_speed = param_find("ACT_6_SPE");
	/* fetch initial parameter values */
	parameters_update();
}

Humming::~Humming()
{
	if (_main_task != -1) {

		/* task wakes up every 100ms or so at the longest */
		_task_should_exit = true;

		/* wait for a second for the task to quit at our request */
		unsigned i = 0;

		do {
			/* wait 20ms */
			usleep(20000);

			/* if we have given up, kill it */
			if (++i > 50) {
				px4_task_delete(_main_task);
				break;
			}
		} while (_main_task != -1);
	}

	humming::g_humming = nullptr;
}

int
Humming::parameters_update()
{	
	float v;
	param_get(_params_handles.act_1_length, &v);
	_params.act_length[0] = v;
	param_get(_params_handles.act_2_length, &v);
	_params.act_length[1] = v;
	param_get(_params_handles.act_3_length, &v);
	_params.act_length[2] = v;
	param_get(_params_handles.act_4_length, &v);
	_params.act_length[3] = v;
	param_get(_params_handles.act_5_length, &v);
	_params.act_length[4] = v;
	param_get(_params_handles.act_6_length, &v);
	_params.act_length[5] = v;

	param_get(_params_handles.act_1_speed, &v);
	_params.act_speed[0] = v;
	param_get(_params_handles.act_2_speed, &v);
	_params.act_speed[1] = v;
	param_get(_params_handles.act_3_speed, &v);
	_params.act_speed[2] = v;
	param_get(_params_handles.act_4_speed, &v);
	_params.act_speed[3] = v;
	param_get(_params_handles.act_5_speed, &v);
	_params.act_speed[4] = v;
	param_get(_params_handles.act_6_speed, &v);
	_params.act_speed[5] = v;
	return OK;
}

void
Humming::parameter_update_poll(bool force)
{
	bool updated;
	struct parameter_update_s param_update;
	/* Check if parameters have changed */
	orb_check(_params_sub, &updated);

	if (updated) {		
		orb_copy(ORB_ID(parameter_update), _params_sub, &param_update);		
	}
	if (updated || force){		
		parameters_update();
	}
}

int
Humming::start()
{
	ASSERT(_main_task == -1);

	/* start the task */
	_main_task = px4_task_spawn_cmd("humming",
					SCHED_DEFAULT,
					SCHED_PRIORITY_DEFAULT ,
					2031,
					(px4_main_t)&Humming::task_main_trampoline,
					nullptr);

	if (_main_task < 0) {
		warn("task start failed");
		return -errno;
	}

	return OK;
}


void
Humming::status()
{
	warnx("running");
}

void
Humming::open_bay()
{
	_actuators.control[0] = -1.0f;
	_actuators.control[1] = 1.0f;

	actuators_publish();

	usleep(500 * 1000);
}

void
Humming::close_bay()
{
	// closed door and locked survival kit
	_actuators.control[0] = -1.0f;
	_actuators.control[1] = -1.0f;

	actuators_publish();

	// delay until the bay is closed
	usleep(500 * 1000);
}

void
Humming::drop()
{
	open_bay();

	_actuators.control[2] = 1.0f;

	actuators_publish();

	// Give it time to drop
	usleep(1000 * 1000);
}

void
Humming::lock_release()
{
	_actuators.control[2] = 1.0f;
	actuators_publish();

	usleep(1000 * 1000);
}

int
Humming::actuators_publish()
{
	_actuators.timestamp = hrt_absolute_time();

	// lazily publish _actuators only once available
	if (_actuator_pub != nullptr) {		
		return orb_publish(ORB_ID(actuator_controls_2), _actuator_pub, &_actuators);

	} else {
		_actuator_pub = orb_advertise(ORB_ID(actuator_controls_2), &_actuators);

		if (_actuator_pub != nullptr) {
			return OK;

		} else {
			return -1;
		}
	}
}

void
Humming::task_main()
{
	/* Wait */
	usleep(1000*1000);

	mavlink_log_info(&_mavlink_log_pub, "[humming] started");

	_command_sub = orb_subscribe(ORB_ID(vehicle_command));	

	_control_mode_sub = orb_subscribe(ORB_ID(vehicle_control_mode));

	_manual_control_setpoint_sub = orb_subscribe(ORB_ID(manual_control_setpoint));	

	orb_advert_t vehicle_command_ack_pub = nullptr;

	pub_dbg = orb_advertise(ORB_ID(debug_key_value), &dbg);

	bool updated = false;
	/* initialize parameters cache */
	parameters_update();

	// wakeup source(s)
	struct pollfd fds[1];

	// Setup of loop
	fds[0].fd = _command_sub;
	fds[0].events = POLLIN;

	/* start */
	/*
	goto_pos(1.0f,1.0f);
	liquid_pos(1.0f);
	probe_pos(1.0f);
	*/

	const unsigned sleeptime_us = 9000;

	while (!_task_should_exit) {

		/* wait for up to 100ms for data */
		int pret = poll(&fds[0], (sizeof(fds) / sizeof(fds[0])), 50);

		/* this is undesirable but not much we can do - might want to flag unhappy status */
		if (pret < 0) {
			warn("poll error %d, %d", pret, errno);
			continue;
		}

		/* vehicle commands updated */
		if (fds[0].revents & POLLIN) {

			parameter_update_poll(true);

			orb_copy(ORB_ID(vehicle_command), _command_sub, &_command);
			handle_command(&_command);			
			ack_vehicle_command(vehicle_command_ack_pub, _command,vehicle_command_s::VEHICLE_CMD_RESULT_ACCEPTED);
		}

		while( _humming_sys_start ){
			static uint64_t last_run = 0;
			float deltaT = (hrt_absolute_time() - last_run) / 1000000.0f;
			last_run = hrt_absolute_time();

			/* guard against too large deltaT's */
			if (deltaT > 1.0f ||
			    fabsf(deltaT) < 0.00001f ||
			    !PX4_ISFINITE(deltaT)) {
				deltaT = 0.01f;
			}

			parameter_update_poll(true);

			orb_check(_command_sub, &updated);

			if (updated && servo_push == false) {
				orb_copy(ORB_ID(vehicle_command), _command_sub, &_command);
				handle_command(&_command);
				ack_vehicle_command(vehicle_command_ack_pub, _command,vehicle_command_s::VEHICLE_CMD_RESULT_ACCEPTED);
			}

			if (servo_push == true)
			{
				servo_loop_cnt = servo_loop_cnt - 1;
				if ( servo_loop_cnt == 0 )
				{
					actuators_setpoint[2] = math::constrain(actuators_setpoint[2] + 0.15f,-1.0f,1.0f);
					servo_push = false;
				}
			}
			for (uint8_t i = 0; i < 4; i++)
			{
				float dead_zone = 0.001 ;
				if( actuators_setpoint[i] - _actuators.control[i] > dead_zone){
					_actuators.control[i] = _actuators.control[i] + (_params.act_speed[i]*deltaT)/_params.act_length[i] ;
				}
				else if( actuators_setpoint[i] - _actuators.control[i] < -dead_zone){
					_actuators.control[i] = _actuators.control[i] - (_params.act_speed[i]*deltaT)/_params.act_length[i] ;
				}
				_actuators.control[i] = math::constrain(_actuators.control[i], -1.0f,1.0f);
			}

			actuators_publish();

			strcpy(dbg.key,"ACT3");
			dbg.value = _actuators.control[2];

			orb_publish(ORB_ID(debug_key_value), pub_dbg, &dbg);

			usleep(sleeptime_us);
		}	
			
		usleep(sleeptime_us);
	}

	warnx("exiting.");
	_main_task = -1;
	_exit(0);
}

void
Humming::handle_command(struct vehicle_command_s *cmd)
{
	switch (cmd->command) {
	case vehicle_command_s::VEHICLE_CMD_CUSTOM_0:

		/*
		 * param1 and param2 set to 1: open and drop
		 * param1 set to 1: open
		 * else: close (and don't drop)
		 */
		if (cmd->param1 > 0.5f && cmd->param2 > 0.5f) {
			open_bay();
			drop();
			mavlink_log_critical(&_mavlink_log_pub, "drop bottle");

		} else if (cmd->param1 > 0.5f) {
			open_bay();
			mavlink_log_critical(&_mavlink_log_pub, "opening bay");

		} else {
			lock_release();
			close_bay();
			mavlink_log_critical(&_mavlink_log_pub, "closing bay");
		}

		answer_command(cmd, vehicle_command_s::VEHICLE_CMD_RESULT_ACCEPTED);
		break;

	case vehicle_command_s::VEHICLE_CMD_CUSTOM_1:
		/* start humming */
		if (cmd->param1 > -0.5f && cmd->param1 < 0.5f)
		{
			_humming_sys_start = true;
			mavlink_log_info(&_mavlink_log_pub, "start humming");
		}

		/* reset all */
		if (cmd->param1 > -1.5f && cmd->param1 < -0.5f)
		{
			mavlink_log_info(&_mavlink_log_pub, "reset all");
			actuators_setpoint[0] = 0.0f;
			actuators_setpoint[1] = 0.0f;
			actuators_setpoint[2] = 0.0f;
		}
		
		/* probe */
		if (cmd->param1 > 0.5f && cmd->param1 < 1.5f )
		{
			/* code */
			/* positive */
			if (cmd->param2 > 0.5f && cmd->param2 < 1.5f)
			{
				actuators_setpoint[0] = math::constrain( actuators_setpoint[0] + 0.2f,-1.0f,1.0f);
				mavlink_log_info(&_mavlink_log_pub, "increase probe position");
			}
			/* negative */
			else if (cmd->param2 > 1.5f && cmd->param2 < 2.5f)
			{
				actuators_setpoint[0] = math::constrain( actuators_setpoint[0] - 0.2f,-1.0f,1.0f);
				mavlink_log_info(&_mavlink_log_pub, "decrease probe position");
			}

			else
			{
				actuators_setpoint[0] = math::constrain( 0.0f,-1.0f,1.0f);
				mavlink_log_info(&_mavlink_log_pub, "reset probe position");
			}
		}
		/* couplant */
		else if (cmd->param1 > 1.5f && cmd->param1 < 2.5f)
		{
			/* code */
			/* positive */
			if (cmd->param2 > 0.5f && cmd->param2 < 1.5f)
			{
				actuators_setpoint[1] = math::constrain( actuators_setpoint[1] + 0.2f,-1.0f,1.0f);
				mavlink_log_info(&_mavlink_log_pub, "increase syringe position");
			}
			/* negative */
			else if (cmd->param2 > 1.5f && cmd->param2 < 2.5f)
			{
				actuators_setpoint[1] = math::constrain( actuators_setpoint[1] - 0.2f,-1.0f,1.0f);
				mavlink_log_info(&_mavlink_log_pub, "decrease syringe position");
			}

			else
			{
				actuators_setpoint[1] = math::constrain( 0.0f,-1.0f,1.0f);
				mavlink_log_info(&_mavlink_log_pub, "reset syringe position");
			}
		}
		/* servo */
		else if (cmd->param1 > 2.5f  && cmd->param1 < 3.5f)
		{
			/* code */
			/* positive */
			if (cmd->param2 > 0.5f && cmd->param2 < 1.5f)
			{
				actuators_setpoint[2] = math::constrain(actuators_setpoint[2] + 0.15f,-1.0f,1.0f);
				mavlink_log_info(&_mavlink_log_pub, "increase servo position");
			}
			/* negative */
			else if (cmd->param2 > 1.5f && cmd->param2 < 2.5f)
			{
				actuators_setpoint[2] = math::constrain(actuators_setpoint[2] - 0.15f,-1.0f,1.0f);
				mavlink_log_info(&_mavlink_log_pub, "decrease servo position");
			}
			/* push */			
			else if (cmd->param2 > 2.5f && cmd->param2 < 3.5f)
			{
				actuators_setpoint[2] = math::constrain(actuators_setpoint[2] - 0.15f,-1.0f,1.0f);
				mavlink_log_info(&_mavlink_log_pub, "push servo");
				servo_push = true;
				servo_loop_cnt = 14;
			}

			else
			{
				actuators_setpoint[2] = math::constrain( 0.0f,-1.0f,1.0f);
				mavlink_log_info(&_mavlink_log_pub, "reset servo position");
			}
		}
		answer_command(cmd, vehicle_command_s::VEHICLE_CMD_RESULT_ACCEPTED);
		break;
	default:
		break;
	}
}

void
Humming::answer_command(struct vehicle_command_s *cmd, unsigned result)
{
	switch (result) {
	case vehicle_command_s::VEHICLE_CMD_RESULT_ACCEPTED:
		break;

	case vehicle_command_s::VEHICLE_CMD_RESULT_DENIED:
		mavlink_log_critical(&_mavlink_log_pub, "command denied: %u", cmd->command);
		break;

	case vehicle_command_s::VEHICLE_CMD_RESULT_FAILED:
		mavlink_log_critical(&_mavlink_log_pub, "command failed: %u", cmd->command);
		break;

	case vehicle_command_s::VEHICLE_CMD_RESULT_TEMPORARILY_REJECTED:
		mavlink_log_critical(&_mavlink_log_pub, "command temporarily rejected: %u", cmd->command);
		break;

	case vehicle_command_s::VEHICLE_CMD_RESULT_UNSUPPORTED:
		mavlink_log_critical(&_mavlink_log_pub, "command unsupported: %u", cmd->command);
		break;

	default:
		break;
	}
}

void
Humming::task_main_trampoline(int argc, char *argv[])
{
	humming::g_humming->task_main();
}

void 
Humming::ack_vehicle_command(orb_advert_t &vehicle_command_ack_pub, struct vehicle_command_s &cmd, uint32_t result)
{
	vehicle_command_ack_s vehicle_command_ack = {
		.timestamp = hrt_absolute_time(),
		.result_param2 = 0,
		.command = cmd.command,
		.result = (uint8_t)result,
		.from_external = 0,
		.result_param1 = 0,
		.target_system = cmd.source_system,
		.target_component = cmd.source_component
	};

	if (vehicle_command_ack_pub == nullptr) {
		vehicle_command_ack_pub = orb_advertise_queue(ORB_ID(vehicle_command_ack), &vehicle_command_ack,
	    vehicle_command_ack_s::ORB_QUEUE_LENGTH);
	} else {
		orb_publish(ORB_ID(vehicle_command_ack), vehicle_command_ack_pub, &vehicle_command_ack);
	}
} 

static void usage()
{
	errx(1, "usage: humming {start|stop|status}");
}

int humming_main(int argc, char *argv[])
{
	if (argc < 2) {
		usage();
	}

	if (!strcmp(argv[1], "start")) {

		if (humming::g_humming != nullptr) {
			errx(1, "already running");
		}

		humming::g_humming = new Humming;

		if (humming::g_humming == nullptr) {
			errx(1, "alloc failed");
		}

		if (OK != humming::g_humming->start()) {
			delete humming::g_humming;
			humming::g_humming = nullptr;
			err(1, "start failed");
		}

		return 0;
	}

	if (humming::g_humming == nullptr) {
		errx(1, "not running");
	}

	if (!strcmp(argv[1], "stop")) {
		delete humming::g_humming;
		humming::g_humming = nullptr;

	} else if (!strcmp(argv[1], "status")) {
		humming::g_humming->status();

	} else {
		usage();
	}

	return 0;
}
