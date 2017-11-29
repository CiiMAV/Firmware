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
 * @file humming_params.c
 * Humming parameters
 *
 */

#include <px4_config.h>
#include <systemlib/param/param.h>

/**
 * Actuator 1 length 
 *
 * @max 20
 * @unit cm 
 * @decimal 1
 * @group Humming
 */
PARAM_DEFINE_FLOAT(ACT_1_LEN, 10.0f);

/**
 * Actuator 2 length 
 *
 * @max 20
 * @unit cm 
 * @decimal 1
 * @group Humming
 */
PARAM_DEFINE_FLOAT(ACT_2_LEN, 10.0f);

/**
 * Actuator 3 length 
 *
 * @max 20
 * @unit cm 
 * @decimal 1
 * @group Humming
 */
PARAM_DEFINE_FLOAT(ACT_3_LEN, 10.0f);

/**
 * Actuator 4 length 
 *
 * @max 20
 * @unit cm 
 * @decimal 1
 * @group Humming
 */
PARAM_DEFINE_FLOAT(ACT_4_LEN, 10.0f);

/**
 * Actuator 5 length 
 *
 * @max 20
 * @unit cm 
 * @decimal 1
 * @group Humming
 */
PARAM_DEFINE_FLOAT(ACT_5_LEN, 10.0f);

/**
 * Actuator 6 length 
 *
 * @max 20
 * @unit cm 
 * @decimal 1
 * @group Humming
 */
PARAM_DEFINE_FLOAT(ACT_6_LEN, 10.0f);

/**
 * Actuator 1 speed
 *
 * @max 10
 * @unit cm/s 
 * @decimal 1
 * @group Humming
 */
PARAM_DEFINE_FLOAT(ACT_1_SPE, 10.0f);

/**
 * Actuator 2 speed
 *
 * @max 10
 * @unit cm/s 
 * @decimal 1
 * @group Humming
 */
PARAM_DEFINE_FLOAT(ACT_2_SPE, 10.0f);

/**
 * Actuator 3 speed
 *
 * @max 10
 * @unit cm/s 
 * @decimal 1
 * @group Humming
 */
PARAM_DEFINE_FLOAT(ACT_3_SPE, 10.0f);

/**
 * Actuator 4 speed
 *
 * @max 10
 * @unit cm/s 
 * @decimal 1
 * @group Humming
 */
PARAM_DEFINE_FLOAT(ACT_4_SPE, 10.0f);

/**
 * Actuator 5 speed
 *
 * @max 10
 * @unit cm/s 
 * @decimal 1
 * @group Humming
 */
PARAM_DEFINE_FLOAT(ACT_5_SPE, 10.0f);

/**
 * Actuator 6 speed
 *
 * @max 10
 * @unit cm/s 
 * @decimal 1
 * @group Humming
 */
PARAM_DEFINE_FLOAT(ACT_6_SPE, 10.0f);
