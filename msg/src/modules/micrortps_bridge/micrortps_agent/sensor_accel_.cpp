// Copyright 2016 Proyectos y Sistemas de Mantenimiento SL (eProsima).
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/*! 
 * @file sensor_accel_.cpp
 * This source file contains the definition of the described types in the IDL file.
 *
 * This file was generated by the tool gen.
 */

#ifdef _WIN32
// Remove linker warning LNK4221 on Visual Studio
namespace { char dummy; }
#endif

#include "sensor_accel_.h"

#include <fastcdr/Cdr.h>

#include <fastcdr/exceptions/BadParamException.h>
using namespace eprosima::fastcdr::exception;

#include <utility>

sensor_accel_::sensor_accel_()
{
    m_integral_dt = 0;
    m_error_count = 0;
    m_x = 0.0;
    m_y = 0.0;
    m_z = 0.0;
    m_x_integral = 0.0;
    m_y_integral = 0.0;
    m_z_integral = 0.0;
    m_temperature = 0.0;
    m_range_m_s2 = 0.0;
    m_scaling = 0.0;
    m_device_id = 0;
    m_x_raw = 0;
    m_y_raw = 0;
    m_z_raw = 0;
    m_temperature_raw = 0;
}

sensor_accel_::~sensor_accel_()
{
}

sensor_accel_::sensor_accel_(const sensor_accel_ &x)
{
    m_integral_dt = x.m_integral_dt;
    m_error_count = x.m_error_count;
    m_x = x.m_x;
    m_y = x.m_y;
    m_z = x.m_z;
    m_x_integral = x.m_x_integral;
    m_y_integral = x.m_y_integral;
    m_z_integral = x.m_z_integral;
    m_temperature = x.m_temperature;
    m_range_m_s2 = x.m_range_m_s2;
    m_scaling = x.m_scaling;
    m_device_id = x.m_device_id;
    m_x_raw = x.m_x_raw;
    m_y_raw = x.m_y_raw;
    m_z_raw = x.m_z_raw;
    m_temperature_raw = x.m_temperature_raw;
}

sensor_accel_::sensor_accel_(sensor_accel_ &&x)
{
    m_integral_dt = x.m_integral_dt;
    m_error_count = x.m_error_count;
    m_x = x.m_x;
    m_y = x.m_y;
    m_z = x.m_z;
    m_x_integral = x.m_x_integral;
    m_y_integral = x.m_y_integral;
    m_z_integral = x.m_z_integral;
    m_temperature = x.m_temperature;
    m_range_m_s2 = x.m_range_m_s2;
    m_scaling = x.m_scaling;
    m_device_id = x.m_device_id;
    m_x_raw = x.m_x_raw;
    m_y_raw = x.m_y_raw;
    m_z_raw = x.m_z_raw;
    m_temperature_raw = x.m_temperature_raw;
}

sensor_accel_& sensor_accel_::operator=(const sensor_accel_ &x)
{
    m_integral_dt = x.m_integral_dt;
    m_error_count = x.m_error_count;
    m_x = x.m_x;
    m_y = x.m_y;
    m_z = x.m_z;
    m_x_integral = x.m_x_integral;
    m_y_integral = x.m_y_integral;
    m_z_integral = x.m_z_integral;
    m_temperature = x.m_temperature;
    m_range_m_s2 = x.m_range_m_s2;
    m_scaling = x.m_scaling;
    m_device_id = x.m_device_id;
    m_x_raw = x.m_x_raw;
    m_y_raw = x.m_y_raw;
    m_z_raw = x.m_z_raw;
    m_temperature_raw = x.m_temperature_raw;
    
    return *this;
}

sensor_accel_& sensor_accel_::operator=(sensor_accel_ &&x)
{
    m_integral_dt = x.m_integral_dt;
    m_error_count = x.m_error_count;
    m_x = x.m_x;
    m_y = x.m_y;
    m_z = x.m_z;
    m_x_integral = x.m_x_integral;
    m_y_integral = x.m_y_integral;
    m_z_integral = x.m_z_integral;
    m_temperature = x.m_temperature;
    m_range_m_s2 = x.m_range_m_s2;
    m_scaling = x.m_scaling;
    m_device_id = x.m_device_id;
    m_x_raw = x.m_x_raw;
    m_y_raw = x.m_y_raw;
    m_z_raw = x.m_z_raw;
    m_temperature_raw = x.m_temperature_raw;
    
    return *this;
}

size_t sensor_accel_::getMaxCdrSerializedSize(size_t current_alignment)
{
    size_t initial_alignment = current_alignment;
            
    current_alignment += 8 + eprosima::fastcdr::Cdr::alignment(current_alignment, 8);

    current_alignment += 8 + eprosima::fastcdr::Cdr::alignment(current_alignment, 8);

    current_alignment += 4 + eprosima::fastcdr::Cdr::alignment(current_alignment, 4);

    current_alignment += 4 + eprosima::fastcdr::Cdr::alignment(current_alignment, 4);

    current_alignment += 4 + eprosima::fastcdr::Cdr::alignment(current_alignment, 4);

    current_alignment += 4 + eprosima::fastcdr::Cdr::alignment(current_alignment, 4);

    current_alignment += 4 + eprosima::fastcdr::Cdr::alignment(current_alignment, 4);

    current_alignment += 4 + eprosima::fastcdr::Cdr::alignment(current_alignment, 4);

    current_alignment += 4 + eprosima::fastcdr::Cdr::alignment(current_alignment, 4);

    current_alignment += 4 + eprosima::fastcdr::Cdr::alignment(current_alignment, 4);

    current_alignment += 4 + eprosima::fastcdr::Cdr::alignment(current_alignment, 4);

    current_alignment += 4 + eprosima::fastcdr::Cdr::alignment(current_alignment, 4);

    current_alignment += 2 + eprosima::fastcdr::Cdr::alignment(current_alignment, 2);

    current_alignment += 2 + eprosima::fastcdr::Cdr::alignment(current_alignment, 2);

    current_alignment += 2 + eprosima::fastcdr::Cdr::alignment(current_alignment, 2);

    current_alignment += 2 + eprosima::fastcdr::Cdr::alignment(current_alignment, 2);


    return current_alignment - initial_alignment;
}

size_t sensor_accel_::getCdrSerializedSize(const sensor_accel_& data, size_t current_alignment)
{
    size_t initial_alignment = current_alignment;
            
    current_alignment += 8 + eprosima::fastcdr::Cdr::alignment(current_alignment, 8);

    current_alignment += 8 + eprosima::fastcdr::Cdr::alignment(current_alignment, 8);

    current_alignment += 4 + eprosima::fastcdr::Cdr::alignment(current_alignment, 4);

    current_alignment += 4 + eprosima::fastcdr::Cdr::alignment(current_alignment, 4);

    current_alignment += 4 + eprosima::fastcdr::Cdr::alignment(current_alignment, 4);

    current_alignment += 4 + eprosima::fastcdr::Cdr::alignment(current_alignment, 4);

    current_alignment += 4 + eprosima::fastcdr::Cdr::alignment(current_alignment, 4);

    current_alignment += 4 + eprosima::fastcdr::Cdr::alignment(current_alignment, 4);

    current_alignment += 4 + eprosima::fastcdr::Cdr::alignment(current_alignment, 4);

    current_alignment += 4 + eprosima::fastcdr::Cdr::alignment(current_alignment, 4);

    current_alignment += 4 + eprosima::fastcdr::Cdr::alignment(current_alignment, 4);

    current_alignment += 4 + eprosima::fastcdr::Cdr::alignment(current_alignment, 4);

    current_alignment += 2 + eprosima::fastcdr::Cdr::alignment(current_alignment, 2);

    current_alignment += 2 + eprosima::fastcdr::Cdr::alignment(current_alignment, 2);

    current_alignment += 2 + eprosima::fastcdr::Cdr::alignment(current_alignment, 2);

    current_alignment += 2 + eprosima::fastcdr::Cdr::alignment(current_alignment, 2);


    return current_alignment - initial_alignment;
}

void sensor_accel_::serialize(eprosima::fastcdr::Cdr &scdr) const
{
    scdr << m_integral_dt;
    scdr << m_error_count;
    scdr << m_x;
    scdr << m_y;
    scdr << m_z;
    scdr << m_x_integral;
    scdr << m_y_integral;
    scdr << m_z_integral;
    scdr << m_temperature;
    scdr << m_range_m_s2;
    scdr << m_scaling;
    scdr << m_device_id;
    scdr << m_x_raw;
    scdr << m_y_raw;
    scdr << m_z_raw;
    scdr << m_temperature_raw;
}

void sensor_accel_::deserialize(eprosima::fastcdr::Cdr &dcdr)
{
    dcdr >> m_integral_dt;
    dcdr >> m_error_count;
    dcdr >> m_x;
    dcdr >> m_y;
    dcdr >> m_z;
    dcdr >> m_x_integral;
    dcdr >> m_y_integral;
    dcdr >> m_z_integral;
    dcdr >> m_temperature;
    dcdr >> m_range_m_s2;
    dcdr >> m_scaling;
    dcdr >> m_device_id;
    dcdr >> m_x_raw;
    dcdr >> m_y_raw;
    dcdr >> m_z_raw;
    dcdr >> m_temperature_raw;
}

size_t sensor_accel_::getKeyMaxCdrSerializedSize(size_t current_alignment)
{
	size_t current_align = current_alignment;
            

















    return current_align;
}

bool sensor_accel_::isKeyDefined()
{
    return false;
}

void sensor_accel_::serializeKey(eprosima::fastcdr::Cdr &scdr) const
{
	 
	 
	 
	 
	 
	 
	 
	 
	 
	 
	 
	 
	 
	 
	 
	 
}