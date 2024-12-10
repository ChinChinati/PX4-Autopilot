/****************************************************************************
 *
 *   Copyright (c) 2019 PX4 Development Team. All rights reserved.
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
 * @file ControlAllocationSequentialDesaturation.cpp
 *
 * @author Roman Bapst <bapstroman@gmail.com>
 * @author Beat Küng <beat-kueng@gmx.net>
 */

#include "ControlAllocationSequentialDesaturation.hpp"
#include<bits/stdc++.h>

using namespace std;
void
ControlAllocationSequentialDesaturation::allocate()
{
	//Compute new gains if needed
	updatePseudoInverse();

	_prev_actuator_sp = _actuator_sp;

	// ####################################

	_motor_failed_sub.update(&_motor_failed_get);
	_vehicle_torque_sub.update(&_vehicle_torque_get);

	if( _motor_failed_get.motor_failed == 0){
		switch (_param_mc_airmode.get()) {
		case 1:
			mixAirmodeRP();
			break;

		case 2:
			mixAirmodeRPY();
			break;

		default:
			mixAirmodeDisabled();
			// testing();
			break;
		}
		}
		else if( _motor_failed_get.motor_failed == 1){
			motor_failed_1();
			// testing();
		}
		else if( _motor_failed_get.motor_failed == 2){
			motor_failed_2();
		}
		else if( _motor_failed_get.motor_failed == 3){
			motor_failed_3();
		}
		else if( _motor_failed_get.motor_failed == 4){
			motor_failed_4();
		}
	// ####################################

	// switch (_param_mc_airmode.get()) {
	// case 1:
	// 	mixAirmodeRP();
	// 	break;

	// case 2:
	// 	mixAirmodeRPY();
	// 	break;

	// default:
	// 	mixAirmodeDisabled();
	// 	break;
	// }
}

void ControlAllocationSequentialDesaturation::desaturateActuators(
	ActuatorVector &actuator_sp,
	const ActuatorVector &desaturation_vector, bool increase_only)
{
	float gain = computeDesaturationGain(desaturation_vector, actuator_sp);

	if (increase_only && gain < 0.f) {
		return;
	}

	for (int i = 0; i < _num_actuators; i++) {
		actuator_sp(i) += gain * desaturation_vector(i);
	}

	gain = 0.5f * computeDesaturationGain(desaturation_vector, actuator_sp);

	for (int i = 0; i < _num_actuators; i++) {
		actuator_sp(i) += gain * desaturation_vector(i);
	}
}

float ControlAllocationSequentialDesaturation::computeDesaturationGain(const ActuatorVector &desaturation_vector,
		const ActuatorVector &actuator_sp)
{
	float k_min = 0.f;
	float k_max = 0.f;

	for (int i = 0; i < _num_actuators; i++) {
		// Do not use try to desaturate using an actuator with weak effectiveness to avoid large desaturation gains
		if (fabsf(desaturation_vector(i)) < 0.2f) {
			continue;
		}

		if (actuator_sp(i) < _actuator_min(i)) {
			float k = (_actuator_min(i) - actuator_sp(i)) / desaturation_vector(i);

			if (k < k_min) { k_min = k; }

			if (k > k_max) { k_max = k; }
		}

		if (actuator_sp(i) > _actuator_max(i)) {
			float k = (_actuator_max(i) - actuator_sp(i)) / desaturation_vector(i);

			if (k < k_min) { k_min = k; }

			if (k > k_max) { k_max = k; }
		}
	}

	// Reduce the saturation as much as possible
	return k_min + k_max;
}

void
ControlAllocationSequentialDesaturation::mixAirmodeRP()
{
	// Airmode for roll and pitch, but not yaw

	// Mix without yaw
	ActuatorVector thrust_z;

	for (int i = 0; i < _num_actuators; i++) {
		_actuator_sp(i) = _actuator_trim(i) +
				  _mix(i, ControlAxis::ROLL) * (_control_sp(ControlAxis::ROLL) - _control_trim(ControlAxis::ROLL)) +
				  _mix(i, ControlAxis::PITCH) * (_control_sp(ControlAxis::PITCH) - _control_trim(ControlAxis::PITCH)) +
				  _mix(i, ControlAxis::THRUST_X) * (_control_sp(ControlAxis::THRUST_X) - _control_trim(ControlAxis::THRUST_X)) +
				  _mix(i, ControlAxis::THRUST_Y) * (_control_sp(ControlAxis::THRUST_Y) - _control_trim(ControlAxis::THRUST_Y)) +
				  _mix(i, ControlAxis::THRUST_Z) * (_control_sp(ControlAxis::THRUST_Z) - _control_trim(ControlAxis::THRUST_Z));
		thrust_z(i) = _mix(i, ControlAxis::THRUST_Z);
	}

	desaturateActuators(_actuator_sp, thrust_z);

	// Mix yaw independently
	mixYaw();
}

void
ControlAllocationSequentialDesaturation::mixAirmodeRPY()
{
	// Airmode for roll, pitch and yaw

	// Do full mixing
	ActuatorVector thrust_z;
	ActuatorVector yaw;

	for (int i = 0; i < _num_actuators; i++) {
		_actuator_sp(i) = _actuator_trim(i) +
				  _mix(i, ControlAxis::ROLL) * (_control_sp(ControlAxis::ROLL) - _control_trim(ControlAxis::ROLL)) +
				  _mix(i, ControlAxis::PITCH) * (_control_sp(ControlAxis::PITCH) - _control_trim(ControlAxis::PITCH)) +
				  _mix(i, ControlAxis::YAW) * (_control_sp(ControlAxis::YAW) - _control_trim(ControlAxis::YAW)) +
				  _mix(i, ControlAxis::THRUST_X) * (_control_sp(ControlAxis::THRUST_X) - _control_trim(ControlAxis::THRUST_X)) +
				  _mix(i, ControlAxis::THRUST_Y) * (_control_sp(ControlAxis::THRUST_Y) - _control_trim(ControlAxis::THRUST_Y)) +
				  _mix(i, ControlAxis::THRUST_Z) * (_control_sp(ControlAxis::THRUST_Z) - _control_trim(ControlAxis::THRUST_Z));
		thrust_z(i) = _mix(i, ControlAxis::THRUST_Z);
		yaw(i) = _mix(i, ControlAxis::YAW);
	}

	desaturateActuators(_actuator_sp, thrust_z);

	// Unsaturate yaw (in case upper and lower bounds are exceeded)
	// to prioritize roll/pitch over yaw.
	desaturateActuators(_actuator_sp, yaw);
}

void
ControlAllocationSequentialDesaturation::mixAirmodeDisabled()
{
	// Airmode disabled: never allow to increase the thrust to unsaturate a motor

	// Mix without yaw
	ActuatorVector thrust_z;
	ActuatorVector roll;
	ActuatorVector pitch;

	// cout<<_actuator_sp(0)<<" "<<_actuator_sp(1)<<" "<<_actuator_sp(2)<<" "<<_actuator_sp(3)<<endl;
	for (int i = 0; i < _num_actuators; i++) {
		// std::cout<<_mix(i, ControlAxis::ROLL)<<" "<<_mix(i, ControlAxis::PITCH)<<" "<<_mix(i, ControlAxis::YAW)
		// <<" "<<_mix(i, ControlAxis::THRUST_X)<<" "<<_mix(i, ControlAxis::THRUST_Y)<<" "<<_mix(i, ControlAxis::THRUST_Z)<<std::endl;
		_actuator_sp(i) = _actuator_trim(i) +
				  _mix(i, ControlAxis::ROLL) * (_control_sp(ControlAxis::ROLL) - _control_trim(ControlAxis::ROLL)) +
				  _mix(i, ControlAxis::PITCH) * (_control_sp(ControlAxis::PITCH) - _control_trim(ControlAxis::PITCH)) +
				  _mix(i, ControlAxis::THRUST_X) * (_control_sp(ControlAxis::THRUST_X) - _control_trim(ControlAxis::THRUST_X)) +
				  _mix(i, ControlAxis::THRUST_Y) * (_control_sp(ControlAxis::THRUST_Y) - _control_trim(ControlAxis::THRUST_Y)) +
				  _mix(i, ControlAxis::THRUST_Z) * (_control_sp(ControlAxis::THRUST_Z) - _control_trim(ControlAxis::THRUST_Z));
		thrust_z(i) = _mix(i, ControlAxis::THRUST_Z);
		roll(i) = _mix(i, ControlAxis::ROLL);
		pitch(i) = _mix(i, ControlAxis::PITCH);
	}
	// std::cout<<std::endl;
	// only reduce thrust
	desaturateActuators(_actuator_sp, thrust_z, true);
	// cout<<_control_sp(ControlAxis::THRUST_Z)<<" control_sp"<<endl;
	// Reduce roll/pitch acceleration if needed to unsaturate
	desaturateActuators(_actuator_sp, roll);
	desaturateActuators(_actuator_sp, pitch);
	// float _actuator_sp_arr[4];
	// _actuator_sp_arr[0] = _actuator_sp(0);
	// _actuator_sp_arr[1] = _actuator_sp(1);
	// _actuator_sp_arr[2] = _actuator_sp(2);
	// _actuator_sp_arr[3] = _actuator_sp(3);

	actuator_speed_s actuator_speed{};

	// torques.copyTo(computed_torque.computed_torque);
	actuator_speed.timestamp = hrt_absolute_time();
	actuator_speed.actuator_speed_sp[0] = _actuator_sp(0);
	actuator_speed.actuator_speed_sp[1] = _actuator_sp(1);
	actuator_speed.actuator_speed_sp[2] = _actuator_sp(2);
	actuator_speed.actuator_speed_sp[3] = _actuator_sp(3);
	// cout<<"here"<<endl;
	_actuator_speed_pub.publish(actuator_speed);

	// Mix yaw independently
	mixYaw();
}

void
ControlAllocationSequentialDesaturation::mixYaw()
{
	// Add yaw to outputs
	ActuatorVector yaw;
	ActuatorVector thrust_z;

	for (int i = 0; i < _num_actuators; i++) {
		_actuator_sp(i) += _mix(i, ControlAxis::YAW) * (_control_sp(ControlAxis::YAW) - _control_trim(ControlAxis::YAW));
		yaw(i) = _mix(i, ControlAxis::YAW);
		thrust_z(i) = _mix(i, ControlAxis::THRUST_Z);
	}

	// Change yaw acceleration to unsaturate the outputs if needed (do not change roll/pitch),
	// and allow some yaw response at maximum thrust
	ActuatorVector max_prev = _actuator_max;
	_actuator_max += (_actuator_max - _actuator_min) * 0.15f;
	desaturateActuators(_actuator_sp, yaw);
	_actuator_max = max_prev;

	// reduce thrust only
	desaturateActuators(_actuator_sp, thrust_z, true);
}

// ###################################################
// MIX MATRIX
// -0.495384 0.707107 0.765306 -1
// 0.495384 -0.707107 1 -1
// 0.495384 0.707107 -0.765306 -1
// -0.495384 -0.707107 -1 -1

void
ControlAllocationSequentialDesaturation:: testing(){
	_actuator_sp(0) = .0f;
	_actuator_sp(1) = .0f;
	_actuator_sp(2) = .65f;
	_actuator_sp(3) = .65f;

	// cout<<"testing"<<endl;
	// cout<<_actuator_sp(0)<<" "<<_actuator_sp(1)<<" "<<_actuator_sp(2)<<" "<<_actuator_sp(3)<<endl;
	// cout<<endl;

}
void
ControlAllocationSequentialDesaturation:: motor_failed_1(){
	matrix::Matrix<float, 3, 3> _mix_updated;

	_mix_updated(0,0) = 0.41025641026/0.6349;
	_mix_updated(1,0) = 0/0.6349;
	_mix_updated(2,0) = -0.41025641026/0.6349;
	_mix_updated(0,1) = -0.58559702124/0.6349;
	_mix_updated(1,1) = 0.507743082/0.6349;
	_mix_updated(2,1) = -0.07692307692/0.6349;
	_mix_updated(0,2) = 0.01179487179/0.0591;
	_mix_updated(1,2) = -0.07692307692/0.0591;
	_mix_updated(2,2) = -0.08871794872/0.0591;

	float t_bias = 0.5;
	float torq_factor = 1.3;
	float t_limit = 10;
	tx = _vehicle_torque_get.tx;
	ty = _vehicle_torque_get.ty;
	if(abs(tx)>t_limit)
		tx = tx/abs(tx)*t_limit;
	if( abs(ty)>t_limit)
		ty = ty/abs(ty)*t_limit;

	for (int i = 0; i < 4; i++) {
		if(i==0){
			_actuator_sp(i) = 0;
			continue;
		}

		_actuator_sp(i) = (_mix_updated(i-1, 0)*tx*t_bias) + (_mix_updated(i-1, 1)*ty*t_bias) + (_mix_updated(i-1, 2)*_control_sp(ControlAxis::THRUST_Z)*torq_factor);

	}
	if(_actuator_sp.max()>1)
	_actuator_sp /= _actuator_sp.max();
	// cout<<_actuator_sp(0)<<" "<<_actuator_sp(1)<<" "<<_actuator_sp(2)<<" "<<_actuator_sp(3)<<endl;

}

void
ControlAllocationSequentialDesaturation::motor_failed_2(){
	matrix::Matrix<float, 3, 3> _mix_updated;
	_mix_updated(0,0) = -0.31397174254/0.555098;
	_mix_updated(1,0) = 0.31397174254/0.555098;
	_mix_updated(2,0) = 0/0.555098;
	_mix_updated(0,1) = 0.44816098564/0.555098;
	_mix_updated(1,1) = 0.05958209636/0.555098;
	_mix_updated(2,1) = -0.507743082/0.555098;
	_mix_updated(0,2) = -0.0090266876/0.0512821;
	_mix_updated(1,2) = -0.06789638932/0.0512821;
	_mix_updated(2,2) = -0.07692307692/0.0512821;

	float t_bias = 0.5;
	float torq_factor = 1.3;
	float t_limit = 10;
	tx = _vehicle_torque_get.tx;
	ty = _vehicle_torque_get.ty;
	if(abs(tx)>t_limit)
		tx = tx/abs(tx)*t_limit;
	if( abs(ty)>t_limit)
		ty = ty/abs(ty)*t_limit;

	_actuator_sp(0) = (_mix_updated(0, 0)*tx*t_bias) + (_mix_updated(0, 1)*ty*t_bias) + (_mix_updated(0, 2)*_control_sp(ControlAxis::THRUST_Z)*torq_factor);
	_actuator_sp(1) = 0;
	_actuator_sp(2) = (_mix_updated(1, 0)*tx*t_bias) + (_mix_updated(1, 1)*ty*t_bias) + (_mix_updated(1, 2)*_control_sp(ControlAxis::THRUST_Z)*torq_factor);
	_actuator_sp(3) = (_mix_updated(2, 0)*tx*t_bias) + (_mix_updated(2, 1)*ty*t_bias) + (_mix_updated(2, 2)*_control_sp(ControlAxis::THRUST_Z)*torq_factor);


	cout<<_actuator_sp(0)<<" "<<_actuator_sp(1)<<" "<<_actuator_sp(2)<<" "<<_actuator_sp(3)<<endl<<endl;
}

void
ControlAllocationSequentialDesaturation::motor_failed_3(){
	matrix::Matrix<float, 3, 3> _mix_updated;
	_mix_updated(0,0) = 0/0.636023;
	_mix_updated(1,0) = 0.41025641026/0.636023;
	_mix_updated(2,0) = -0.41025641026/0.636023;
	_mix_updated(0,1) = 0.507743082/0.636023;
	_mix_updated(1,1) = 0.07785393924/0.636023;
	_mix_updated(2,1) = -0.58559702124/0.636023;
	_mix_updated(0,2) = -0.07692307692/0.0591453;
	_mix_updated(1,2) = -0.08871794872/0.0591453;
	_mix_updated(2,2) = 0.01179487179/0.0591453;

	float t_bias = 0.5;
	float torq_factor = 1.3;
	float t_limit = 10;
	tx = _vehicle_torque_get.tx;
	ty = _vehicle_torque_get.ty;
	if(abs(tx)>t_limit)
		tx = tx/abs(tx)*t_limit;
	if( abs(ty)>t_limit)
		ty = ty/abs(ty)*t_limit;

	_actuator_sp(0) = (_mix_updated(0, 0)*tx*t_bias) + (_mix_updated(0, 1)*ty*t_bias) + (_mix_updated(0, 2)*_control_sp(ControlAxis::THRUST_Z)*torq_factor);
	_actuator_sp(1) = (_mix_updated(1, 0)*tx*t_bias) + (_mix_updated(1, 1)*ty*t_bias) + (_mix_updated(1, 2)*_control_sp(ControlAxis::THRUST_Z)*torq_factor);
	_actuator_sp(2) = 0;
	_actuator_sp(3) = (_mix_updated(2, 0)*tx*t_bias) + (_mix_updated(2, 1)*ty*t_bias) + (_mix_updated(2, 2)*_control_sp(ControlAxis::THRUST_Z)*torq_factor);


	cout<<_actuator_sp(0)<<" "<<_actuator_sp(1)<<" "<<_actuator_sp(2)<<" "<<_actuator_sp(3)<<endl<<endl;
}

void
ControlAllocationSequentialDesaturation::motor_failed_4(){
	matrix::Matrix<float, 3, 3> _mix_updated;
	_mix_updated(0,0) = -0.31397174254/0.555098;
	_mix_updated(1,0) = 0/0.555098;
	_mix_updated(2,0) = 0.31397174254/0.555098;
	_mix_updated(0,1) = 0.05958209636/0.555098;
	_mix_updated(1,1) = -0.507743082/0.555098;
	_mix_updated(2,1) = 0.44816098564/0.555098;
	_mix_updated(0,2) = -0.06789638932/0.0512821;
	_mix_updated(1,2) = -0.07692307692/0.0512821;
	_mix_updated(2,2) = -0.0090266876/0.0512821;

	float t_bias = 0.5;
	float torq_factor = 1.3;
	float t_limit = 10;
	tx = _vehicle_torque_get.tx;
	ty = _vehicle_torque_get.ty;
	if(abs(tx)>t_limit)
		tx = tx/abs(tx)*t_limit;
	if( abs(ty)>t_limit)
		ty = ty/abs(ty)*t_limit;

	_actuator_sp(0) = (_mix_updated(0, 0)*tx*t_bias) + (_mix_updated(0, 1)*ty*t_bias) + (_mix_updated(0, 2)*_control_sp(ControlAxis::THRUST_Z)*torq_factor);
	_actuator_sp(1) = (_mix_updated(1, 0)*tx*t_bias) + (_mix_updated(1, 1)*ty*t_bias) + (_mix_updated(1, 2)*_control_sp(ControlAxis::THRUST_Z)*torq_factor);
	_actuator_sp(2) = (_mix_updated(2, 0)*tx*t_bias) + (_mix_updated(2, 1)*ty*t_bias) + (_mix_updated(2, 2)*_control_sp(ControlAxis::THRUST_Z)*torq_factor);
	_actuator_sp(3) = 0;


	cout<<_actuator_sp(0)<<" "<<_actuator_sp(1)<<" "<<_actuator_sp(2)<<" "<<_actuator_sp(3)<<endl<<endl;
}
// ###################################################

void
ControlAllocationSequentialDesaturation::updateParameters()
{
	updateParams();
}
