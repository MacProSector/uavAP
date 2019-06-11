////////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2018 University of Illinois Board of Trustees
//
// This file is part of uavAP.
//
// uavAP is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// uavAP is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.
////////////////////////////////////////////////////////////////////////////////
/*
 * L1AdaptiveCascade.cpp
 *
 *  Created on: Jun 5, 2019
 *      Author: simonyu
 */

#include "uavAP/Core/SensorData.h"
#include "uavAP/MissionControl/ManeuverPlanner/Override.h"
#include "uavAP/FlightControl/Controller/AdaptiveController/L1AdaptiveController/detail/L1AdaptiveCascade.h"

L1AdaptiveCascade::L1AdaptiveCascade(SensorData& sensorData, ControllerTarget& controllerTarget,
		ControllerOutput& controllerOutput) :
		sensorData_(sensorData), controllerTarget_(controllerTarget), controlEnvironment_(
				&sensorData.timestamp), hardRollSaturation_(30.0), hardRollRateSaturation_(30.0), hardPitchSaturation_(
				30.0), hardPitchRateSaturation_(30.0), rollSaturation_(30.0), rollRateSaturation_(
				30.0), rollOutSaturation_(1.0), pitchSaturation_(30.0), pitchRateSaturation_(30.0), pitchOutSaturation_(
				1.0), yawOutSaturation_(1.0), throttleOutSaturation_(1.0), beta_(0), rollTarget_(0)
{
	PIDParameter defaultParameter;
	defaultParameter.kp = 1;

	/* Roll Control */
	double* rollInputValue = &sensorData.attitude[0];
	double* rollRateInputValue = &sensorData.angularRate[0];
	double* rollTargetValue = &rollTarget_;

	auto rollInput = controlEnvironment_.addInput<double>(rollInputValue);
	auto rollRateInput = controlEnvironment_.addInput<double>(rollRateInputValue);
	auto rollTarget = controlEnvironment_.addInput<double>(rollTargetValue);

	rollTargetSaturation_ = controlEnvironment_.addSaturation<double>(rollTarget,
			degToRad(-rollSaturation_), degToRad(rollSaturation_), degToRad(-hardRollSaturation_),
			degToRad(hardRollSaturation_));

	auto rollPID = controlEnvironment_.addPID<double>(rollInput, rollTargetSaturation_,
			rollRateInput, defaultParameter);

	/* Roll Rate Control */
	rollRateTargetSaturation_ = controlEnvironment_.addSaturation<double>(rollPID,
			degToRad(-rollRateSaturation_), degToRad(rollRateSaturation_),
			degToRad(-hardRollRateSaturation_), degToRad(hardRollRateSaturation_));

	auto rollRatePID = controlEnvironment_.addPID<double>(rollRateInput, rollRateTargetSaturation_,
			defaultParameter);

	/* Roll Output */
	double* rollOutputValue = &controllerOutput.rollOutput;

	rollOutputSaturation_ = controlEnvironment_.addSaturation<double>(rollRatePID, -1, 1);

	auto rollOutput = controlEnvironment_.addOutput<double, double>(rollOutputSaturation_,
			rollOutputValue);

	/* Climb Angle Control */
	double* aoaInputValue = &sensorData.angleOfAttack;
	double* pitchInputValue = &sensorData.attitude[1];
	double* climbAngleTargetValue = &controllerTarget.climbAngle;

	auto aoaInput = controlEnvironment_.addInput<double>(aoaInputValue);
	auto pitchInput = controlEnvironment_.addInput<double>(pitchInputValue);
	auto climbAngleTarget = controlEnvironment_.addInput<double>(climbAngleTargetValue);

	auto climbAngleSum = controlEnvironment_.addSum<double, double, double>(pitchInput, aoaInput,
			false);

	auto climbAnglePID = controlEnvironment_.addPID<double>(climbAngleSum, climbAngleTarget,
			defaultParameter);

	/* Pitch Control */
	double* pitchRateInputValue = &sensorData.angularRate[1];

	auto pitchRateInput = controlEnvironment_.addInput<double>(pitchRateInputValue);

	pitchTargetSaturation_ = controlEnvironment_.addSaturation<double>(climbAnglePID,
			degToRad(-pitchSaturation_), degToRad(pitchSaturation_),
			degToRad(-hardPitchSaturation_), degToRad(hardPitchSaturation_));

	auto pitchPID = controlEnvironment_.addPID<double>(pitchInput, pitchTargetSaturation_,
			pitchRateInput, defaultParameter);

	/* Pitch Rate Control */
	pitchRateTargetSaturation_ = controlEnvironment_.addSaturation<double>(pitchPID,
			degToRad(-pitchRateSaturation_), degToRad(pitchRateSaturation_),
			degToRad(-hardPitchRateSaturation_), degToRad(hardPitchRateSaturation_));

	auto pitchRatePID = controlEnvironment_.addPID<double>(pitchRateInput,
			pitchRateTargetSaturation_, defaultParameter);

	/* Pitch Output */
	double* pitchOutputValue = &controllerOutput.pitchOutput;

	pitchOutputSaturation_ = controlEnvironment_.addSaturation<double>(pitchRatePID, -1, 1);

	auto pitchOutput = controlEnvironment_.addOutput<double, double>(pitchOutputSaturation_,
			pitchOutputValue);

	/* Beta Control */
	double* betaInputValue = &beta_;

	auto betaInput = controlEnvironment_.addInput<double>(betaInputValue);
	auto betaTarget = controlEnvironment_.addConstant<double>(0);
	auto betaPID = controlEnvironment_.addPID<double>(betaInput, betaTarget, defaultParameter);
	auto betaPIDInverted = controlEnvironment_.addGain<double, double, double>(betaPID, -1);

	/* Yaw Output */
	double* yawOutputValue = &controllerOutput.yawOutput;

	yawOutputSaturation_ = controlEnvironment_.addSaturation<double>(betaPIDInverted, -1, 1);

	auto yawOutput = controlEnvironment_.addOutput<double, double>(yawOutputSaturation_,
			yawOutputValue);

	/* Velocity Control */
	double* velocityInputValue = &sensorData.airSpeed;
	double* accelerationInputValue = &sensorData.acceleration[0];
	double* velocityTargetValue = &controllerTarget.velocity;

	auto velocityInput = controlEnvironment_.addInput<double>(velocityInputValue);
	auto accelerationInput = controlEnvironment_.addInput<double>(accelerationInputValue);
	auto velocityTarget = controlEnvironment_.addInput<double>(velocityTargetValue);

	auto velocityPID = controlEnvironment_.addPID<double>(velocityInput, velocityTarget,
			accelerationInput, defaultParameter);

	auto velocityOffset = controlEnvironment_.addConstant<double>(1);
	auto velocitySum = controlEnvironment_.addSum<double, double, double>(velocityPID,
			velocityOffset, false);

	/* Throttle Output */
	double* throttleOutputValue = &controllerOutput.throttleOutput;

	throttleOutputSaturation_ = controlEnvironment_.addSaturation<double>(velocitySum, -1, 1);

	auto throttleOutput = controlEnvironment_.addOutput<double, double>(throttleOutputSaturation_,
			throttleOutputValue);

	pids_.insert(std::make_pair(PIDs::ROLL, rollPID));
	pids_.insert(std::make_pair(PIDs::ROLL_RATE, rollRatePID));
	pids_.insert(std::make_pair(PIDs::CLIMB_ANGLE, climbAnglePID));
	pids_.insert(std::make_pair(PIDs::PITCH, pitchPID));
	pids_.insert(std::make_pair(PIDs::PITCH_RATE, pitchRatePID));
	pids_.insert(std::make_pair(PIDs::VELOCITY, velocityPID));
	pids_.insert(std::make_pair(PIDs::RUDDER, betaPID));

	outputs_.insert(std::make_pair(ControllerOutputs::ROLL, rollOutput));
	outputs_.insert(std::make_pair(ControllerOutputs::PITCH, pitchOutput));
	outputs_.insert(std::make_pair(ControllerOutputs::YAW, yawOutput));
	outputs_.insert(std::make_pair(ControllerOutputs::THROTTLE, throttleOutput));

	outputWaveforms_.insert(std::make_pair(ControllerOutputsWaveforms::ROLL, rollOutput));
	outputWaveforms_.insert(std::make_pair(ControllerOutputsWaveforms::PITCH, pitchOutput));
	outputWaveforms_.insert(std::make_pair(ControllerOutputsWaveforms::YAW, yawOutput));
	outputWaveforms_.insert(std::make_pair(ControllerOutputsWaveforms::THROTTLE, throttleOutput));

	saturations_.insert(std::make_pair(ControllerConstraints::ROLL, rollTargetSaturation_));
	saturations_.insert(
			std::make_pair(ControllerConstraints::ROLL_RATE, rollRateTargetSaturation_));
	saturations_.insert(std::make_pair(ControllerConstraints::ROLL_OUTPUT, rollOutputSaturation_));
	saturations_.insert(std::make_pair(ControllerConstraints::PITCH, pitchTargetSaturation_));
	saturations_.insert(
			std::make_pair(ControllerConstraints::PITCH_RATE, pitchRateTargetSaturation_));
	saturations_.insert(
			std::make_pair(ControllerConstraints::PITCH_OUTPUT, pitchOutputSaturation_));
	saturations_.insert(std::make_pair(ControllerConstraints::YAW_OUTPUT, yawOutputSaturation_));
	saturations_.insert(
			std::make_pair(ControllerConstraints::THROTTLE_OUTPUT, throttleOutputSaturation_));
}

bool
L1AdaptiveCascade::configure(const boost::property_tree::ptree& config)
{
	PropertyMapper pm(config);
	PIDParameter params;
	boost::property_tree::ptree pidConfig;

	pm.add<double>("hard_roll_constraint", hardRollSaturation_, false);
	pm.add<double>("hard_pitch_constraint", hardPitchSaturation_, false);
	pm.add<double>("hard_roll_rate_constraint", hardRollRateSaturation_, false);
	pm.add<double>("hard_pitch_rate_constraint", hardPitchRateSaturation_, false);

	pm.add<double>("roll_constraint", rollSaturation_, false);
	pm.add<double>("pitch_constraint", pitchSaturation_, false);
	pm.add<double>("roll_rate_constraint", rollRateSaturation_, false);
	pm.add<double>("pitch_rate_constraint", pitchRateSaturation_, false);

	pm.add("pids", pidConfig, false);

	rollTargetSaturation_->setHardSaturationValue(degToRad(hardRollSaturation_));
	pitchTargetSaturation_->setHardSaturationValue(degToRad(hardPitchSaturation_));
	rollRateTargetSaturation_->setHardSaturationValue(degToRad(hardRollRateSaturation_));
	pitchRateTargetSaturation_->setHardSaturationValue(degToRad(hardPitchRateSaturation_));

	rollTargetSaturation_->setSaturationValue(degToRad(rollSaturation_));
	pitchTargetSaturation_->setSaturationValue(degToRad(pitchSaturation_));
	rollRateTargetSaturation_->setSaturationValue(degToRad(rollRateSaturation_));
	pitchRateTargetSaturation_->setSaturationValue(degToRad(pitchRateSaturation_));

	for (auto it : pidConfig)
	{
		if (!params.configure(it.second))
		{
			APLOG_ERROR << "L1AdaptiveCascade: Invalid PID Configuration " << it.first;
			continue;
		}

		tunePID(EnumMap<PIDs>::convert(it.first), params);
	}

	return true;
}

PIDStati
L1AdaptiveCascade::getPIDStatus() const
{
	PIDStati status;

	for (const auto& it : pids_)
	{
		status.insert(std::make_pair(it.first, it.second->getStatus()));
	}

	return status;
}

void
L1AdaptiveCascade::evaluate()
{
	calculateControl();
	controlEnvironment_.evaluate();
}

bool
L1AdaptiveCascade::tunePID(PIDs pid, const PIDParameter& parameter)
{
	auto it = pids_.find(pid);

	if (it == pids_.end())
	{
		APLOG_ERROR << "L1AdaptiveCascade: Unknown PID " << static_cast<int>(pid);
		return false;
	}

	it->second->setParameter(parameter);

	return true;
}

void
L1AdaptiveCascade::overrideCascade(const Override& override)
{
	for (auto& it : pids_)
	{
		it.second->disableOverride();
	}

	for (auto& it : outputs_)
	{
		it.second->disableOverride();
	}

	for (auto& it : saturations_)
	{
		it.second->disableOverride();
	}

	if (override.isEmpty())
	{
		return;
	}

	for (const auto& it : override.pid)
	{
		if (auto pid = findInMap(pids_, it.first))
		{
			pid->second->overrideTarget(it.second);
		}
	}

	for (const auto& it : override.waveform)
	{
		if (auto out = findInMap(outputWaveforms_, it.first))
		{
			out->second->setWaveform(it.second);
			out->second->setPeriod(override.period);
			out->second->setPhase(override.phase);
		}
	}

	for (const auto& it : override.output)
	{
		if (auto out = findInMap(outputs_, it.first))
		{
			out->second->overrideOutput(it.second);
		}
	}

	for (const auto& it : override.constraint)
	{
		if (auto saturation = findInMap(saturations_, it.first))
		{
			saturation->second->overrideSaturationValue(it.second);
		}
	}
}

void
L1AdaptiveCascade::calculateControl()
{
	Eigen::Matrix3d rotationMatrix;
	Vector3 velocityBody;
	double velocityBodyTotal;
	double velocityBodyLateral;
	double roll = sensorData_.attitude.x();
	double pitch = -sensorData_.attitude.y();
	double yaw = sensorData_.attitude.z();
	double gravity = 9.81;

	rotationMatrix = Eigen::AngleAxisd(-roll, Vector3::UnitX())
			* Eigen::AngleAxisd(-pitch, Vector3::UnitY())
			* Eigen::AngleAxisd(-yaw, Vector3::UnitZ());

	velocityBody = rotationMatrix * sensorData_.velocity;
	velocityBodyTotal = velocityBody.norm();
	velocityBodyLateral = velocityBody[1];

	beta_ = -asin(velocityBodyLateral / velocityBodyTotal);
	rollTarget_ = -atan2(velocityBodyTotal * controllerTarget_.yawRate, gravity);
}
