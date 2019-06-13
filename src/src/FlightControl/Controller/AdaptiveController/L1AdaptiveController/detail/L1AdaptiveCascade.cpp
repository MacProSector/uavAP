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
		sensorData_(sensorData), controllerTarget_(controllerTarget), controllerOutput_(
				controllerOutput), controlEnvironment_(&sensorData.timestamp), hardRollSaturation_(
				30.0), hardRollRateSaturation_(30.0), hardPitchSaturation_(30.0), hardPitchRateSaturation_(
				30.0), rollSaturation_(30.0), rollRateSaturation_(30.0), pitchSaturation_(30.0), pitchRateSaturation_(
				30.0), rollOutSaturation_(1.0), pitchOutSaturation_(1.0), yawOutSaturation_(1.0), throttleOutSaturation_(
				1.0), beta_(0), rollTarget_(0)
{
}

bool
L1AdaptiveCascade::configure(const boost::property_tree::ptree& config)
{
	PropertyMapper pm(config);
	boost::property_tree::ptree adaptiveConfig;
	boost::property_tree::ptree pidConfig;
	L1AdaptiveParameter adaptiveParameter;
	PIDParameter pidParameter;

	pm.add<double>("hard_roll_constraint", hardRollSaturation_, false);
	pm.add<double>("hard_pitch_constraint", hardPitchSaturation_, false);
	pm.add<double>("hard_roll_rate_constraint", hardRollRateSaturation_, false);
	pm.add<double>("hard_pitch_rate_constraint", hardPitchRateSaturation_, false);

	pm.add<double>("roll_constraint", rollSaturation_, false);
	pm.add<double>("pitch_constraint", pitchSaturation_, false);
	pm.add<double>("roll_rate_constraint", rollRateSaturation_, false);
	pm.add<double>("pitch_rate_constraint", pitchRateSaturation_, false);

	pm.add("adaptives", adaptiveConfig, false);
	pm.add("pids", pidConfig, false);

	for (const auto& it : adaptiveConfig)
	{
		if (!adaptiveParameter.configure(it.second))
		{
			APLOG_ERROR << "L1AdaptiveCascade: Invalid Adaptive Configuration " << it.first;
			continue;
		}

		adaptiveParameters_.insert(
				std::make_pair(EnumMap<Adaptives>::convert(it.first), adaptiveParameter));
	}

	for (auto it : pidConfig)
	{
		if (!pidParameter.configure(it.second))
		{
			APLOG_ERROR << "L1AdaptiveCascade: Invalid PID Configuration " << it.first;
			continue;
		}

		pidParameters_.insert(std::make_pair(EnumMap<PIDs>::convert(it.first), pidParameter));
	}

	createCascade();

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
L1AdaptiveCascade::createCascade()
{
	L1AdaptiveParameter rollAdaptiveParameter = getParameter(adaptiveParameters_, Adaptives::ROLL);
	L1AdaptiveParameter pitchAdaptiveParameter = getParameter(adaptiveParameters_, Adaptives::PITCH);
	PIDParameter rollPIDParameter = getParameter(pidParameters_, PIDs::ROLL);
	PIDParameter rollRatePIDParameter = getParameter(pidParameters_, PIDs::ROLL_RATE);
	PIDParameter climbAnglePIDParameter = getParameter(pidParameters_, PIDs::CLIMB_ANGLE);
	PIDParameter pitchPIDParameter = getParameter(pidParameters_, PIDs::PITCH);
	PIDParameter pitchRatePIDParameter = getParameter(pidParameters_, PIDs::PITCH_RATE);
	PIDParameter betaPIDParameter = getParameter(pidParameters_, PIDs::RUDDER);
	PIDParameter velocityPIDParameter = getParameter(pidParameters_, PIDs::VELOCITY);

	/* Roll Control */
	double* rollInputValue = &sensorData_.attitude[0];
	double* rollRateInputValue = &sensorData_.angularRate[0];
	double* rollTargetValue = &rollTarget_;

	auto rollInput = controlEnvironment_.addInput<double>(rollInputValue);
	auto rollRateInput = controlEnvironment_.addInput<double>(rollRateInputValue);
	auto rollTarget = controlEnvironment_.addInput<double>(rollTargetValue);

	auto rollTargetSaturation = controlEnvironment_.addSaturation<double>(rollTarget,
			degToRad(-rollSaturation_), degToRad(rollSaturation_), degToRad(-hardRollSaturation_),
			degToRad(hardRollSaturation_));

	auto rollPID = controlEnvironment_.addPID<double>(rollInput, rollTargetSaturation,
			rollRateInput, rollPIDParameter);

	/* Roll Rate Control */
	auto rollRateTargetSaturation = controlEnvironment_.addSaturation<double>(rollPID,
			degToRad(-rollRateSaturation_), degToRad(rollRateSaturation_),
			degToRad(-hardRollRateSaturation_), degToRad(hardRollRateSaturation_));

	auto rollRatePID = controlEnvironment_.addPID<double>(rollRateInput, rollRateTargetSaturation,
			rollRatePIDParameter);

	/* Roll Output */
	double* rollOutputValue = &controllerOutput_.rollOutput;

	auto rollOutputSaturation = controlEnvironment_.addSaturation<double>(rollRatePID, -1, 1);

	auto rollOutput = controlEnvironment_.addOutput<double, double>(rollOutputSaturation,
			rollOutputValue);

	/* Climb Angle Control */
	double* aoaInputValue = &sensorData_.angleOfAttack;
	double* pitchInputValue = &sensorData_.attitude[1];
	double* pitchRateInputValue = &sensorData_.angularRate[1];
	double* climbAngleTargetValue = &controllerTarget_.climbAngle;

	auto aoaInput = controlEnvironment_.addInput<double>(aoaInputValue);
	auto pitchInput = controlEnvironment_.addInput<double>(pitchInputValue);
	auto pitchRateInput = controlEnvironment_.addInput<double>(pitchRateInputValue);
	auto climbAngleTarget = controlEnvironment_.addInput<double>(climbAngleTargetValue);

	auto climbAngleSum = controlEnvironment_.addSum<double, double, double>(pitchInput, aoaInput,
			false);

	auto climbAnglePID = controlEnvironment_.addPID<double>(climbAngleSum, climbAngleTarget,
			climbAnglePIDParameter);

	/* Pitch Control */
	auto pitchTargetSaturation = controlEnvironment_.addSaturation<double>(climbAnglePID,
			degToRad(-pitchSaturation_), degToRad(pitchSaturation_),
			degToRad(-hardPitchSaturation_), degToRad(hardPitchSaturation_));

	auto pitchPID = controlEnvironment_.addPID<double>(pitchInput, pitchTargetSaturation,
			pitchRateInput, pitchPIDParameter);

	/* Pitch Rate Control */
	auto pitchRateTargetSaturation = controlEnvironment_.addSaturation<double>(pitchPID,
			degToRad(-pitchRateSaturation_), degToRad(pitchRateSaturation_),
			degToRad(-hardPitchRateSaturation_), degToRad(hardPitchRateSaturation_));

	auto pitchRatePID = controlEnvironment_.addPID<double>(pitchRateInput,
			pitchRateTargetSaturation, pitchRatePIDParameter);

	/* Pitch Output */
	double* pitchOutputValue = &controllerOutput_.pitchOutput;

	auto pitchOutputSaturation = controlEnvironment_.addSaturation<double>(pitchRatePID, -1, 1);

	auto pitchOutput = controlEnvironment_.addOutput<double, double>(pitchOutputSaturation,
			pitchOutputValue);

//	/* ------------------------- Pitch Adaptive Control ------------------------- */
//	std::vector<AdaptiveElement<double>> longitudinalInput
//	{ aoaInput, pitchRateInput, pitchInput };
//
//	auto longitudinalInputTrim = controlEnvironment_.addConstant<MatrixX>(
//			pitchAdaptiveParameter.inputTrim);
//
//	auto longitudinalOutputTrim = controlEnvironment_.addConstant<MatrixX>(
//			pitchAdaptiveParameter.outputTrim);
//
//	auto longitudinalInputMux = controlEnvironment_.addMux<double, MatrixX>(longitudinalInput);
//
//	auto longitudinalInputSum = controlEnvironment_.addSum<MatrixX, MatrixX, MatrixX>(
//			longitudinalInputMux, longitudinalInputTrim, false);
//
//	auto longitudinalInputGain = controlEnvironment_.addGain<MatrixX, MatrixX, MatrixX>(
//			longitudinalInputSum, pitchAdaptiveParameter.inputGain);
//
//	pitchTargetSaturation = controlEnvironment_.addSaturation<double>(climbAnglePID,
//			degToRad(-pitchSaturation_), degToRad(pitchSaturation_),
//			degToRad(-hardPitchSaturation_), degToRad(hardPitchSaturation_));
//
//	auto pitchTargetGain = controlEnvironment_.addGain<double, MatrixX, MatrixX>(
//			pitchTargetSaturation, pitchAdaptiveParameter.targetGain);
//	/* ------------------------- Pitch Adaptive Control ------------------------- */

	/* Beta Control */
	double* betaInputValue = &beta_;

	auto betaInput = controlEnvironment_.addInput<double>(betaInputValue);
	auto betaTarget = controlEnvironment_.addConstant<double>(0);
	auto betaPID = controlEnvironment_.addPID<double>(betaInput, betaTarget, betaPIDParameter);
	auto betaPIDInverted = controlEnvironment_.addGain<double, double, double>(betaPID, -1);

	/* Yaw Output */
	double* yawOutputValue = &controllerOutput_.yawOutput;

	auto yawOutputSaturation = controlEnvironment_.addSaturation<double>(betaPIDInverted, -1, 1);

	auto yawOutput = controlEnvironment_.addOutput<double, double>(yawOutputSaturation,
			yawOutputValue);

	/* Velocity Control */
	double* velocityInputValue = &sensorData_.airSpeed;
	double* accelerationInputValue = &sensorData_.acceleration[0];
	double* velocityTargetValue = &controllerTarget_.velocity;

	auto velocityInput = controlEnvironment_.addInput<double>(velocityInputValue);
	auto accelerationInput = controlEnvironment_.addInput<double>(accelerationInputValue);
	auto velocityTarget = controlEnvironment_.addInput<double>(velocityTargetValue);

	auto velocityPID = controlEnvironment_.addPID<double>(velocityInput, velocityTarget,
			accelerationInput, velocityPIDParameter);

	auto velocityOffset = controlEnvironment_.addConstant<double>(1);
	auto velocitySum = controlEnvironment_.addSum<double, double, double>(velocityPID,
			velocityOffset, false);

	/* Throttle Output */
	double* throttleOutputValue = &controllerOutput_.throttleOutput;

	auto throttleOutputSaturation = controlEnvironment_.addSaturation<double>(velocitySum, -1, 1);

	auto throttleOutput = controlEnvironment_.addOutput<double, double>(throttleOutputSaturation,
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

	saturations_.insert(std::make_pair(ControllerConstraints::ROLL, rollTargetSaturation));
	saturations_.insert(std::make_pair(ControllerConstraints::ROLL_RATE, rollRateTargetSaturation));
	saturations_.insert(std::make_pair(ControllerConstraints::ROLL_OUTPUT, rollOutputSaturation));
	saturations_.insert(std::make_pair(ControllerConstraints::PITCH, pitchTargetSaturation));
	saturations_.insert(
			std::make_pair(ControllerConstraints::PITCH_RATE, pitchRateTargetSaturation));
	saturations_.insert(std::make_pair(ControllerConstraints::PITCH_OUTPUT, pitchOutputSaturation));
	saturations_.insert(std::make_pair(ControllerConstraints::YAW_OUTPUT, yawOutputSaturation));
	saturations_.insert(
			std::make_pair(ControllerConstraints::THROTTLE_OUTPUT, throttleOutputSaturation));
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
