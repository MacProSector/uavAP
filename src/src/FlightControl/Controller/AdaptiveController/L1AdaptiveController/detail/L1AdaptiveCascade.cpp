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
				controllerOutput), controlEnvironment_(&sensorData.autopilotActive,
				&sensorData.timestamp), rollAngleTarget_(0), angleOfSideslipTarget_(0)
{
	deflections_.throttleOutput = 0;
}

bool
L1AdaptiveCascade::configure(const boost::property_tree::ptree& config)
{
	PropertyMapper pm(config);
	boost::property_tree::ptree adaptiveConfig;
	boost::property_tree::ptree pidConfig;
	boost::property_tree::ptree deflectionConfig;
	boost::property_tree::ptree saturationConfig;
	bool configured = true;

	pm.add("adaptive_controllers", adaptiveConfig, false);
	pm.add("pid_controllers", pidConfig, false);
	pm.add("controller_deflections", deflectionConfig, false);
	pm.add("controller_saturations", saturationConfig, false);

	configured = configured && configureAdaptive(adaptiveConfig);
	configured = configured && configurePID(pidConfig);
	configured = configured && configureDeflection(deflectionConfig);

	createCascade();

	configured = configured && configureSaturation(saturationConfig);

	return configured && pm.map();
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
	calculateControllerTarget();
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

bool
L1AdaptiveCascade::configureAdaptive(const boost::property_tree::ptree& config)
{
	PropertyMapper pm(config);

	pm.add<RollL1AdaptiveParameter>("roll", rollAdaptiveParameter_, false);
	pm.add<PitchL1AdaptiveParameter>("pitch", pitchAdaptiveParameter_, false);
	pm.add<YawL1AdaptiveParameter>("yaw", yawAdaptiveParameter_, false);

	return pm.map();
}

bool
L1AdaptiveCascade::configurePID(const boost::property_tree::ptree& config)
{
	PIDParameter pidParameter;

	for (const auto& it : config)
	{
		if (!pidParameter.configure(it.second))
		{
			APLOG_ERROR << "L1AdaptiveCascade: Invalid PID Controller Configuration " << it.first;
			return false;
		}

		pidParameters_.insert(std::make_pair(EnumMap<PIDs>::convert(it.first), pidParameter));
	}

	return true;
}

bool
L1AdaptiveCascade::configureSaturation(const boost::property_tree::ptree& config)
{
	bool configured = true;

	for (const auto& it : config)
	{
		PropertyMapper pm(it.second);
		ControllerConstraints saturationEnum = EnumMap<ControllerConstraints>::convert(it.first);
		double hardSaturation = 0;
		double softSaturation = 0;

		pm.add<double>("hard", hardSaturation, false);
		pm.add<double>("soft", softSaturation, false);

		auto saturationPair = findInMap(saturations_, saturationEnum);

		if (!saturationPair)
		{
			APLOG_WARN << "L1AdaptiveCascade: Invalid Controller Saturation Configuration "
					<< it.first;
			break;
		}

		auto saturation = saturationPair->second;

		setSaturation(saturationEnum, saturation, hardSaturation, softSaturation);

		configured = configured && pm.map();
	}

	return configured;
}

bool
L1AdaptiveCascade::configureDeflection(const boost::property_tree::ptree& config)
{
	PropertyMapper pm(config);

	for (const auto& it : config)
	{
		ControllerOutputs deflectionEnum = EnumMap<ControllerOutputs>::convert(it.first);

		switch (deflectionEnum)
		{
		case ControllerOutputs::ROLL:
		{
			pm.add<double>(it.first, deflections_.rollOutput, true);
			deflections_.rollOutput = degToRad(deflections_.rollOutput);

			break;
		}
		case ControllerOutputs::PITCH:
		{
			pm.add<double>(it.first, deflections_.pitchOutput, true);
			deflections_.pitchOutput = degToRad(deflections_.pitchOutput);

			break;
		}
		case ControllerOutputs::YAW:
		{
			pm.add<double>(it.first, deflections_.yawOutput, true);
			deflections_.yawOutput = degToRad(deflections_.yawOutput);

			break;
		}
		case ControllerOutputs::THROTTLE:
		{
			pm.add<double>(it.first, deflections_.throttleOutput, true);

			break;
		}
		case ControllerOutputs::INVALID:
		{
			APLOG_WARN << "L1AdaptiveCascade: Invalid Controller Deflection Configuration.";
			break;
		}
		default:
		{
			APLOG_WARN << "L1AdaptiveCascade: Unknown Controller Deflection Configuration.";
			break;
		}
		}
	}

	return pm.map();
}

void
L1AdaptiveCascade::setSaturation(const ControllerConstraints& saturationEnum,
		std::shared_ptr<Saturation<double>> saturation, double& hardSaturation,
		double& softSaturation)
{
	switch (saturationEnum)
	{
	case ControllerConstraints::ROLL:
	{
		hardSaturation = degToRad(hardSaturation);
		softSaturation = degToRad(softSaturation);

		break;
	}
	case ControllerConstraints::ROLL_RATE:
	{
		hardSaturation = degToRad(hardSaturation);
		softSaturation = degToRad(softSaturation);

		break;
	}
	case ControllerConstraints::PITCH:
	{
		hardSaturation = degToRad(hardSaturation);
		softSaturation = degToRad(softSaturation);

		break;
	}
	case ControllerConstraints::PITCH_RATE:
	{
		hardSaturation = degToRad(hardSaturation);
		softSaturation = degToRad(softSaturation);

		break;
	}
	default:
	{
		break;
	}
	}

	saturation->setHardSaturationValue(hardSaturation);
	saturation->setSaturationValue(softSaturation);
}

void
L1AdaptiveCascade::createCascade()
{
	PIDParameter climbAnglePIDParameter = getParameter(pidParameters_, PIDs::CLIMB_ANGLE);
	PIDParameter velocityPIDParameter = getParameter(pidParameters_, PIDs::VELOCITY);

	/* Roll control input */
	double* rollAngleInputValue = &sensorData_.attitude[0];

	auto rollAngleInput = controlEnvironment_.addInput<double>(rollAngleInputValue);

	std::vector<AdaptiveElement<double>> rollControlInputMuxVector
	{ rollAngleInput };

	auto rollControlInputMux = controlEnvironment_.addMux<double, Scalar>(
			rollControlInputMuxVector);

	auto rollControlInputTrim = controlEnvironment_.addConstant<Scalar>(
			rollAdaptiveParameter_.inputTrim);

	auto rollControlInputSum = controlEnvironment_.addSum<Scalar, Scalar, Scalar>(
			rollControlInputMux, rollControlInputTrim, false);

	/* Roll control control law */
	double* rollAngleTargetValue = &rollAngleTarget_;

	auto rollAngleTarget = controlEnvironment_.addInput<double>(rollAngleTargetValue);

	auto rollAngleTargetSaturation = controlEnvironment_.addSaturation<double>(rollAngleTarget,
			degToRad(-30.0), degToRad(30.0));

	auto rollControlTargetGain = controlEnvironment_.addGain<double, Scalar, Scalar>(
			rollAngleTargetSaturation, rollAdaptiveParameter_.targetGain);

	auto rollControlAdaptiveFeedback = controlEnvironment_.addFeedback<Scalar>();

	auto rollControlControlLawStateSpace = controlEnvironment_.addStateSpace<Vector2, Scalar,
			Matrix2, Vector2, RowVector2, Scalar, Scalar>(rollAdaptiveParameter_.controlLawState,
			rollControlAdaptiveFeedback, rollAdaptiveParameter_.controlLawMatrixA,
			rollAdaptiveParameter_.controlLawMatrixB, rollAdaptiveParameter_.controlLawMatrixC,
			rollAdaptiveParameter_.controlLawMatrixD, Scalar());

	auto rollControlControlLawSum = controlEnvironment_.addSum<Scalar, Scalar, Scalar>(
			rollControlTargetGain, rollControlControlLawStateSpace, false);

	/* Roll control output predictor */
	auto rollControlPredictorGain = controlEnvironment_.addGain<Scalar, Scalar, Scalar>(
			rollControlControlLawSum, rollAdaptiveParameter_.predictorGain);

	auto rollControlPredictorInputSum = controlEnvironment_.addSum<Scalar, Scalar, Scalar>(
			rollControlPredictorGain, rollControlAdaptiveFeedback, true);

	auto rollControlPredictorStateSpace = controlEnvironment_.addStateSpace<Scalar, Scalar, Scalar,
			Scalar, Scalar, Scalar, Scalar>(rollAdaptiveParameter_.predictorState,
			rollControlPredictorInputSum, rollAdaptiveParameter_.predictorMatrixA,
			rollAdaptiveParameter_.predictorMatrixB, rollAdaptiveParameter_.predictorMatrixC,
			rollAdaptiveParameter_.predictorMatrixD, Scalar());

	auto rollControlPredictorOutputSum = controlEnvironment_.addSum<Scalar, Scalar, Scalar>(
			rollControlPredictorStateSpace, rollControlInputSum, false);

	auto rollControlAdaptiveGain = controlEnvironment_.addGain<Scalar, Scalar, Scalar>(
			rollControlPredictorOutputSum, rollAdaptiveParameter_.adaptiveGain);

	rollControlAdaptiveFeedback->setInput(rollControlAdaptiveGain);

	/* Roll control output */
	auto rollControlOutputTrim = controlEnvironment_.addConstant<Scalar>(
			rollAdaptiveParameter_.outputTrim);

	auto rollControlOutputSum = controlEnvironment_.addSum<Scalar, Scalar, Scalar>(
			rollControlOutputTrim, rollControlControlLawSum, true);

	auto rollOutputConstant = controlEnvironment_.addConstant<double>(0);

	std::vector<std::shared_ptr<Constant<double>>> rollControlOutputVector
	{ rollOutputConstant };

	auto rollControlOutputDemux = controlEnvironment_.addDemux<Scalar, double>(rollControlOutputSum,
			rollControlOutputVector);

	double* rollOutputValue = &controllerOutput_.rollOutput;

	auto rollOutputGain = controlEnvironment_.addGain<double, double, double>(rollOutputConstant,
			1 / deflections_.rollOutput);

	auto rollOutputSaturation = controlEnvironment_.addSaturation<double>(rollOutputGain, -1, 1);

	auto rollOutput = controlEnvironment_.addOutput<double, double>(rollOutputSaturation,
			rollOutputValue);

	/* Climb angle control */
	double* angleOfAttackInputValue = &sensorData_.angleOfAttack;
	double* pitchAngleInputValue = &sensorData_.attitude[1];
	double* climbAngleTargetValue = &controllerTarget_.climbAngle;

	auto angleOfAttackInput = controlEnvironment_.addInput<double>(angleOfAttackInputValue);
	auto pitchAngleInput = controlEnvironment_.addInput<double>(pitchAngleInputValue);
	auto climbAngleTarget = controlEnvironment_.addInput<double>(climbAngleTargetValue);

	auto climbAngleSum = controlEnvironment_.addSum<double, double, double>(pitchAngleInput,
			angleOfAttackInput, false);

	auto climbAnglePID = controlEnvironment_.addPID<double>(climbAngleSum, climbAngleTarget,
			climbAnglePIDParameter);

	/* Pitch control input */
	std::vector<AdaptiveElement<double>> pitchControlInputMuxVector
	{ pitchAngleInput };

	auto pitchControlInputMux = controlEnvironment_.addMux<double, Scalar>(
			pitchControlInputMuxVector);

	auto pitchControlInputTrim = controlEnvironment_.addConstant<Scalar>(
			pitchAdaptiveParameter_.inputTrim);

	auto pitchControlInputSum = controlEnvironment_.addSum<Scalar, Scalar, Scalar>(
			pitchControlInputMux, pitchControlInputTrim, false);

	/* Pitch control control law */
	auto pitchAngleTargetSaturation = controlEnvironment_.addSaturation<double>(climbAnglePID,
			degToRad(-30.0), degToRad(30.0));

	auto pitchControlTargetGain = controlEnvironment_.addGain<double, Scalar, Scalar>(
			pitchAngleTargetSaturation, pitchAdaptiveParameter_.targetGain);

	auto pitchControlAdaptiveFeedback = controlEnvironment_.addFeedback<Scalar>();

	auto pitchControlControlLawStateSpace = controlEnvironment_.addStateSpace<Vector2, Scalar,
			Matrix2, Vector2, RowVector2, Scalar, Scalar>(pitchAdaptiveParameter_.controlLawState,
			pitchControlAdaptiveFeedback, pitchAdaptiveParameter_.controlLawMatrixA,
			pitchAdaptiveParameter_.controlLawMatrixB, pitchAdaptiveParameter_.controlLawMatrixC,
			pitchAdaptiveParameter_.controlLawMatrixD, Scalar());

	auto pitchControlControlLawSum = controlEnvironment_.addSum<Scalar, Scalar, Scalar>(
			pitchControlTargetGain, pitchControlControlLawStateSpace, false);

	/* Pitch control output predictor */
	auto pitchControlPredictorGain = controlEnvironment_.addGain<Scalar, Scalar, Scalar>(
			pitchControlControlLawSum, pitchAdaptiveParameter_.predictorGain);

	auto pitchControlPredictorInputSum = controlEnvironment_.addSum<Scalar, Scalar, Scalar>(
			pitchControlPredictorGain, pitchControlAdaptiveFeedback, true);

	auto pitchControlPredictorStateSpace = controlEnvironment_.addStateSpace<Scalar, Scalar, Scalar,
			Scalar, Scalar, Scalar, Scalar>(pitchAdaptiveParameter_.predictorState,
			pitchControlPredictorInputSum, pitchAdaptiveParameter_.predictorMatrixA,
			pitchAdaptiveParameter_.predictorMatrixB, pitchAdaptiveParameter_.predictorMatrixC,
			pitchAdaptiveParameter_.predictorMatrixD, Scalar());

	auto pitchControlPredictorOutputSum = controlEnvironment_.addSum<Scalar, Scalar, Scalar>(
			pitchControlPredictorStateSpace, pitchControlInputSum, false);

	auto pitchControlAdaptiveGain = controlEnvironment_.addGain<Scalar, Scalar, Scalar>(
			pitchControlPredictorOutputSum, pitchAdaptiveParameter_.adaptiveGain);

	pitchControlAdaptiveFeedback->setInput(pitchControlAdaptiveGain);

	/* Pitch control output */
	auto pitchControlOutputTrim = controlEnvironment_.addConstant<Scalar>(
			pitchAdaptiveParameter_.outputTrim);

	auto pitchControlOutputSum = controlEnvironment_.addSum<Scalar, Scalar, Scalar>(
			pitchControlOutputTrim, pitchControlControlLawSum, true);

	auto pitchOutputConstant = controlEnvironment_.addConstant<double>(0);

	std::vector<std::shared_ptr<Constant<double>>> pitchControlOutputVector
	{ pitchOutputConstant };

	auto pitchControlOutputDemux = controlEnvironment_.addDemux<Scalar, double>(
			pitchControlOutputSum, pitchControlOutputVector);

	double* pitchOutputValue = &controllerOutput_.pitchOutput;

	auto pitchOutputGain = controlEnvironment_.addGain<double, double, double>(pitchOutputConstant,
			1 / deflections_.pitchOutput);

	auto pitchOutputSaturation = controlEnvironment_.addSaturation<double>(pitchOutputGain, -1, 1);

	auto pitchOutput = controlEnvironment_.addOutput<double, double>(pitchOutputSaturation,
			pitchOutputValue);

	/* Yaw control input */
	double* angleOfSideslipInputValue = &sensorData_.angleOfSideslip;

	auto angleOfSideslipInput = controlEnvironment_.addInput<double>(angleOfSideslipInputValue);

	std::vector<AdaptiveElement<double>> yawControlInputMuxVector
	{ angleOfSideslipInput };

	auto yawControlInputMux = controlEnvironment_.addMux<double, Scalar>(yawControlInputMuxVector);

	auto yawControlInputTrim = controlEnvironment_.addConstant<Scalar>(
			yawAdaptiveParameter_.inputTrim);

	auto yawControlInputSum = controlEnvironment_.addSum<Scalar, Scalar, Scalar>(yawControlInputMux,
			yawControlInputTrim, false);

	/* Yaw control control law */
	double* angleOfSideslipTargetValue = &angleOfSideslipTarget_;

	auto angleOfSideslipTarget = controlEnvironment_.addInput<double>(angleOfSideslipTargetValue);

	auto angleOfSideslipTargetSaturation = controlEnvironment_.addSaturation<double>(
			angleOfSideslipTarget, degToRad(-30.0), degToRad(30.0));

	auto yawControlTargetGain = controlEnvironment_.addGain<double, Scalar, Scalar>(
			angleOfSideslipTargetSaturation, yawAdaptiveParameter_.targetGain);

	auto yawControlAdaptiveFeedback = controlEnvironment_.addFeedback<Scalar>();

	auto yawControlControlLawStateSpace = controlEnvironment_.addStateSpace<Vector2, Scalar,
			Matrix2, Vector2, RowVector2, Scalar, Scalar>(yawAdaptiveParameter_.controlLawState,
			yawControlAdaptiveFeedback, yawAdaptiveParameter_.controlLawMatrixA,
			yawAdaptiveParameter_.controlLawMatrixB, yawAdaptiveParameter_.controlLawMatrixC,
			yawAdaptiveParameter_.controlLawMatrixD, Scalar());

	auto yawControlControlLawSum = controlEnvironment_.addSum<Scalar, Scalar, Scalar>(
			yawControlTargetGain, yawControlControlLawStateSpace, false);

	/* Yaw control output predictor */
	auto yawControlPredictorGain = controlEnvironment_.addGain<Scalar, Scalar, Scalar>(
			yawControlControlLawSum, yawAdaptiveParameter_.predictorGain);

	auto yawControlPredictorInputSum = controlEnvironment_.addSum<Scalar, Scalar, Scalar>(
			yawControlPredictorGain, yawControlAdaptiveFeedback, true);

	auto yawControlPredictorStateSpace = controlEnvironment_.addStateSpace<Scalar, Scalar, Scalar,
			Scalar, Scalar, Scalar, Scalar>(yawAdaptiveParameter_.predictorState,
			yawControlPredictorInputSum, yawAdaptiveParameter_.predictorMatrixA,
			yawAdaptiveParameter_.predictorMatrixB, yawAdaptiveParameter_.predictorMatrixC,
			yawAdaptiveParameter_.predictorMatrixD, Scalar());

	auto yawControlPredictorOutputSum = controlEnvironment_.addSum<Scalar, Scalar, Scalar>(
			yawControlPredictorStateSpace, yawControlInputSum, false);

	auto yawControlAdaptiveGain = controlEnvironment_.addGain<Scalar, Scalar, Scalar>(
			yawControlPredictorOutputSum, yawAdaptiveParameter_.adaptiveGain);

	yawControlAdaptiveFeedback->setInput(yawControlAdaptiveGain);

	/* Yaw control output */
	auto yawControlOutputTrim = controlEnvironment_.addConstant<Scalar>(
			yawAdaptiveParameter_.outputTrim);

	auto yawControlOutputSum = controlEnvironment_.addSum<Scalar, Scalar, Scalar>(
			yawControlOutputTrim, yawControlControlLawSum, true);

	auto yawOutputConstant = controlEnvironment_.addConstant<double>(0);

	std::vector<std::shared_ptr<Constant<double>>> yawControlOutputVector
	{ yawOutputConstant };

	auto yawControlOutputDemux = controlEnvironment_.addDemux<Scalar, double>(yawControlOutputSum,
			yawControlOutputVector);

	double* yawOutputValue = &controllerOutput_.yawOutput;

	auto yawOutputGain = controlEnvironment_.addGain<double, double, double>(yawOutputConstant,
			1 / deflections_.yawOutput);

	auto yawOutputSaturation = controlEnvironment_.addSaturation<double>(yawOutputGain, -1, 1);

	auto yawOutput = controlEnvironment_.addOutput<double, double>(yawOutputSaturation,
			yawOutputValue);

	/* Velocity control */
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

	/* Throttle output */
	double* throttleOutputValue = &controllerOutput_.throttleOutput;

	auto throttleOutputGain = controlEnvironment_.addGain<double, double, double>(velocitySum,
			1 / deflections_.throttleOutput);

	auto throttleOutputSaturation = controlEnvironment_.addSaturation<double>(throttleOutputGain,
			-1, 1);

	auto throttleOutput = controlEnvironment_.addOutput<double, double>(throttleOutputSaturation,
			throttleOutputValue);

	pids_.insert(std::make_pair(PIDs::CLIMB_ANGLE, climbAnglePID));
	pids_.insert(std::make_pair(PIDs::VELOCITY, velocityPID));

	outputs_.insert(std::make_pair(ControllerOutputs::ROLL, rollOutput));
	outputs_.insert(std::make_pair(ControllerOutputs::PITCH, pitchOutput));
	outputs_.insert(std::make_pair(ControllerOutputs::YAW, yawOutput));
	outputs_.insert(std::make_pair(ControllerOutputs::THROTTLE, throttleOutput));

	outputWaveforms_.insert(std::make_pair(ControllerOutputsWaveforms::ROLL, rollOutput));
	outputWaveforms_.insert(std::make_pair(ControllerOutputsWaveforms::PITCH, pitchOutput));
	outputWaveforms_.insert(std::make_pair(ControllerOutputsWaveforms::YAW, yawOutput));
	outputWaveforms_.insert(std::make_pair(ControllerOutputsWaveforms::THROTTLE, throttleOutput));

	saturations_.insert(std::make_pair(ControllerConstraints::ROLL, rollAngleTargetSaturation));
	saturations_.insert(std::make_pair(ControllerConstraints::PITCH, pitchAngleTargetSaturation));
	saturations_.insert(
			std::make_pair(ControllerConstraints::YAW, angleOfSideslipTargetSaturation));
	saturations_.insert(std::make_pair(ControllerConstraints::ROLL_OUTPUT, rollOutputSaturation));
	saturations_.insert(std::make_pair(ControllerConstraints::PITCH_OUTPUT, pitchOutputSaturation));
	saturations_.insert(std::make_pair(ControllerConstraints::YAW_OUTPUT, yawOutputSaturation));
	saturations_.insert(
			std::make_pair(ControllerConstraints::THROTTLE_OUTPUT, throttleOutputSaturation));
}

void
L1AdaptiveCascade::calculateControllerTarget()
{
	Eigen::Matrix3d rotationMatrix;
	Vector3 velocityBody;
	double velocityBodyTotal;
	double roll = sensorData_.attitude.x();
	double pitch = -sensorData_.attitude.y();
	double yaw = sensorData_.attitude.z();
	double gravity = 9.81;

	rotationMatrix = Eigen::AngleAxisd(-roll, Vector3::UnitX())
			* Eigen::AngleAxisd(-pitch, Vector3::UnitY())
			* Eigen::AngleAxisd(-yaw, Vector3::UnitZ());

	velocityBody = rotationMatrix * sensorData_.velocity;
	velocityBodyTotal = velocityBody.norm();

	rollAngleTarget_ = -atan2(velocityBodyTotal * controllerTarget_.yawRate, gravity);
	angleOfSideslipTarget_ = 0;
}
