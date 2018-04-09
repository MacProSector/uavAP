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
/**
 *  @file         PIDController.cpp
 *  @author  Mirco Theile
 *  @date      05 June 2017
 *  @brief      UAV Autopilot PID Controller Source File
 *
 *  Description
 */

#include <iostream>
#include <cmath>
#include <mutex>

#include "uavAP/FlightControl/Controller/PIDController/detail/AirplanePIDCascade.h"
#include "uavAP/FlightControl/Controller/PIDController/detail/HelicopterPIDCascade.h"
#include "uavAP/FlightControl/Controller/PIDController/PIDController.h"
#include "uavAP/Core/Logging/APLogger.h"
#include "uavAP/Core/PropertyMapper/PropertyMapper.h"
#include "uavAP/Core/LockTypes.h"
#include "uavAP/Core/Scheduler/IScheduler.h"

PIDController::PIDController() :
		airplane_(true)
{
}

std::shared_ptr<PIDController>
PIDController::create(const boost::property_tree::ptree& configuration)
{
	auto flightController = std::make_shared<PIDController>();
	flightController->configure(configuration);

	return flightController;
}

bool
PIDController::configure(const boost::property_tree::ptree& config)
{
	PropertyMapper propertyMapper(config);
	propertyMapper.add("airplane", airplane_, false);

	if (airplane_)
	{
		pidCascade_ = std::make_shared<AirplanePIDCascade>(&sensorData_, velocityInertial_,
				accelerationInertial_, &controllerTarget_, &controllerOutput_);
	}
	else
	{
		pidCascade_ = std::make_shared<HelicopterPIDCascade>(&sensorData_, &controllerTarget_,
				&controllerOutput_);
	}

	return pidCascade_->configure(config);
}

bool
PIDController::run(RunStage stage)
{
	switch (stage)
	{
	case RunStage::INIT:
	{
		if (!sensAct_.isSet())
		{
			APLOG_ERROR << "PIDController: Failed to Load SensingActuationIO";

			return true;
		}
		if (!scheduler_.isSet())
		{
			APLOG_ERROR << "PIDController: Failed to Load Scheduler";

			return true;
		}

		break;
	}
	case RunStage::NORMAL:
	{
		auto scheduler = scheduler_.get();
		scheduler->schedule(std::bind(&PIDController::calculateControl, this), Milliseconds(0),
				Milliseconds(10));

		break;
	}
	case RunStage::FINAL:
	{
		break;
	}
	default:
	{
		break;
	}
	}

	return false;
}

void
PIDController::setControllerTarget(const ControllerTarget& target)
{
	LockGuard lock(controllerTargetMutex_);
	controllerTarget_ = target;
}

ControllerOutput
PIDController::getControllerOutput()
{
	return controllerOutput_;
}

std::shared_ptr<IPIDCascade>
PIDController::getCascade()
{
	return pidCascade_;
}

void
PIDController::calculateControl()
{
	auto sensAct = sensAct_.get();

	if (!sensAct)
	{
		APLOG_ERROR << "PIDController: Failed to Locate FlightControlData";

		return;
	}

	SharedLock sensorLock(sensAct->mutex);
	sensorData_ = sensAct->sensorData;
	sensorLock.unlock();

	Eigen::Matrix3d m;
	m = Eigen::AngleAxisd(sensorData_.attitude.x(), Vector3::UnitX())
			* Eigen::AngleAxisd(sensorData_.attitude.y(), Vector3::UnitY())
			* Eigen::AngleAxisd(sensorData_.attitude.z(), Vector3::UnitZ());

	velocityInertial_ = m.transpose() * sensorData_.velocity;
	accelerationInertial_ = m.transpose() * sensorData_.acceleration;
	accelerationInertial_[2] *= -1;
	Lock targetLock(controllerTargetMutex_);

	if (!pidCascade_)
	{
		APLOG_ERROR << "ControlEnv not set.";
		return;
	}
	pidCascade_->evaluate();
	targetLock.unlock();

	sensAct->setControllerOutput(controllerOutput_);
}

void
PIDController::notifyAggregationOnUpdate(Aggregator& agg)
{
	sensAct_.setFromAggregationIfNotSet(agg);
	scheduler_.setFromAggregationIfNotSet(agg);
}
