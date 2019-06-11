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
 * L1AdaptiveController.cpp
 *
 *  Created on: Jun 5, 2019
 *      Author: simonyu
 */

#include "uavAP/Core/LockTypes.h"
#include "uavAP/Core/IPC/IPC.h"
#include "uavAP/Core/Scheduler/IScheduler.h"
#include "uavAP/Core/DataPresentation/Packet.h"
#include "uavAP/MissionControl/ManeuverPlanner/Override.h"
#include "uavAP/FlightControl/SensingActuationIO/ISensingActuationIO.h"
#include "uavAP/FlightControl/Controller/ControllerOutput.h"
#include "uavAP/FlightControl/Controller/ControllerTarget.h"
#include "uavAP/FlightControl/Controller/AdaptiveController/IAdaptiveCascade.h"
#include "uavAP/FlightControl/Controller/AdaptiveController/L1AdaptiveController/L1AdaptiveController.h"
#include "uavAP/FlightControl/Controller/AdaptiveController/L1AdaptiveController/detail/L1AdaptiveCascade.h"
#include "uavAP/Core/DataPresentation/BinarySerialization.hpp"

L1AdaptiveController::L1AdaptiveController()
{
}

std::shared_ptr<L1AdaptiveController>
L1AdaptiveController::create(const boost::property_tree::ptree& config)
{
	auto l1AdaptiveController = std::make_shared<L1AdaptiveController>();
	l1AdaptiveController->configure(config);

	return l1AdaptiveController;
}

bool
L1AdaptiveController::configure(const boost::property_tree::ptree& config)
{
	l1AdaptiveCascade_ = std::make_shared<L1AdaptiveCascade>(sensorData_, controllerTarget_,
			controllerOutput_);

	return l1AdaptiveCascade_->configure(config);
}

bool
L1AdaptiveController::run(RunStage stage)
{
	switch (stage)
	{
	case RunStage::INIT:
	{
		if (!ipc_.isSet())
		{
			APLOG_ERROR << "L1AdaptiveController: IPC Missing";
			return true;
		}

		if (!scheduler_.isSet())
		{
			APLOG_ERROR << "L1AdaptiveController: Scheduler Missing";
			return true;
		}

		if (!sensingActuationIO_.isSet())
		{
			APLOG_ERROR << "L1AdaptiveController: SensingActuationIO Missing";
			return true;
		}

		auto ipc = ipc_.get();

		controllerOutputPublisherMP_ = ipc->publishPackets("controller_output_mp");
		controllerOutputPublisherTA_ = ipc->publishPackets("controller_output_ta");
		controllerOutputPublisherMA_ = ipc->publishPackets("controller_output_ma");

		break;
	}
	case RunStage::NORMAL:
	{
		auto ipc = ipc_.get();

		overrideSubscription_ = ipc->subscribeOnPacket("override",
				std::bind(&L1AdaptiveController::onOverridePacket, this,
						std::placeholders::_1));

		if (!overrideSubscription_.connected())
		{
			APLOG_ERROR << "L1AdaptiveController: Override Subscription Missing";
			return true;
		}

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
L1AdaptiveController::notifyAggregationOnUpdate(const Aggregator& agg)
{
	ipc_.setFromAggregationIfNotSet(agg);
	scheduler_.setFromAggregationIfNotSet(agg);
	sensingActuationIO_.setFromAggregationIfNotSet(agg);
}

ControllerOutput
L1AdaptiveController::getControllerOutput() const
{
	return controllerOutput_;
}

std::shared_ptr<IAdaptiveCascade>
L1AdaptiveController::getCascade() const
{
	return l1AdaptiveCascade_;
}

void
L1AdaptiveController::setControllerTarget(const ControllerTarget& target)
{
	controllerTarget_ = target;
	calculateControl();
}

void
L1AdaptiveController::calculateControl()
{
	auto sensingActuationIO = sensingActuationIO_.get();

	if (!sensingActuationIO)
	{
		APLOG_ERROR << "L1AdaptiveController: SensingActuationIO Missing";
		return;
	}

	sensorData_ = sensingActuationIO->getSensorData();

	Lock targetLock(controllerTargetMutex_);
	if (!l1AdaptiveCascade_)
	{
		APLOG_ERROR << "L1AdaptiveController: Cascade Missing";
		return;
	}

	l1AdaptiveCascade_->evaluate();

	controllerOutput_.sequenceNr = std::min(sensorData_.sequenceNr, controllerTarget_.sequenceNr);
	targetLock.unlock();

	sensingActuationIO->setControllerOutput(controllerOutput_);
	controllerOutputPublisherMP_.publish(dp::serialize(controllerOutput_));
	controllerOutputPublisherTA_.publish(dp::serialize(controllerOutput_));
	controllerOutputPublisherMA_.publish(dp::serialize(controllerOutput_));

	APLOG_TRACE << "Roll Output: " << controllerOutput_.rollOutput;
	APLOG_TRACE << "Pitch Output: " << controllerOutput_.pitchOutput;
	APLOG_TRACE << "Yaw Output: " << controllerOutput_.yawOutput;
	APLOG_TRACE << "Throttle Output: " << controllerOutput_.throttleOutput;
	APLOG_TRACE << " ";
}

void
L1AdaptiveController::onOverridePacket(const Packet& packet)
{
	auto override = dp::deserialize<Override>(packet);

	LockGuard lock(controllerTargetMutex_);
	l1AdaptiveCascade_->overrideCascade(override);
}
