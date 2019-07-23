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
 *  @file	FlightControlDataHandling.cpp
 *  @author	Mirco Theile
 *  @date	26 July 2017
 *  @brief	UAV Autopilot Flight Control Data Handling Source File
 *
 *  Description
 */

#include <memory>

#include "uavAP/Core/DataPresentation/IDataPresentation.h"
#include "uavAP/Core/PropertyMapper/PropertyMapper.h"
#include "uavAP/Core/Scheduler/IScheduler.h"
#include "uavAP/Core/Logging/APLogger.h"
#include "uavAP/Core/LockTypes.h"
#include "uavAP/Core/DataPresentation/Content.h"
#include "uavAP/FlightControl/Controller/AdaptiveController/IAdaptiveController.h"
#include "uavAP/FlightControl/Controller/AdvancedControl.h"
#include "uavAP/FlightControl/Controller/IController.h"
#include "uavAP/FlightControl/Controller/ControllerOutput.h"
#include "uavAP/FlightControl/DataHandling/FlightControlDataHandling.h"
#include "uavAP/FlightControl/Controller/PIDController/RatePIDController/RatePIDController.h"
#include "uavAP/FlightControl/Controller/PIDController/SimplePIDController/SimplePIDController.h"
#include "uavAP/FlightControl/LocalPlanner/ILocalPlanner.h"
#include "uavAP/Core/DataPresentation/BinarySerialization.hpp"

FlightControlDataHandling::FlightControlDataHandling() :
		period_(Milliseconds(100)), compressSensorData_(false)
{
}

std::shared_ptr<FlightControlDataHandling>
FlightControlDataHandling::create(const boost::property_tree::ptree& configuration)
{
	auto flightControlDataHandling = std::make_shared<FlightControlDataHandling>();

	if (!flightControlDataHandling->configure(configuration))
	{
		APLOG_ERROR << "FlightControlDataHandling: Failed to Load Config.";
	}

	return flightControlDataHandling;
}

bool
FlightControlDataHandling::configure(const boost::property_tree::ptree& configuration)
{
	PropertyMapper propertyMapper(configuration);
	propertyMapper.add("period", period_, false);
	propertyMapper.add<bool>("compress_sensor_data", compressSensorData_, false);

	return propertyMapper.map();
}

bool
FlightControlDataHandling::run(RunStage stage)
{
	switch (stage)
	{
	case RunStage::INIT:
	{
		if (!localPlanner_.isSet())
		{
			APLOG_WARN << "FlightControlDataHandling: Local Planner Missing.";
		}
		if (!controller_.isSet())
		{
			APLOG_WARN << "FlightControlDataHandling: Controller Missing.";
		}

		if (!scheduler_.isSet())
		{
			APLOG_ERROR << "FlightControlDataHandling: Scheduler Missing.";

			return true;
		}
		if (!ipc_.isSet())
		{
			APLOG_ERROR << "FlightControlDataHandling: IPC Missing.";

			return true;
		}
		if (!dataPresentation_.isSet())
		{
			APLOG_ERROR << "FlightControlDataHandling: Data Presentation Missing.";

			return true;
		}

		auto ipc = ipc_.get();
		publisher_ = ipc->publishPackets("data_fc_com");
		pidStatiPublisher_ = ipc->publishPackets("pid_stati");
		advancedControlPublisher_ = ipc->publishOnSharedMemory<AdvancedControl>("advanced_control");

		break;
	}
	case RunStage::NORMAL:
	{
		auto ipc = ipc_.get();

		flightControlSubscription_ = ipc->subscribeOnPacket("data_com_fc",
				boost::bind(&FlightControlDataHandling::receiveAndDistribute, this, _1));

		if (!flightControlSubscription_.connected())
		{
			APLOG_WARN << "FlightControlDataHandling: Flight Control Subscription Missing.";
		}

		auto scheduler = scheduler_.get();
		scheduler->schedule(std::bind(&FlightControlDataHandling::collectAndSend, this),
				Milliseconds(0), period_);

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
FlightControlDataHandling::collectAndSend()
{
	auto dp = dataPresentation_.get();
	if (!dp)
	{
		APLOG_ERROR << "FlightControlDataHandling: Data Presentation Missing.";
		return;
	}

	collectAndSendSensorData(dp);
	collectAndSendPIDStatus(dp);
	collectAndSendLocalPlannerStatus(dp);
}

void
FlightControlDataHandling::receiveAndDistribute(const Packet& packet)
{
	auto dp = dataPresentation_.get();

	if (!dp)
	{
		APLOG_ERROR << "FlightControlDataHandling: Data Presentation Missing.";
		return;
	}

	Content content = Content::INVALID;
	auto any = dp->deserialize(packet, content);

	switch (content)
	{
	case Content::TUNE_PID:
	{
		tunePID(boost::any_cast<PIDTuning>(any));
		break;
	}
	case Content::TUNE_LOCAL_PLANNER:
	{
		tuneLocalPlanner(boost::any_cast<LocalPlannerParams>(any));
		break;
	}
	case Content::REQUEST_DATA:
	{
		auto request = boost::any_cast<DataRequest>(any);
		if (request == DataRequest::MISSION)
		{
			APLOG_WARN << "FlightControlDataHandling: Received Mission Request.";
		}
		else if (request == DataRequest::TRAJECTORY)
		{
			APLOG_DEBUG << "FlightControlDataHandling: Received Trajectory Request.";
			collectAndSendTrajectory(dp);
		}

		break;
	}

	case Content::ADVANCED_CONTROL:
	{
		auto advanced = boost::any_cast<AdvancedControl>(any);

		APLOG_TRACE << "FlightControlDataHandling: Camber Control "
				<< EnumMap<CamberControl>::convert(advanced.camberSelection);
		APLOG_TRACE << "FlightControlDataHandling: Special Control "
				<< EnumMap<SpecialControl>::convert(advanced.specialSelection);
		APLOG_TRACE << "FlightControlDataHandling: Throw Control "
				<< EnumMap<ThrowsControl>::convert(advanced.throwsSelection);
		APLOG_TRACE << "FlightControlDataHandling: Camber Value " << advanced.camberValue;
		APLOG_TRACE << "FlightControlDataHandling: Special Value " << advanced.specialValue;

		advancedControlPublisher_.publish(advanced);
		break;
	}
	default:
	{
		APLOG_ERROR << "FlightControlDataHandling: Unknown Content " << static_cast<int>(content) << ".";
		break;
	}
	}
}

void
FlightControlDataHandling::notifyAggregationOnUpdate(const Aggregator& agg)
{
	localPlanner_.setFromAggregationIfNotSet(agg);
	controller_.setFromAggregationIfNotSet(agg);
	scheduler_.setFromAggregationIfNotSet(agg);
	dataPresentation_.setFromAggregationIfNotSet(agg);
	ipc_.setFromAggregationIfNotSet(agg);
	sensActIO_.setFromAggregationIfNotSet(agg);
}

void
FlightControlDataHandling::collectAndSendSensorData(
		std::shared_ptr<IDataPresentation<Content, Target>> dp)
{
	auto sens = sensActIO_.get();
	if (!sens)
	{
		APLOG_ERROR << "FlightControlDataHandling: Sensing Actuation IO Missing.";
		return;
	}

	SensorData sensData = sens->getSensorData();

	Packet sensorPacket;
	sensorPacket = dp->serialize(sensData, Content::SENSOR_DATA);

	publisher_.publish(sensorPacket);
}

void
FlightControlDataHandling::collectAndSendTrajectory(
		std::shared_ptr<IDataPresentation<Content, Target>> dp)
{
	APLOG_DEBUG << "FlightControlDataHandling: Collect and Send Trajectory.";
	auto lp = localPlanner_.get();
	if (!lp)
	{
		APLOG_ERROR << "FlightControlDataHandling: Local Planner Missing.";
		return;
	}

	Trajectory traj = lp->getTrajectory();
	Packet packet = dp->serialize(traj, Content::TRAJECTORY);
	publisher_.publish(packet);
}

void
FlightControlDataHandling::tunePID(const PIDTuning& params)
{
	auto controller = controller_.get();
	if (!controller)
	{
		APLOG_ERROR << "FlightControlDataHandling: Controller Missing.";
		return;
	}

	if (auto pidController = std::dynamic_pointer_cast<IPIDController>(controller))
	{
		pidController->getCascade()->tunePID(static_cast<PIDs>(params.pid), params.params);
	}
	else if (auto adaptiveController = std::dynamic_pointer_cast<IAdaptiveController>(controller))
	{
		adaptiveController->getCascade()->tunePID(static_cast<PIDs>(params.pid), params.params);
	}
	else
	{
		APLOG_ERROR << "FlightControlDataHandling: Unknown Controller Type.";
		return;
	}
}

void
FlightControlDataHandling::collectAndSendPIDStatus(
		std::shared_ptr<IDataPresentation<Content, Target>> dp)
{
	auto controller = controller_.get();
	if (!controller)
	{
		APLOG_ERROR << "FlightControlDataHandling: Controller Missing.";
		return;
	}

	PIDStati status;
	if (auto pidController = std::dynamic_pointer_cast<IPIDController>(controller))
	{
		status = pidController->getCascade()->getPIDStatus();
	}
	else if (auto adaptiveController = std::dynamic_pointer_cast<IAdaptiveController>(controller))
	{
		status = adaptiveController->getCascade()->getPIDStatus();
	}
	else
	{
		APLOG_ERROR << "FlightControlDataHandling: Unknown Controller Type.";
		return;
	}

	Packet packet = dp->serialize(status, Content::PID_STATUS);
	publisher_.publish(packet);
	pidStatiPublisher_.publish(dp::serialize(status));
}

void
FlightControlDataHandling::collectAndSendLocalPlannerStatus(
		std::shared_ptr<IDataPresentation<Content, Target>> dp)
{
	auto lp = localPlanner_.get();
	if (!lp)
	{
		APLOG_ERROR << "FlightControlDataHandling: Local Planner Missing.";
		return;
	}

	auto status = lp->getStatus();
	Packet packet = dp->serialize(status, Content::LOCAL_PLANNER_STATUS);
	publisher_.publish(packet);
}

void
FlightControlDataHandling::tuneLocalPlanner(const LocalPlannerParams& params)
{
	auto lp = localPlanner_.get();
	if (!lp)
	{
		APLOG_ERROR << "FlightControlDataHandling: Local Planner Missing.";
		return;
	}

	lp->tune(params);
}
