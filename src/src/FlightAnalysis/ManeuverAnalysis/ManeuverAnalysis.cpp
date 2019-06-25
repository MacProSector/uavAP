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
 * ManeuverAnalysis.cpp
 *
 *  Created on: Aug 6, 2018
 *      Author: simonyu
 */

#include <string>

#include "uavAP/Core/PropertyMapper/PropertyMapper.h"
#include "uavAP/FlightAnalysis/ManeuverAnalysis/ManeuverAnalysis.h"
#include "uavAP/Core/DataPresentation/BinarySerialization.hpp"

ManeuverAnalysis::ManeuverAnalysis() :
		collectInit_(false), counter_(0), maneuver_(Maneuvers::GEOFENCING), loggingPeriod_(0)
{
}

std::shared_ptr<ManeuverAnalysis>
ManeuverAnalysis::create(const boost::property_tree::ptree& config)
{
	auto maneuverAnalysis = std::make_shared<ManeuverAnalysis>();

	if (!maneuverAnalysis->configure(config))
	{
		APLOG_ERROR << "ManeuverAnalysis: Failed to Load Config.";
	}

	return maneuverAnalysis;
}

bool
ManeuverAnalysis::configure(const boost::property_tree::ptree& config)
{
	PropertyMapper pm(config);
	std::string maneuver;

	pm.add("log_path", logPath_, true);
	pm.add<unsigned int>("logging_period", loggingPeriod_, false);

	if (pm.add("maneuver", maneuver, false))
	{
		maneuver_ = EnumMap<Maneuvers>::convert(maneuver);
	}

	collectInit_ = false;
	counter_ = 0;

	return pm.map();
}

bool
ManeuverAnalysis::run(RunStage stage)
{
	switch (stage)
	{
	case RunStage::INIT:
	{
		if (!ipcHandle_.isSet())
		{
			APLOG_ERROR << "IPC Missing.";
			return true;
		}
		if (!scheduler_.isSet())
		{
			APLOG_ERROR << "Scheduler Missing.";
			return true;
		}

		break;
	}
	case RunStage::NORMAL:
	{
		auto ipc = ipcHandle_.get();

		sensorDataSubscription_ = ipc->subscribeOnSharedMemory<SensorData>("sensor_data",
				std::bind(&ManeuverAnalysis::onSensorData, this, std::placeholders::_1));

		if (!sensorDataSubscription_.connected())
		{
			APLOG_ERROR << "Sensor Data Subscription Missing.";
			return true;
		}

		maneuverAnalysisSubscription_ = ipc->subscribeOnPacket("maneuver_analysis_status",
				std::bind(&ManeuverAnalysis::onManeuverAnalysisStatus, this,
						std::placeholders::_1));

		if (!maneuverAnalysisSubscription_.connected())
		{
			APLOG_ERROR << "Maneuver Analysis Subscription Missing.";
			return true;
		}

		if (loggingPeriod_ > 0)
		{
			auto scheduler = scheduler_.get();

			scheduler->schedule(std::bind(&ManeuverAnalysis::logSensorData, this),
					Milliseconds(0), Milliseconds(loggingPeriod_));
		}

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
ManeuverAnalysis::notifyAggregationOnUpdate(const Aggregator& agg)
{
	ipcHandle_.setFromAggregationIfNotSet(agg);
	scheduler_.setFromAggregationIfNotSet(agg);
}

void
ManeuverAnalysis::onSensorData(const SensorData& data)
{
	std::unique_lock<std::mutex> lock(sensorDataMutex_);
	sensorData_ = data;
	lock.unlock();

	if (loggingPeriod_ == 0)
		logSensorData();
}

void
ManeuverAnalysis::logSensorData()
{
	std::unique_lock<std::mutex> lock(maneuverAnalysisStatusMutex_);
	ManeuverAnalysisStatus analysis = analysis_;
	lock.unlock();

	std::unique_lock<std::mutex> lockSensorData(sensorDataMutex_);
	if (analysis.analysis && !collectInit_)
	{
		collectStateInit(sensorData_, analysis.maneuver, analysis_.interrupted);
	}
	else if (analysis.analysis && collectInit_)
	{
		collectStateNormal(sensorData_);
	}
	else if (!analysis.analysis && collectInit_)
	{
		collectStateFinal(sensorData_);
	}
}

void
ManeuverAnalysis::onManeuverAnalysisStatus(const Packet& status)
{
	std::unique_lock<std::mutex> lock(maneuverAnalysisStatusMutex_);
	analysis_ = dp::deserialize<ManeuverAnalysisStatus>(status);
	lock.unlock();
}

void
ManeuverAnalysis::collectStateInit(const SensorData& data, const std::string& maneuver,
		const bool& interrupted)
{
	APLOG_DEBUG << "ManeuverAnalysis::collectStateInit.";

	if (!interrupted)
	{
		counter_++;
	}

	std::string maneuverName = std::to_string(counter_) + "_" + maneuver;
	std::string logFileName = logPath_ + maneuverName + ".log";
	auto t = std::chrono::system_clock::to_time_t(data.timestamp);
	std::string time = std::ctime(&t);

	logFile_.open(logFileName);
	logFile_.precision(15);

	if (!logFile_.is_open())
	{
		APLOG_ERROR << "ManeuverAnalysis: Failed to Open Log File. Cannot Log.";
		return;
	}

	logFile_ << maneuverName << std::endl;
	logFile_ << time << std::endl;
	logFile_ << "collectStateInit:" << std::endl;

	switch (maneuver_)
	{
	case Maneuvers::GEOFENCING:
	{
		collectGeofencing(data, CollectStates::INIT);
		break;
	}
	case Maneuvers::ADVANCED_CONTROL:
	{
		collectAdvancedControl(data, CollectStates::INIT);
		break;
	}
	case Maneuvers::SEQUENCE:
	{
		collectSequence(data, CollectStates::INIT);
		break;
	}
	default:
	{
		break;
	}
	}

	logFile_ << "collectStateNormal:" << std::endl;

	collectInit_ = true;
}

void
ManeuverAnalysis::collectStateNormal(const SensorData& data)
{
	APLOG_DEBUG << "ManeuverAnalysis::collectStateNormal.";

	if (!logFile_.is_open())
	{
		APLOG_ERROR << "ManeuverAnalysis: Failed to Open Log File. Cannot Log.";
		return;
	}

	switch (maneuver_)
	{
	case Maneuvers::GEOFENCING:
	{
		collectGeofencing(data, CollectStates::NORMAL);
		break;
	}
	case Maneuvers::ADVANCED_CONTROL:
	{
		collectAdvancedControl(data, CollectStates::NORMAL);
		break;
	}
	case Maneuvers::SEQUENCE:
	{
		collectSequence(data, CollectStates::NORMAL);
		break;
	}
	default:
	{
		break;
	}
	}
}

void
ManeuverAnalysis::collectStateFinal(const SensorData& data)
{
	APLOG_DEBUG << "ManeuverAnalysis::collectStateFinal.";

	if (!logFile_.is_open())
	{
		APLOG_ERROR << "ManeuverAnalysis: Failed to Open Log File. Cannot Log.";
		return;
	}

	logFile_ << "collectStateFinal:" << std::endl;

	switch (maneuver_)
	{
	case Maneuvers::GEOFENCING:
	{
		collectGeofencing(data, CollectStates::FINAL);
		break;
	}
	case Maneuvers::ADVANCED_CONTROL:
	{
		collectAdvancedControl(data, CollectStates::FINAL);
		break;
	}
	case Maneuvers::SEQUENCE:
	{
		collectSequence(data, CollectStates::FINAL);
		break;
	}
	default:
	{
		break;
	}
	}

	logFile_.close();

	collectInit_ = false;
}

void
ManeuverAnalysis::collectGeofencing(const SensorData& data, const CollectStates& states)
{
	switch (states)
	{
	case CollectStates::INIT:
	{
		logFile_ << "Position E (m)" << "	" << "Position N (m)" << "	" << "Roll Angle (Rad)" << "	"
				<< "Yaw Angle (Rad)" << "	" << "Ground Speed (m/s)" << "	" << "Roll Rate (Rad/s)"
				<< std::endl;

		logFile_ << data.position.x() << "	" << data.position.y() << "	" << data.attitude.x() << "	"
				<< data.attitude.z() << "	" << data.groundSpeed << "	" << data.angularRate.x()
				<< std::endl;

		break;
	}
	case CollectStates::NORMAL:
	{
		logFile_ << data.position.x() << "	" << data.position.y() << "	" << data.attitude.x() << "	"
				<< data.attitude.z() << "	" << data.groundSpeed << "	" << data.angularRate.x()
				<< std::endl;

		break;
	}
	case CollectStates::FINAL:
	{
		break;
	}
	default:
	{
		break;
	}
	}
}

void
ManeuverAnalysis::collectAdvancedControl(const SensorData& data, const CollectStates& states)
{
	switch (states)
	{
	case CollectStates::INIT:
	{
		logFile_ << "Velocity E (m/s)" << "	" << "Velocity N (m/s)" << "	" << "Velocity U (m/s)"
				<< "	" << "Air Speed (m/s)" << "	" << "Roll Angle (Rad)" << "	"
				<< "Pitch Angle (Rad)" << "	" << "Yaw Angle (Rad)" << "	" << "Battery Voltage (V)"
				<< "	" << "Battery Current (A)" << "	" << "Throttle Level (%)" << "	"
				<< "Motor Speed (RPM)" << std::endl;

		logFile_ << data.velocity.x() << "	" << data.velocity.y() << "	" << data.velocity.z() << "	"
				<< data.airSpeed << "	" << data.attitude.x() << "	" << data.attitude.y() << "	"
				<< data.attitude.z() << "	" << data.batteryVoltage << "	" << data.batteryCurrent
				<< "	" << data.throttle << "	" << data.rpm << std::endl;

		break;
	}
	case CollectStates::NORMAL:
	{
		logFile_ << data.velocity.x() << "	" << data.velocity.y() << "	" << data.velocity.z() << "	"
				<< data.airSpeed << "	" << data.attitude.x() << "	" << data.attitude.y() << "	"
				<< data.attitude.z() << "	" << data.batteryVoltage << "	" << data.batteryCurrent
				<< "	" << data.throttle << "	" << data.rpm << std::endl;

		break;
	}
	case CollectStates::FINAL:
	{
		break;
	}
	default:
	{
		break;
	}
	}
}

void
ManeuverAnalysis::collectSequence(const SensorData& data, const CollectStates& states)
{
	switch (states)
	{
	case CollectStates::INIT:
	{
		logFile_ << "Position E (m)" << "	" << "Position N (m)" << "	" << "Roll Angle (Rad)" << "	"
				<< "Yaw Angle (Rad)" << "	" << "Ground Speed (m/s)" << "	" << "Roll Rate (Rad/s)"
				<< "	" << "Time Stamp (YYYY-mmm-DD HH:MM:SS.fffffffff)" << std::endl;

		logFile_ << data.position.x() << "	" << data.position.y() << "	" << data.attitude.x() << "	"
				<< data.attitude.z() << "	" << data.groundSpeed << "	" << data.angularRate.x()
				<< "	" << data.timestamp.time_since_epoch().count() << std::endl;

		break;
	}
	case CollectStates::NORMAL:
	{
		logFile_ << data.position.x() << "	" << data.position.y() << "	" << data.attitude.x() << "	"
				<< data.attitude.z() << "	" << data.groundSpeed << "	" << data.angularRate.x()
				<< "	" << data.timestamp.time_since_epoch().count() << std::endl;

		break;
	}
	case CollectStates::FINAL:
	{
		logFile_ << data.position.x() << "	" << data.position.y() << "	" << data.attitude.x() << "	"
				<< data.attitude.z() << "	" << data.groundSpeed << "	" << data.angularRate.x()
				<< "	" << data.timestamp.time_since_epoch().count() << std::endl;

		break;
	}
	default:
	{
		break;
	}
	}
}
