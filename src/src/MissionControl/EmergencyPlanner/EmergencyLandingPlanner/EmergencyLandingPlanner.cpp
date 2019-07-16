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
 * EmergencyLandingPlanner.cpp
 *
 *  Created on: Jul 9, 2019
 *      Author: simonyu
 */

#include "uavAP/MissionControl/EmergencyPlanner/EmergencyLandingPlanner/EmergencyLandingPlanner.h"

EmergencyLandingPlanner::EmergencyLandingPlanner()
{
}

std::shared_ptr<EmergencyLandingPlanner>
EmergencyLandingPlanner::create(const boost::property_tree::ptree& config)
{
	auto emergencyLandingPlanner = std::make_shared<EmergencyLandingPlanner>();

	if (!emergencyLandingPlanner->configure(config))
	{
		APLOG_ERROR << "EmergencyLandingPlanner: Failed to Load Config.";
	}

	return emergencyLandingPlanner;
}

bool
EmergencyLandingPlanner::configure(const boost::property_tree::ptree& config)
{
}

void
EmergencyLandingPlanner::notifyAggregationOnUpdate(const Aggregator& agg)
{
}

bool
EmergencyLandingPlanner::run(RunStage stage)
{
}

void
EmergencyLandingPlanner::onSensorData(const SensorData& data)
{
}
