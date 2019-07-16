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
 * DirectRollGeofencingModel.cpp
 *
 *  Created on: Aug 21, 2018
 *      Author: simonyu
 */

#include "uavAP/Core/PropertyMapper/PropertyMapper.h"
#include "uavAP/MissionControl/EmergencyPlanner/EmergencyGeofencingPlanner/GeofencingModel/DirectRollGeofencingModel/DirectRollGeofencingModel.h"

DirectRollModel::DirectRollModel() :
		rollMax_(0), g_(9.81), radiusOrbit_(0)
{
}

std::shared_ptr<DirectRollModel>
DirectRollModel::create(const boost::property_tree::ptree& config)
{
	auto directRollModel = std::make_shared<DirectRollModel>();

	if (!directRollModel->configure(config))
	{
		APLOG_ERROR << "DirectRollModel: Failed to Load Config.";
	}

	return directRollModel;
}

bool
DirectRollModel::configure(const boost::property_tree::ptree& config)
{
	PropertyMapper pm(config);

	pm.add<double>("roll_max", rollMax_, true);
	pm.add<double>("g", g_, false);

	degToRadRef(rollMax_);

	return pm.map();
}

void
DirectRollModel::notifyAggregationOnUpdate(const Aggregator& agg)
{
}

bool
DirectRollModel::updateModel(const SensorData& data)
{
	std::unique_lock<std::mutex> lock(queryMutex_, std::try_to_lock);

	if (!lock.owns_lock())
	{
		APLOG_DEBUG << "Data in use. Will update next time.";
		return false;
	}

	VehicleOneFrame frame(data.attitude[2]);

	double velocity = data.airSpeed;
	double curvature = tan(rollMax_) * g_ / velocity;

	radiusOrbit_ = velocity / curvature;
	centerOrbitLeft_ = frame.toInertialFramePosition(Vector3(-radiusOrbit_, 0, 0));
	centerOrbitRight_ = frame.toInertialFramePosition(Vector3(radiusOrbit_, 0, 0));

	return true;
}

std::vector<Vector3>
DirectRollModel::getCriticalPoints(const Edge& edge, RollDirection dir)
{
	std::unique_lock<std::mutex> lock(queryMutex_);

	std::vector<Vector3> result;
	Vector3 normal(-edge.normal.x(), -edge.normal.y(), 0);

	if (dir == RollDirection::LEFT)
	{
		result.push_back(centerOrbitLeft_ + normal * radiusOrbit_);
	}
	else
	{
		result.push_back(centerOrbitRight_ + normal * radiusOrbit_);
	}

	return result;
}
