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
 * EmergencyLandingPlanner.h
 *
 *  Created on: Jul 9, 2019
 *      Author: simonyu
 */

#ifndef UAVAP_MISSIONCONTROL_EMERGENCYPLANNER_EMERGENCYLANDINGPLANNER_EMERGENCYLANDINGPLANNER_H_
#define UAVAP_MISSIONCONTROL_EMERGENCYPLANNER_EMERGENCYLANDINGPLANNER_EMERGENCYLANDINGPLANNER_H_

#include "uavAP/Core/LinearAlgebra.h"
#include "uavAP/Core/IPC/Publisher.h"
#include "uavAP/Core/IPC/Subscription.h"
#include "uavAP/Core/Object/IAggregatableObject.h"
#include "uavAP/Core/Object/ObjectHandle.h"
#include "uavAP/Core/Runner/IRunnableObject.h"

class IPC;
class IScheduler;
class SensorData;

class EmergencyLandingPlanner: public IAggregatableObject, public IRunnableObject
{
public:

	static constexpr TypeId typeId = "emergency_landing_planner";

	EmergencyLandingPlanner();

	static std::shared_ptr<EmergencyLandingPlanner>
	create(const boost::property_tree::ptree& config);

	bool
	configure(const boost::property_tree::ptree& config);

	void
	notifyAggregationOnUpdate(const Aggregator& agg) override;

	bool
	run(RunStage stage) override;

private:

	void
	onSensorData(const SensorData& data);

	ObjectHandle<IPC> ipc_;
	ObjectHandle<IScheduler> scheduler_;

	Publisher overridePublisher_;
	Subscription sensorDataSubscription_;

	Vector3 landingPosition_;
	Vector3 landingAttitude_;
	Vector3 obstaclePosition_;

	double landingVelocity_;
};

#endif /* UAVAP_MISSIONCONTROL_EMERGENCYPLANNER_EMERGENCYLANDINGPLANNER_EMERGENCYLANDINGPLANNER_H_ */
