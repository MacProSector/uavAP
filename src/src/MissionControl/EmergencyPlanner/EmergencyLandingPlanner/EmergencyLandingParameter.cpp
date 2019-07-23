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
 * EmergencyLandingParameter.cpp
 *
 *  Created on: Jul 17, 2019
 *      Author: simonyu
 */

#include "uavAP/Core/PropertyMapper/PropertyMapper.h"
#include "uavAP/MissionControl/EmergencyPlanner/EmergencyLandingPlanner/EmergencyLandingParameter.h"

bool
PlanningParameter::configure(const boost::property_tree::ptree& config)
{
	PropertyMapper pm(config);

	pm.add<double>("acceleration", acceleration, isDefault);
	pm.add<double>("climb_rate", climbRate, isDefault);
	pm.add<double>("yaw_rate", yawRate, isDefault);
	pm.add<double>("gliding_velocity", glidingVelocity, isDefault);
	pm.add<double>("gliding_climb_angle", glidingClimbAngle, isDefault);
	pm.add<double>("period", period, isDefault);

	periodSecond = period / 1000;

	return pm.map();
}

bool
SearchingParameter::configure(const boost::property_tree::ptree& config)
{
	PropertyMapper pm(config);

	pm.add<double>("acceleration", acceleration, isDefault);
	pm.add<double>("climb_rate", climbRate, isDefault);
	pm.add<double>("yaw_rate", yawRate, isDefault);
	pm.add<double>("cruise_radius", cruiseRadius, isDefault);
	pm.add<double>("loiter_radius", loiterRadius, isDefault);
	pm.add<double>("loiter_lambda", loiterLambda, isDefault);
	pm.add<double>("approach_altitude", approachAltitude, isDefault);
	pm.add<double>("line_distance", lineDistance, isDefault);
	pm.add<double>("line_delta", lineDelta, isDefault);

	return pm.map();
}

bool
ApproachingParameter::configure(const boost::property_tree::ptree& config)
{
	PropertyMapper pm(config);
	boost::property_tree::ptree obstacleConfig;

	pm.add("position", position, isDefault);
	pm.add<double>("velocity", velocity, isDefault);
	pm.add<double>("climb_angle", climbAngle, isDefault);
	pm.add<double>("yaw_angle", yawAngle, isDefault);
	pm.add<double>("sideslip_angle", angleOfSideslip, isDefault);
	pm.add("obstacles", obstacleConfig, false);

	PropertyMapper obstaclePm(obstacleConfig);
	Obstacles obstacles;

	for (const auto& it : obstacleConfig)
	{
		std::string obstacleName = it.first;
		Vector3 obstacleSpecification;

		if (obstaclePm.add(it.first, obstacleSpecification, false))
		{
			Obstacle obstacle = std::make_pair(obstacleName, obstacleSpecification);
			obstacles.push_back(obstacle);
		}
	}

	this->obstacles = obstacles;

	return pm.map() && obstaclePm.map();
}

bool
EmergencyLandingParameter::configure(const boost::property_tree::ptree& config)
{
	PropertyMapper pm(config);

	planningParameter.isDefault = isDefault;
	searchingParameter.isDefault = isDefault;
	approachingParameter.isDefault = isDefault;

	radianToDegree(planningParameter);
	radianToDegree(searchingParameter);
	radianToDegree(approachingParameter);

	pm.addStruct<PlanningParameter>("planning", planningParameter, isDefault);
	pm.addStruct<SearchingParameter>("searching", searchingParameter, isDefault);
	pm.addStruct<ApproachingParameter>("approaching", approachingParameter, isDefault);

	degreeToRadian(planningParameter);
	degreeToRadian(searchingParameter);
	degreeToRadian(approachingParameter);

	phase = EmergencyLandingPhases::CRUISING;

	return pm.map();
}

void
degreeToRadian(PlanningParameter& planningParameter)
{
	planningParameter.climbRate = degToRad(planningParameter.climbRate);
	planningParameter.yawRate = degToRad(planningParameter.yawRate);
	planningParameter.glidingClimbAngle = degToRad(planningParameter.glidingClimbAngle);
}

void
degreeToRadian(SearchingParameter& searchingParameter)
{
	searchingParameter.climbRate = degToRad(searchingParameter.climbRate);
	searchingParameter.yawRate = degToRad(searchingParameter.yawRate);
	searchingParameter.loiterLambda = degToRad(searchingParameter.loiterLambda);
}

void
degreeToRadian(ApproachingParameter& approachingParameter)
{
	approachingParameter.climbAngle = degToRad(approachingParameter.climbAngle);
	approachingParameter.yawAngle = degToRad(approachingParameter.yawAngle);
	approachingParameter.angleOfSideslip = degToRad(approachingParameter.angleOfSideslip);
}

void
radianToDegree(PlanningParameter& planningParameter)
{
	planningParameter.climbRate = radToDeg(planningParameter.climbRate);
	planningParameter.yawRate = radToDeg(planningParameter.yawRate);
	planningParameter.glidingClimbAngle = radToDeg(planningParameter.glidingClimbAngle);
}

void
radianToDegree(SearchingParameter& searchingParameter)
{
	searchingParameter.climbRate = radToDeg(searchingParameter.climbRate);
	searchingParameter.yawRate = radToDeg(searchingParameter.yawRate);
	searchingParameter.loiterLambda = radToDeg(searchingParameter.loiterLambda);
}

void
radianToDegree(ApproachingParameter& approachingParameter)
{
	approachingParameter.climbAngle = radToDeg(approachingParameter.climbAngle);
	approachingParameter.yawAngle = radToDeg(approachingParameter.yawAngle);
	approachingParameter.angleOfSideslip = radToDeg(approachingParameter.angleOfSideslip);
}
