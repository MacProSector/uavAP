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
 * EmergencyLandingParameter.h
 *
 *  Created on: Jul 17, 2019
 *      Author: simonyu
 */

#ifndef UAVAP_MISSIONCONTROL_EMERGENCYPLANNER_EMERGENCYLANDINGPLANNER_EMERGENCYLANDINGPARAMETER_H_
#define UAVAP_MISSIONCONTROL_EMERGENCYPLANNER_EMERGENCYLANDINGPLANNER_EMERGENCYLANDINGPARAMETER_H_

#include <boost/property_tree/ptree.hpp>

#include "uavAP/Core/Scheduler/Event.h"
#include "uavAP/Core/LinearAlgebra.h"
#include "uavAP/Core/EnumMap.hpp"

enum class EmergencyLandingPhases
{
	INVALID, CRUISING, DESCENDING, APPROACHING, NUM_PHASES
};

ENUMMAP_INIT(EmergencyLandingPhases, { { EmergencyLandingPhases::CRUISING, "cruising"},
		{ EmergencyLandingPhases::DESCENDING, "descending"}, { EmergencyLandingPhases::APPROACHING,
		"approaching"} });

using Obstacle = std::pair<std::string, Vector3>;
using Obstacles = std::vector<Obstacle>;

struct PlanningParameter
{
	double acceleration;
	double climbRate;
	double yawRate;
	double glidingVelocity;
	double glidingClimbAngle;
	double period;
	double periodSecond;
	bool isDefault;

	bool
	configure(const boost::property_tree::ptree& config);
};

struct SearchingParameter
{
	double acceleration;
	double climbRate;
	double yawRate;
	double loiterRadius;
	double loiterLambda;
	double lineDistance;
	double lineDelta;
	double approachRadius;
	double approachAltitude;
	bool isDefault;

	bool
	configure(const boost::property_tree::ptree& config);
};

struct ApproachingParameter
{
	Vector3 position;
	double velocity;
	double climbAngle;
	double yawAngle;
	double angleOfSideslip;
	Obstacles obstacles;
	bool isDefault;

	bool
	configure(const boost::property_tree::ptree& config);
};

struct EmergencyLandingParameter
{
	std::string name;
	PlanningParameter planningParameter;
	SearchingParameter searchingParameter;
	ApproachingParameter approachingParameter;
	EmergencyLandingPhases phase;
	Event schedulingEvent;
	bool isDefault;

	bool
	configure(const boost::property_tree::ptree& config);
};

void
degreeToRadian(PlanningParameter& planningParameter);

void
degreeToRadian(SearchingParameter& searchingParameter);

void
degreeToRadian(ApproachingParameter& approachingParameter);

void
radianToDegree(PlanningParameter& planningParameter);

void
radianToDegree(SearchingParameter& searchingParameter);

void
radianToDegree(ApproachingParameter& approachingParameter);

#endif /* UAVAP_MISSIONCONTROL_EMERGENCYPLANNER_EMERGENCYLANDINGPLANNER_EMERGENCYLANDINGPARAMETER_H_ */
