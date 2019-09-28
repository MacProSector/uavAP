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
 * EmergencyLandingPlannerTest.cpp
 *
 *  Created on: Jul 22, 2019
 *      Author: simonyu
 */

#include <boost/test/unit_test.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>

#include "uavAP/Core/PropertyMapper/PropertyMapper.h"
#include "uavAP/MissionControl/EmergencyPlanner/EmergencyLandingPlanner/EmergencyLandingPlanner.h"

BOOST_AUTO_TEST_SUITE(EmergencyLandingPlannerTest)

BOOST_AUTO_TEST_CASE(Configuration)
{
	boost::property_tree::ptree config;
	boost::property_tree::ptree landingPlannerConfig;
	boost::property_tree::read_json("MissionControl/EmergencyPlanner/config/mission_control.json",
			config);

	PropertyMapper pm(config);
	pm.add("emergency_landing_planner", landingPlannerConfig, true);

	BOOST_CHECK_EQUAL(pm.map(), true);

	auto landingPlanner = EmergencyLandingPlanner::create(landingPlannerConfig);

	std::vector<EmergencyLandingParameter> landingParameters =
			landingPlanner->getEmergencyLandingParameters();

	BOOST_CHECK_EQUAL(landingParameters.size(), 3);

	EmergencyLandingParameter landingParameter;
	Vector3 approachingPosition;
	Vector3 obstacleOneSpecification;
	Vector3 obstacleTwoSpecification;

	landingParameter = landingParameters[0];
	approachingPosition << 100, 200, 10;
	obstacleOneSpecification << 1000, 1000, 200;

	BOOST_CHECK_EQUAL(landingParameter.planningParameter.acceleration, 0.1524);
	BOOST_CHECK_EQUAL(landingParameter.planningParameter.climbRate, degToRad(0.3));
	BOOST_CHECK_EQUAL(landingParameter.planningParameter.yawRate, degToRad(5));
	BOOST_CHECK_EQUAL(landingParameter.planningParameter.glidingVelocity, 36);
	BOOST_CHECK_EQUAL(landingParameter.planningParameter.glidingClimbAngle, degToRad(-5));
	BOOST_CHECK_EQUAL(landingParameter.planningParameter.period, 10000);
	BOOST_CHECK_EQUAL(landingParameter.planningParameter.periodSecond, 10);

	BOOST_CHECK_EQUAL(landingParameter.searchingParameter.acceleration, 0.9144);
	BOOST_CHECK_EQUAL(landingParameter.searchingParameter.climbRate, degToRad(0.25));
	BOOST_CHECK_EQUAL(landingParameter.searchingParameter.yawRate, degToRad(5));
	BOOST_CHECK_EQUAL(landingParameter.searchingParameter.cruiseRadius, 2621.28);
	BOOST_CHECK_EQUAL(landingParameter.searchingParameter.loiterRadius, 2438.4);
	BOOST_CHECK_EQUAL(landingParameter.searchingParameter.loiterRadiusOffset, 6);
	BOOST_CHECK_EQUAL(landingParameter.searchingParameter.loiterLambda, degToRad(22.9183));
	BOOST_CHECK_EQUAL(landingParameter.searchingParameter.lineDistance, 1828.8);
	BOOST_CHECK_EQUAL(landingParameter.searchingParameter.lineDelta, 609.6);
	BOOST_CHECK_EQUAL(landingParameter.searchingParameter.approachAltitude, 304.8);

	BOOST_CHECK_EQUAL(landingParameter.approachingParameter.velocity, 21.336);
	BOOST_CHECK_EQUAL(landingParameter.approachingParameter.climbAngle, degToRad(-3));
	BOOST_CHECK_EQUAL(landingParameter.approachingParameter.yawAngle, degToRad(5));
	BOOST_CHECK_EQUAL(landingParameter.approachingParameter.yawRate, degToRad(5));
	BOOST_CHECK_EQUAL(landingParameter.approachingParameter.angleOfSideslip, degToRad(5));
	BOOST_CHECK_EQUAL(landingParameter.approachingParameter.position, approachingPosition);
	BOOST_CHECK_EQUAL(landingParameter.approachingParameter.obstacles.size(), 1);
	BOOST_CHECK_EQUAL(landingParameter.approachingParameter.obstacles[0].first, "farm_house");
	BOOST_CHECK_EQUAL(landingParameter.approachingParameter.obstacles[0].second,
			obstacleOneSpecification);

	landingParameter = landingParameters[1];
	approachingPosition << 2, 4, 6;
	obstacleOneSpecification << 100, 100, 20;
	obstacleTwoSpecification << 2000, 2000, 400;

	BOOST_CHECK_EQUAL(landingParameter.planningParameter.acceleration, 0.1524);
	BOOST_CHECK_EQUAL(landingParameter.planningParameter.climbRate, degToRad(1000));
	BOOST_CHECK_EQUAL(landingParameter.planningParameter.yawRate, degToRad(5));
	BOOST_CHECK_EQUAL(landingParameter.planningParameter.glidingVelocity, 36);
	BOOST_CHECK_EQUAL(landingParameter.planningParameter.glidingClimbAngle, degToRad(-5));
	BOOST_CHECK_EQUAL(landingParameter.planningParameter.period, 50000);
	BOOST_CHECK_EQUAL(landingParameter.planningParameter.periodSecond, 50);

	BOOST_CHECK_EQUAL(landingParameter.searchingParameter.climbRate, degToRad(0.25));
	BOOST_CHECK_EQUAL(landingParameter.searchingParameter.yawRate, degToRad(5));
	BOOST_CHECK_EQUAL(landingParameter.searchingParameter.cruiseRadius, 2621.28);
	BOOST_CHECK_EQUAL(landingParameter.searchingParameter.loiterRadius, 2438.4);
	BOOST_CHECK_EQUAL(landingParameter.searchingParameter.loiterRadiusOffset, 6);
	BOOST_CHECK_EQUAL(landingParameter.searchingParameter.loiterLambda, degToRad(22.9183));
	BOOST_CHECK_EQUAL(landingParameter.searchingParameter.lineDistance, 2);
	BOOST_CHECK_EQUAL(landingParameter.searchingParameter.lineDelta, 609.6);
	BOOST_CHECK_EQUAL(landingParameter.searchingParameter.approachAltitude, 4);

	BOOST_CHECK_EQUAL(landingParameter.approachingParameter.velocity, 50);
	BOOST_CHECK_EQUAL(landingParameter.approachingParameter.climbAngle, degToRad(-3));
	BOOST_CHECK_EQUAL(landingParameter.approachingParameter.yawAngle, degToRad(5));
	BOOST_CHECK_EQUAL(landingParameter.approachingParameter.yawRate, degToRad(5));
	BOOST_CHECK_EQUAL(landingParameter.approachingParameter.angleOfSideslip, degToRad(5));
	BOOST_CHECK_EQUAL(landingParameter.approachingParameter.position, approachingPosition);
	BOOST_CHECK_EQUAL(landingParameter.approachingParameter.obstacles.size(), 2);
	BOOST_CHECK_EQUAL(landingParameter.approachingParameter.obstacles[0].first, "farm_house_one");
	BOOST_CHECK_EQUAL(landingParameter.approachingParameter.obstacles[0].second,
			obstacleOneSpecification);
	BOOST_CHECK_EQUAL(landingParameter.approachingParameter.obstacles[1].first, "farm_house_two");
	BOOST_CHECK_EQUAL(landingParameter.approachingParameter.obstacles[1].second,
			obstacleTwoSpecification);

	landingParameter = landingParameters[2];
	approachingPosition << 100, 200, 10;
	obstacleOneSpecification << 1000, 1000, 200;

	BOOST_CHECK_EQUAL(landingParameter.planningParameter.acceleration, 0.1524);
	BOOST_CHECK_EQUAL(landingParameter.planningParameter.climbRate, degToRad(0.3));
	BOOST_CHECK_EQUAL(landingParameter.planningParameter.yawRate, degToRad(10));
	BOOST_CHECK_EQUAL(landingParameter.planningParameter.glidingVelocity, 36);
	BOOST_CHECK_EQUAL(landingParameter.planningParameter.glidingClimbAngle, degToRad(-5));
	BOOST_CHECK_EQUAL(landingParameter.planningParameter.period, 10000);
	BOOST_CHECK_EQUAL(landingParameter.planningParameter.periodSecond, 10);

	BOOST_CHECK_EQUAL(landingParameter.searchingParameter.acceleration, 0.9144);
	BOOST_CHECK_EQUAL(landingParameter.searchingParameter.climbRate, degToRad(0.25));
	BOOST_CHECK_EQUAL(landingParameter.searchingParameter.yawRate, degToRad(5));
	BOOST_CHECK_EQUAL(landingParameter.searchingParameter.cruiseRadius, 2621.28);
	BOOST_CHECK_EQUAL(landingParameter.searchingParameter.loiterRadius, 2438.4);
	BOOST_CHECK_EQUAL(landingParameter.searchingParameter.loiterRadiusOffset, 6);
	BOOST_CHECK_EQUAL(landingParameter.searchingParameter.loiterLambda, degToRad(22.9183));
	BOOST_CHECK_EQUAL(landingParameter.searchingParameter.lineDistance, 1828.8);
	BOOST_CHECK_EQUAL(landingParameter.searchingParameter.lineDelta, 609.6);
	BOOST_CHECK_EQUAL(landingParameter.searchingParameter.approachAltitude, 304.8);

	BOOST_CHECK_EQUAL(landingParameter.approachingParameter.velocity, 21.336);
	BOOST_CHECK_EQUAL(landingParameter.approachingParameter.climbAngle, degToRad(-3));
	BOOST_CHECK_EQUAL(landingParameter.approachingParameter.yawAngle, degToRad(5));
	BOOST_CHECK_EQUAL(landingParameter.approachingParameter.yawRate, degToRad(5));
	BOOST_CHECK_EQUAL(landingParameter.approachingParameter.angleOfSideslip, degToRad(5));
	BOOST_CHECK_EQUAL(landingParameter.approachingParameter.position, approachingPosition);
	BOOST_CHECK_EQUAL(landingParameter.approachingParameter.obstacles.size(), 1);
	BOOST_CHECK_EQUAL(landingParameter.approachingParameter.obstacles[0].first, "farm_house");
	BOOST_CHECK_EQUAL(landingParameter.approachingParameter.obstacles[0].second,
			obstacleOneSpecification);
}

BOOST_AUTO_TEST_SUITE_END()
