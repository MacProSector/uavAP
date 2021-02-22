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
 * EmergencyLandingModelTest.cpp
 *
 *  Created on: Feb 22, 2021
 *      Author: simonyu
 */

#include <boost/test/unit_test.hpp>

#include "uavAP/MissionControl/EmergencyPlanner/EmergencyLandingPlanner/EmergencyLandingModel.h"

BOOST_AUTO_TEST_SUITE(EmergencyLandingModelTest)

BOOST_AUTO_TEST_CASE(TenSeconds)
{
	double period = 0.01;
	double stopTime = 10;
	int steps = stopTime / period;

	double aileronDeflection = 0.01;
	double elevatorDeflection = 0.01;

	EmergencyLandingStatus landingStatus;

	landingStatus.position.x() = 20000;
	landingStatus.position.y() = 20000;
	landingStatus.position.z() = 8000;
	landingStatus.airSpeed = 140;
	landingStatus.climbAngle = 0;
	landingStatus.yawAngle = 0;

	auto landingModel = std::make_shared<EmergencyLandingModel>();

	landingModel->configure(period, landingStatus);

	for (int i = 0; i < steps; i++)
	{
		landingModel->update(aileronDeflection, elevatorDeflection);
	}

	landingStatus = landingModel->getLandingStatus();

	BOOST_CHECK_CLOSE(landingStatus.position.x(), 21319.5417273138, 1);
	BOOST_CHECK_CLOSE(landingStatus.position.y(), 20178.0045514481, 1);
	BOOST_CHECK_CLOSE(landingStatus.position.z(), 8140.36025021809, 1);
	BOOST_CHECK_CLOSE(landingStatus.airSpeed, 128.962816276824, 1);
	BOOST_CHECK_CLOSE(landingStatus.climbAngle, 0.167854503500166, 1);
	BOOST_CHECK_CLOSE(landingStatus.yawAngle, 0.40969614155454, 1);
}

BOOST_AUTO_TEST_CASE(OneHundredSeconds)
{
	double period = 0.01;
	double stopTime = 100;
	int steps = stopTime / period;

	double aileronDeflection = 0.01;
	double elevatorDeflection = 0.01;

	EmergencyLandingStatus landingStatus;

	landingStatus.position.x() = 20000;
	landingStatus.position.y() = 20000;
	landingStatus.position.z() = 8000;
	landingStatus.airSpeed = 140;
	landingStatus.climbAngle = 0;
	landingStatus.yawAngle = 0;

	auto landingModel = std::make_shared<EmergencyLandingModel>();

	landingModel->configure(period, landingStatus);

	for (int i = 0; i < steps; i++)
	{
		landingModel->update(aileronDeflection, elevatorDeflection);
	}

	landingStatus = landingModel->getLandingStatus();

	BOOST_CHECK_CLOSE(landingStatus.position.x(), 21449.1017677875, 1);
	BOOST_CHECK_CLOSE(landingStatus.position.y(), 21171.1577901631, 1);
	BOOST_CHECK_CLOSE(landingStatus.position.z(), 8592.20432484013, 1);
	BOOST_CHECK_CLOSE(landingStatus.airSpeed, 56.2239116834816, 1);
	BOOST_CHECK_CLOSE(landingStatus.climbAngle, 0.0370265738760054, 1);
	BOOST_CHECK_CLOSE(landingStatus.yawAngle, 26.2686110165689, 1);
}

BOOST_AUTO_TEST_SUITE_END()
