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
 * AdaptiveControlEnvironmentTest.cpp
 *
 *  Created on: May 29, 2019
 *      Author: simonyu
 */

#include <iostream>
#include <boost/test/unit_test.hpp>

#include "uavAP/Core/LinearAlgebra.h"
#include "uavAP/FlightControl/Controller/AdaptiveControlEnvironment/AdaptiveControlElements/Input.hpp"
#include "uavAP/FlightControl/Controller/AdaptiveControlEnvironment/AdaptiveControlElements/Gain.hpp"
#include "uavAP/FlightControl/Controller/AdaptiveControlEnvironment/AdaptiveControlElements/Sum.hpp"
#include "uavAP/FlightControl/Controller/AdaptiveControlEnvironment/EvaluableAdaptiveControlElements/StateSpace.hpp"

BOOST_AUTO_TEST_SUITE(AdaptiveControlEnvironmentTest)

BOOST_AUTO_TEST_CASE(AdaptiveLaw)
{
	double yTilde = 1;
	Vector2 M;
	Matrix2 phiInv;
	double negativeOne = -1;
	Vector2 result;

	M << 0.9712, 0.0038;
	phiInv << 14.0568, 87.7019, -19.9110, -3.4262;

	auto input = std::make_shared<Input<double>>(yTilde);
	auto gainOne = std::make_shared<Gain<double, Vector2, Vector2>>(input, M);
	auto gainTwo = std::make_shared<Gain<Vector2, Matrix2, Vector2>>(gainOne, phiInv);
	auto gainThree = std::make_shared<Gain<Vector2, double, Vector2>>(gainTwo, negativeOne);

	result = gainThree->getValue();

	BOOST_CHECK_CLOSE(result.x(), -13.99, 1);
	BOOST_CHECK_CLOSE(result.y(), 19.35, 1);
}

BOOST_AUTO_TEST_CASE(ControlLaw)
{
	Scalar r;
	double a0 = -0.1835;
	Vector2 state;
	Vector2 sighat;
	Matrix2 matrixA;
	Matrix2 matrixB;
	RowVector2 matrixC;
	RowVector2 matrixD;
	Scalar output;

	r << 1;
	state << 0, 0;
	sighat << 1, 1;
	matrixA << 0.2354, -0.3395, -0.2027, 0.8230;
	matrixB << 0.0077, 0.0566, -0.0156, 0.0272;
	matrixC << 6.5658, 2.0057;
	matrixD << 0, 0;
	output << 0;

	auto inputR = std::make_shared<Input<Scalar>>(r);
	auto inputSighat = std::make_shared<Input<Vector2>>(sighat);
	auto gain = std::make_shared<Gain<Scalar, double, Scalar>>(inputR, a0);
	auto stateSpace = std::make_shared<
			StateSpace<Vector2, Vector2, Matrix2, Matrix2, RowVector2, RowVector2, Scalar>>(state,
			inputSighat, matrixA, matrixB, matrixC, matrixD, output);
	auto sum = std::make_shared<Sum<Scalar, Scalar, Scalar>>(gain, stateSpace, false);

	BOOST_CHECK_CLOSE(sum->getValue().x(), -0.1835, 1);

	for (int i = 0; i < 5; i++)
	{
		stateSpace->evaluate();
	}

	BOOST_CHECK_CLOSE(sum->getValue().x(), -0.7229, 1);

	for (int i = 0; i < 5; i++)
	{
		stateSpace->evaluate();
	}

	BOOST_CHECK_CLOSE(sum->getValue().x(), -0.7455, 1);

	for (int i = 0; i < 5; i++)
	{
		stateSpace->evaluate();
	}

	BOOST_CHECK_CLOSE(sum->getValue().x(), -0.7605, 1);
}

BOOST_AUTO_TEST_SUITE_END()
