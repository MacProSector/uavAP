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
#include "uavAP/FlightControl/Controller/AdaptiveControlEnvironment/AdaptiveControlElements/Constant.hpp"
#include "uavAP/FlightControl/Controller/AdaptiveControlEnvironment/AdaptiveControlElements/Gain.hpp"
#include "uavAP/FlightControl/Controller/AdaptiveControlEnvironment/AdaptiveControlElements/Input.hpp"
#include "uavAP/FlightControl/Controller/AdaptiveControlEnvironment/AdaptiveControlElements/ManualSwitch.hpp"
#include "uavAP/FlightControl/Controller/AdaptiveControlEnvironment/AdaptiveControlElements/Sum.hpp"
#include "uavAP/FlightControl/Controller/AdaptiveControlEnvironment/EvaluableAdaptiveControlElements/StateSpace.hpp"

BOOST_AUTO_TEST_SUITE(AdaptiveControlEnvironmentTest)

BOOST_AUTO_TEST_CASE(ConstantElement)
{
	double value = 1.5;
	double result = 0;

	auto constant = std::make_shared<Constant<double>>(value);

	result = constant->getValue();

	BOOST_CHECK_EQUAL(result, value);

	value = 2.0;
	constant->setConstant(value);
	result = constant->getValue();

	BOOST_CHECK_EQUAL(result, value);
}

BOOST_AUTO_TEST_CASE(GainElement)
{
	double valueInputOne = 1.5;
	double valueInputTwo = 3.0;
	double valueGain = 2.0;
	double result = 0;

	auto inputOne = std::make_shared<Input<double>>(valueInputOne);
	auto inputTwo = std::make_shared<Input<double>>(valueInputTwo);
	auto gain = std::make_shared<Gain<double, double, double>>(inputOne, valueGain);

	result = gain->getValue();

	BOOST_CHECK_EQUAL(result, valueInputOne * valueGain);

	valueGain = 3.0;
	gain->setGain(valueGain);
	result = gain->getValue();

	BOOST_CHECK_EQUAL(result, valueInputOne * valueGain);

	gain->setInput(inputTwo);
	result = gain->getValue();

	BOOST_CHECK_EQUAL(result, valueInputTwo * valueGain);
}

BOOST_AUTO_TEST_CASE(InputElement)
{
	double valueOne = 1.5;
	double valueTwo = 3.0;
	double result = 0;

	auto input = std::make_shared<Input<double>>(valueOne);

	result = input->getValue();

	BOOST_CHECK_EQUAL(result, valueOne);

	valueOne = 2.0;
	result = input->getValue();

	BOOST_CHECK_EQUAL(result, valueOne);

	input->setInput(valueTwo);
	result = input->getValue();

	BOOST_CHECK_EQUAL(result, valueTwo);
}

BOOST_AUTO_TEST_CASE(ManualSwitchElement)
{
	double valueInputTrueOne = 1.5;
	double valueInputTrueTwo = 3.0;
	double valueInputFalseOne = 2.0;
	double valueInputFalseTwo = 4.0;
	double result = 0;

	auto inputTrueOne = std::make_shared<Input<double>>(valueInputTrueOne);
	auto inputTrueTwo = std::make_shared<Input<double>>(valueInputTrueTwo);
	auto inputFalseOne = std::make_shared<Input<double>>(valueInputFalseOne);
	auto inputFalseTwo = std::make_shared<Input<double>>(valueInputFalseTwo);
	auto manualSwitch = std::make_shared<ManualSwitch<double>>(inputTrueOne, inputFalseOne, true);

	result = manualSwitch->getValue();

	BOOST_CHECK_EQUAL(result, valueInputTrueOne);

	manualSwitch->setSelection(false);
	result = manualSwitch->getValue();

	BOOST_CHECK_EQUAL(result, valueInputFalseOne);

	manualSwitch->setInputTrue(inputTrueTwo);
	manualSwitch->setInputFalse(inputFalseTwo);
	manualSwitch->setSelection(true);
	result = manualSwitch->getValue();

	BOOST_CHECK_EQUAL(result, valueInputTrueTwo);

	manualSwitch->setSelection(false);
	result = manualSwitch->getValue();

	BOOST_CHECK_EQUAL(result, valueInputFalseTwo);
}

BOOST_AUTO_TEST_CASE(SumElement)
{
	double valueInputOneOne = 1.5;
	double valueInputTwoOne = 2.0;
	double valueInputOneTwo = 3.0;
	double valueInputTwoTwo = 4.0;
	double result = 0;

	auto inputOneOne = std::make_shared<Input<double>>(valueInputOneOne);
	auto inputTwoOne = std::make_shared<Input<double>>(valueInputTwoOne);
	auto inputOneTwo = std::make_shared<Input<double>>(valueInputOneTwo);
	auto inputTwoTwo = std::make_shared<Input<double>>(valueInputTwoTwo);
	auto sum = std::make_shared<Sum<double, double, double>>(inputOneOne, inputTwoOne, true);

	result = sum->getValue();

	BOOST_CHECK_EQUAL(result, valueInputOneOne + valueInputTwoOne);

	sum->setAddition(false);
	result = sum->getValue();

	BOOST_CHECK_EQUAL(result, valueInputOneOne - valueInputTwoOne);

	sum->setInputOne(inputOneTwo);
	sum->setInputTwo(inputTwoTwo);
	sum->setAddition(true);
	result = sum->getValue();

	BOOST_CHECK_EQUAL(result, valueInputOneTwo + valueInputTwoTwo);
}

BOOST_AUTO_TEST_CASE(StateSpaceElement)
{
	Vector2 vectorState;
	Vector2 vectorInputOne;
	Vector2 vectorInputTwo;
	Matrix2 matrixAOne;
	Matrix2 matrixATwo;
	Matrix2 matrixBOne;
	Matrix2 matrixBTwo;
	RowVector2 matrixCOne;
	RowVector2 matrixCTwo;
	RowVector2 matrixDOne;
	RowVector2 matrixDTwo;
	Scalar scalarOutput;
	double result = 0;

	vectorState << 0, 0;
	vectorInputOne << 1, 1;
	vectorInputTwo << 2, 2;
	matrixAOne << 0.2354, -0.3395, -0.2027, 0.8230;
	matrixATwo << 0.9429, -0.0368, 0.0389, 0.9993;
	matrixBOne << 0.0077, 0.0566, -0.0156, 0.0272;
	matrixBTwo << 0.0194, -0.0004, 0.0004, 0.0200;
	matrixCOne << 6.5658, 2.0057;
	matrixCTwo << -0.0517, -2.5840;
	matrixDOne << 0, 0;
	matrixDTwo << 0, 0;
	scalarOutput << 0;

	auto inputOne = std::make_shared<Input<Vector2>>(vectorInputOne);
	auto inputTwo = std::make_shared<Input<Vector2>>(vectorInputTwo);
	auto stateSpace = std::make_shared<
			StateSpace<Vector2, Vector2, Matrix2, Matrix2, RowVector2, RowVector2, Scalar>>(
			vectorState, inputOne, matrixAOne, matrixBOne, matrixCOne, matrixDOne, scalarOutput);

	stateSpace->evaluate();
	result = stateSpace->getValue().x();

	BOOST_CHECK_EQUAL(result, 0);

	stateSpace->evaluate();
	result = stateSpace->getValue().x();

	BOOST_CHECK_CLOSE(result, 0.4453, 1);

	stateSpace->evaluate();
	result = stateSpace->getValue().x();

	BOOST_CHECK_CLOSE(result, 0.5117, 1);

	stateSpace->setState(vectorState);
	stateSpace->setInput(inputTwo);
	stateSpace->setMatrixA(matrixATwo);
	stateSpace->setMatrixB(matrixBTwo);
	stateSpace->setMatrixC(matrixCTwo);
	stateSpace->setMatrixD(matrixDTwo);
	stateSpace->setOutput(scalarOutput);

	stateSpace->evaluate();
	result = stateSpace->getValue().x();

	BOOST_CHECK_EQUAL(result, 0);

	stateSpace->evaluate();
	result = stateSpace->getValue().x();

	BOOST_CHECK_CLOSE(result, -0.1073, 1);

	stateSpace->evaluate();
	result = stateSpace->getValue().x();

	BOOST_CHECK_CLOSE(result, -0.2182, 1);
}

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
	double result = 0;

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

	result = sum->getValue().x();

	BOOST_CHECK_CLOSE(result, -0.1835, 1);

	for (int i = 0; i < 5; i++)
	{
		stateSpace->evaluate();
	}

	result = sum->getValue().x();

	BOOST_CHECK_CLOSE(result, -0.7229, 1);

	for (int i = 0; i < 5; i++)
	{
		stateSpace->evaluate();
	}

	result = sum->getValue().x();

	BOOST_CHECK_CLOSE(result, -0.7455, 1);

	for (int i = 0; i < 5; i++)
	{
		stateSpace->evaluate();
	}

	result = sum->getValue().x();

	BOOST_CHECK_CLOSE(result, -0.7605, 1);
}

BOOST_AUTO_TEST_SUITE_END()
