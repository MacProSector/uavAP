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
#include "uavAP/FlightControl/Controller/AdaptiveControlEnvironment/AdaptiveControlEnvironment.h"
#include "uavAP/FlightControl/Controller/AdaptiveControlEnvironment/AdaptiveControlElements/Constant.hpp"
#include "uavAP/FlightControl/Controller/AdaptiveControlEnvironment/AdaptiveControlElements/Gain.hpp"
#include "uavAP/FlightControl/Controller/AdaptiveControlEnvironment/AdaptiveControlElements/Input.hpp"
#include "uavAP/FlightControl/Controller/AdaptiveControlEnvironment/AdaptiveControlElements/ManualSwitch.hpp"
#include "uavAP/FlightControl/Controller/AdaptiveControlEnvironment/AdaptiveControlElements/Saturation.hpp"
#include "uavAP/FlightControl/Controller/AdaptiveControlEnvironment/AdaptiveControlElements/Sum.hpp"
#include "uavAP/FlightControl/Controller/AdaptiveControlEnvironment/EvaluableAdaptiveControlElements/LowPassFilter.hpp"
#include "uavAP/FlightControl/Controller/AdaptiveControlEnvironment/EvaluableAdaptiveControlElements/Output.hpp"
#include "uavAP/FlightControl/Controller/AdaptiveControlEnvironment/EvaluableAdaptiveControlElements/StateSpace.hpp"

BOOST_AUTO_TEST_SUITE(AdaptiveControlEnvironmentTest)

BOOST_AUTO_TEST_CASE(ConstantElement)
{
	double value = 1.5;
	double result = 0;
	AdaptiveControlEnvironment controlEnvironment;

	auto constant = controlEnvironment.addConstant<double>(value);

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
	AdaptiveControlEnvironment controlEnvironment;

	auto inputOne = controlEnvironment.addInput<double>(&valueInputOne);
	auto inputTwo = controlEnvironment.addInput<double>(&valueInputTwo);
	auto gain = controlEnvironment.addGain<double, double, double>(inputOne, valueGain);

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
	AdaptiveControlEnvironment controlEnvironment;

	auto input = controlEnvironment.addInput<double>(&valueOne);

	result = input->getValue();

	BOOST_CHECK_EQUAL(result, valueOne);

	valueOne = 2.0;
	result = input->getValue();

	BOOST_CHECK_EQUAL(result, valueOne);

	input->setInput(&valueTwo);
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
	AdaptiveControlEnvironment controlEnvironment;

	auto inputTrueOne = controlEnvironment.addInput<double>(&valueInputTrueOne);
	auto inputTrueTwo = controlEnvironment.addInput<double>(&valueInputTrueTwo);
	auto inputFalseOne = controlEnvironment.addInput<double>(&valueInputFalseOne);
	auto inputFalseTwo = controlEnvironment.addInput<double>(&valueInputFalseTwo);
	auto manualSwitch = controlEnvironment.addManualSwitch<double>(inputTrueOne, inputFalseOne,
			true);

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

BOOST_AUTO_TEST_CASE(SaturationElement)
{
	double valueInputOne = 25;
	double valueInputTwo = -60;
	double valueSaturationMin = -30;
	double valueSaturationMax = 15;
	double valueHardSaturationMin = -40;
	double valueHardSaturationMax = 20;
	double result = 0;
	AdaptiveControlEnvironment controlEnvironment;

	auto inputOne = controlEnvironment.addInput<double>(&valueInputOne);
	auto inputTwo = controlEnvironment.addInput<double>(&valueInputTwo);
	auto saturation = controlEnvironment.addSaturation<double>(inputOne, valueSaturationMin,
			valueSaturationMax, valueHardSaturationMin, valueHardSaturationMax);

	result = saturation->getValue();

	BOOST_CHECK_EQUAL(result, 15);

	saturation->overrideSaturationValue(valueSaturationMin, valueSaturationMax * 2);
	result = saturation->getValue();

	BOOST_CHECK_EQUAL(result, 20);

	saturation->setInput(inputTwo);
	result = saturation->getValue();

	BOOST_CHECK_EQUAL(result, -30);

	saturation->overrideSaturationValue(valueSaturationMin * 2, valueSaturationMax);
	result = saturation->getValue();

	BOOST_CHECK_EQUAL(result, -40);

	saturation->disableOverride();
	result = saturation->getValue();

	BOOST_CHECK_EQUAL(result, -30);
}

BOOST_AUTO_TEST_CASE(SumElement)
{
	double valueInputOneOne = 1.5;
	double valueInputTwoOne = 2.0;
	double valueInputOneTwo = 3.0;
	double valueInputTwoTwo = 4.0;
	double result = 0;
	AdaptiveControlEnvironment controlEnvironment;

	auto inputOneOne = controlEnvironment.addInput<double>(&valueInputOneOne);
	auto inputTwoOne = controlEnvironment.addInput<double>(&valueInputTwoOne);
	auto inputOneTwo = controlEnvironment.addInput<double>(&valueInputOneTwo);
	auto inputTwoTwo = controlEnvironment.addInput<double>(&valueInputTwoTwo);
	auto sum = controlEnvironment.addSum<double, double, double>(inputOneOne, inputTwoOne, true);

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

BOOST_AUTO_TEST_CASE(LowPassFilterElement)
{
	double valueInputOne = 1.0;
	double valueInputTwo = 15;
	double valueAlpha = 0.5;
	double result = 0;
	double resultCheck = 0;
	LowPassDataFilter filterCheck(valueInputOne, valueAlpha);
	AdaptiveControlEnvironment controlEnvironment;

	auto inputOne = controlEnvironment.addInput<double>(&valueInputOne);
	auto inputTwo = controlEnvironment.addInput<double>(&valueInputTwo);
	auto filter = controlEnvironment.addLowPassFilter<double>(inputOne, valueAlpha);

	result = filter->getValue();
	resultCheck = filterCheck.getFilteredData();

	BOOST_CHECK_EQUAL(result, resultCheck);

	for (double i = 0; i < 5; i++)
	{
		valueInputOne += 0.1 * i;
		controlEnvironment.evaluate();
		filterCheck.filterData(inputOne->getValue());
	}

	result = filter->getValue();
	resultCheck = filterCheck.getFilteredData();

	BOOST_CHECK_EQUAL(result, resultCheck);

	filter->setAlpha(0.7);
	filterCheck.tune(0.7);

	for (double i = 0; i < 5; i++)
	{
		valueInputOne += 0.2 * i;
		controlEnvironment.evaluate();
		filterCheck.filterData(inputOne->getValue());
	}

	result = filter->getValue();
	resultCheck = filterCheck.getFilteredData();

	BOOST_CHECK_EQUAL(result, resultCheck);

	filter->setInput(inputTwo);
	filter->setInitialValue(inputTwo->getValue());
	filterCheck.initialize(inputTwo->getValue());

	for (double i = 0; i < 10; i++)
	{
		valueInputOne += i;
		controlEnvironment.evaluate();
		filterCheck.filterData(inputTwo->getValue());
	}

	result = filter->getValue();
	resultCheck = filterCheck.getFilteredData();

	BOOST_CHECK_EQUAL(result, resultCheck);
}

BOOST_AUTO_TEST_CASE(OutputElement)
{
	double valueInputOne = 1.5;
	double valueInputTwo = 3.0;
	double resultOne = -5;
	double resultTwo = 5;
	double period = 1000;
	double time = 0;
	TimePoint timestamp;
	TimePoint current;
	AdaptiveControlEnvironment controlEnvironment;

	auto inputOne = controlEnvironment.addInput<double>(&valueInputOne);
	auto inputTwo = controlEnvironment.addInput<double>(&valueInputTwo);
	auto output = controlEnvironment.addOutput<double, double>(inputOne, &resultOne);

	for (int i = 0; i < 5; i++)
	{
		controlEnvironment.evaluate();
	}

	BOOST_CHECK_EQUAL(resultOne, valueInputOne);

	output->setInput(inputTwo);
	output->setOutput(&resultTwo);

	for (int i = 0; i < 5; i++)
	{
		controlEnvironment.evaluate();
	}

	BOOST_CHECK_EQUAL(resultTwo, valueInputTwo);

	output->setWaveform(Waveforms::SINE);
	output->setPeriod(period);
	output->setPhase(500);
	output->overrideOutput(1);
	timestamp = boost::posix_time::microsec_clock::local_time();
	current = boost::posix_time::microsec_clock::local_time();
	time = (current - timestamp).total_milliseconds();

	while (time < (period / 4))
	{
		current = boost::posix_time::microsec_clock::local_time();
		time = (current - timestamp).total_milliseconds();
		controlEnvironment.evaluate();
	}

	BOOST_CHECK_CLOSE(resultTwo, -1, 1);

	while (time < (period / 2))
	{
		current = boost::posix_time::microsec_clock::local_time();
		time = (current - timestamp).total_milliseconds();
		controlEnvironment.evaluate();
	}

	BOOST_CHECK_EQUAL(resultTwo < 0.00001, true);

	while (time < (period * 3 / 4))
	{
		current = boost::posix_time::microsec_clock::local_time();
		time = (current - timestamp).total_milliseconds();
		controlEnvironment.evaluate();
	}

	BOOST_CHECK_CLOSE(resultTwo, 1, 1);

	while (time < period)
	{
		current = boost::posix_time::microsec_clock::local_time();
		time = (current - timestamp).total_milliseconds();
		controlEnvironment.evaluate();
	}

	BOOST_CHECK_EQUAL(resultTwo < 0.00001, true);
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
	AdaptiveControlEnvironment controlEnvironment;

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

	auto inputOne = controlEnvironment.addInput<Vector2>(&vectorInputOne);
	auto inputTwo = controlEnvironment.addInput<Vector2>(&vectorInputTwo);
	auto stateSpace = controlEnvironment.addStateSpace<Vector2, Vector2, Matrix2, Matrix2,
			RowVector2, RowVector2, Scalar>(vectorState, inputOne, matrixAOne, matrixBOne,
			matrixCOne, matrixDOne, scalarOutput);

	controlEnvironment.evaluate();
	result = stateSpace->getValue().x();

	BOOST_CHECK_EQUAL(result, 0);

	controlEnvironment.evaluate();
	result = stateSpace->getValue().x();

	BOOST_CHECK_CLOSE(result, 0.4453, 1);

	controlEnvironment.evaluate();
	result = stateSpace->getValue().x();

	BOOST_CHECK_CLOSE(result, 0.5117, 1);

	stateSpace->setState(vectorState);
	stateSpace->setInput(inputTwo);
	stateSpace->setMatrixA(matrixATwo);
	stateSpace->setMatrixB(matrixBTwo);
	stateSpace->setMatrixC(matrixCTwo);
	stateSpace->setMatrixD(matrixDTwo);
	stateSpace->setOutput(scalarOutput);

	controlEnvironment.evaluate();
	result = stateSpace->getValue().x();

	BOOST_CHECK_EQUAL(result, 0);

	controlEnvironment.evaluate();
	result = stateSpace->getValue().x();

	BOOST_CHECK_CLOSE(result, -0.1073, 1);

	controlEnvironment.evaluate();
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
	AdaptiveControlEnvironment controlEnvironment;

	M << 0.9712, 0.0038;
	phiInv << 14.0568, 87.7019, -19.9110, -3.4262;

	auto input = controlEnvironment.addInput<double>(&yTilde);
	auto gainOne = controlEnvironment.addGain<double, Vector2, Vector2>(input, M);
	auto gainTwo = controlEnvironment.addGain<Vector2, Matrix2, Vector2>(gainOne, phiInv);
	auto gainThree = controlEnvironment.addGain<Vector2, double, Vector2>(gainTwo, negativeOne);

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
	AdaptiveControlEnvironment controlEnvironment;

	r << 1;
	state << 0, 0;
	sighat << 1, 1;
	matrixA << 0.2354, -0.3395, -0.2027, 0.8230;
	matrixB << 0.0077, 0.0566, -0.0156, 0.0272;
	matrixC << 6.5658, 2.0057;
	matrixD << 0, 0;
	output << 0;

	auto inputR = controlEnvironment.addInput<Scalar>(&r);
	auto inputSighat = controlEnvironment.addInput<Vector2>(&sighat);
	auto gain = controlEnvironment.addGain<Scalar, double, Scalar>(inputR, a0);
	auto stateSpace = controlEnvironment.addStateSpace<Vector2, Vector2, Matrix2, Matrix2, RowVector2, RowVector2, Scalar>(state,
			inputSighat, matrixA, matrixB, matrixC, matrixD, output);
	auto sum = controlEnvironment.addSum<Scalar, Scalar, Scalar>(gain, stateSpace, false);

	result = sum->getValue().x();

	BOOST_CHECK_CLOSE(result, -0.1835, 1);

	for (int i = 0; i < 5; i++)
	{
		controlEnvironment.evaluate();
	}

	result = sum->getValue().x();

	BOOST_CHECK_CLOSE(result, -0.7229, 1);

	for (int i = 0; i < 5; i++)
	{
		controlEnvironment.evaluate();
	}

	result = sum->getValue().x();

	BOOST_CHECK_CLOSE(result, -0.7455, 1);

	for (int i = 0; i < 5; i++)
	{
		controlEnvironment.evaluate();
	}

	result = sum->getValue().x();

	BOOST_CHECK_CLOSE(result, -0.7605, 1);
}

BOOST_AUTO_TEST_SUITE_END()
