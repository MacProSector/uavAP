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
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>

#include "uavAP/Core/SensorData.h"
#include "uavAP/Core/LinearAlgebra.h"
#include "uavAP/Core/PropertyMapper/PropertyMapper.h"
#include "uavAP/FlightControl/Controller/ControllerTarget.h"
#include "uavAP/FlightControl/Controller/AdaptiveControlEnvironment/AdaptiveControlEnvironment.h"
#include "uavAP/FlightControl/Controller/AdaptiveController/L1AdaptiveController/L1AdaptiveParameter.h"

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

BOOST_AUTO_TEST_CASE(MuxElement)
{
	double valueInputOne = 1.5;
	double valueInputTwo = 3.0;
	double valueInputThree = 2.0;
	std::vector<AdaptiveElement<double>> input;
	Eigen::Matrix<double, 3, 1> result;
	Eigen::Matrix<double, 3, 1> vectorCheck;
	AdaptiveControlEnvironment controlEnvironment;

	vectorCheck << valueInputOne, valueInputTwo, valueInputThree;

	auto inputOne = controlEnvironment.addInput<double>(&valueInputOne);
	auto inputTwo = controlEnvironment.addInput<double>(&valueInputTwo);
	auto inputThree = controlEnvironment.addInput<double>(&valueInputThree);

	input.push_back(inputOne);
	input.push_back(inputTwo);
	input.push_back(inputThree);

	auto mux = controlEnvironment.addMux<double, Eigen::Matrix<double, 3, 1>>(input);

	result = mux->getValue();

	BOOST_CHECK_EQUAL(result, vectorCheck);
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

BOOST_AUTO_TEST_CASE(DemuxElement)
{
	double valueInputOne = 1.5;
	double valueInputTwo = 3.0;
	double valueInputThree = 2.0;
	Eigen::Matrix<double, 3, 1> vectorInput;
	std::vector<std::shared_ptr<Constant<double>>> output;
	AdaptiveControlEnvironment controlEnvironment;

	vectorInput << valueInputOne, valueInputTwo, valueInputThree;

	auto input = controlEnvironment.addInput<Eigen::Matrix<double, 3, 1>>(&vectorInput);
	auto constantOne = controlEnvironment.addConstant<double>(0);
	auto constantTwo = controlEnvironment.addConstant<double>(0);
	auto constantThree = controlEnvironment.addConstant<double>(0);

	output.push_back(constantOne);
	output.push_back(constantTwo);
	output.push_back(constantThree);

	auto demux = controlEnvironment.addDemux<Eigen::Matrix<double, 3, 1>, double>(input, output);

	controlEnvironment.evaluate();

	BOOST_CHECK_EQUAL(valueInputOne, constantOne->getValue());
	BOOST_CHECK_EQUAL(valueInputTwo, constantTwo->getValue());
	BOOST_CHECK_EQUAL(valueInputThree, constantThree->getValue());

	vectorInput *= 2;

	controlEnvironment.evaluate();

	BOOST_CHECK_EQUAL(valueInputOne * 2, constantOne->getValue());
	BOOST_CHECK_EQUAL(valueInputTwo * 2, constantTwo->getValue());
	BOOST_CHECK_EQUAL(valueInputThree * 2, constantThree->getValue());
}

BOOST_AUTO_TEST_CASE(FeedbackElement)
{
	double valueInput = 1.5;
	double valueGain = 2.0;
	double result = 0;
	int loop = 3;
	AdaptiveControlEnvironment controlEnvironment;

	auto feedback = controlEnvironment.addFeedback<double>();
	auto gain = controlEnvironment.addGain<double, double, double>(feedback, valueGain);

	feedback->setInput(gain);
	feedback->setOutput(valueInput);

	for (int i = 0; i < loop; i++)
	{
		controlEnvironment.evaluate();
	}

	result = feedback->getValue();

	BOOST_CHECK_EQUAL(result, valueInput * pow(valueGain, loop));
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
	auto stateSpace = controlEnvironment.addStateSpace<Vector2, Vector2, Matrix2, Matrix2,
			RowVector2, RowVector2, Scalar>(state, inputSighat, matrixA, matrixB, matrixC, matrixD,
			output);
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

BOOST_AUTO_TEST_CASE(PitchControl)
{
	PitchL1AdaptiveParameter parameter;
	PitchL1AdaptiveParameter parameterCheck;
	AdaptiveControlEnvironment controlEnvironment;
	ControllerOutput controllerOutput;
	boost::property_tree::ptree config;

	/* Parameter configuration */
	boost::property_tree::read_json("FlightControl/Controller/config/Adaptive_Pitch.json", config);

	PropertyMapper pm(config);

	pm.add<PitchL1AdaptiveParameter>("pitch", parameter, true);

	/* Check parameter configuration */
	parameterCheck.inputTrim << -0.0204, 0.000034859, -0.0204;
	parameterCheck.outputTrim << -0.0096;
	parameterCheck.inputGain << 0.432623897657296, -3.069112284374556, -3.162000908248524;
	parameterCheck.targetGain << -3.162277690781023;
	parameterCheck.predictorGain << -1.033800000000000, -38.008899999999997, -0.336100000000000;
	parameterCheck.adaptiveGain << -15.068460043805631, -11.309920647165402, -99.431509092904790;
	parameterCheck.controlLawMatrixA << 0.747910460324683, -0.083391970385317, -0.287771449143700, 0.182977708804847, -0.015382789122404, 0.905288017253303, 0.060295353347410, 0.234363993063759, -0.286248246258019, -0.102313310829525, 0.603639746521179, 0.208164671219345, 0.062680975691429, 0.023246448784124, 0.069627635633610, 0.856595887607014;
	parameterCheck.controlLawMatrixB << 0.001997246246091,  0.000363286354825, -0.006609632644596, 0.000192018277151,  0.000034699815639, -0.021227058859546, -0.001118363689036,  0.000524567449641,  0.000965923975990, 0.000567451063383, -0.000149613118455,  0.028795195046024;
	parameterCheck.controlLawMatrixC << 0.686236990612626, -4.150108169281860, -1.183070482982532, -3.362903471609596;
	parameterCheck.controlLawMatrixD << 0, 0, 0;
	parameterCheck.controlLawState << 0, 0, 0, 0;
	parameterCheck.predictorMatrixA << 0.970661041677723, -0.013273900733976, -0.022466611112179, -0.125340305985425,  0.304078111546135, -0.695023036922366, -0.001021439481865, -0.000440607566581,  0.989763156489849;
	parameterCheck.predictorMatrixB << 0.009850145373881, -0.000079849272022, -0.000126299264661, -0.000754866137152,  0.005840614982235, -0.004165238403006, -0.000005222019056, -0.000002657352984,  0.009948267008766;
	parameterCheck.predictorMatrixC << 0, 0, 1;
	parameterCheck.predictorMatrixD << 0, 0, 0;
	parameterCheck.predictorState << 0, 0, 0;

	/* Parameter configuration test */
	BOOST_CHECK_EQUAL(parameter.inputTrim, parameterCheck.inputTrim);
	BOOST_CHECK_EQUAL(parameter.outputTrim, parameterCheck.outputTrim);
	BOOST_CHECK_EQUAL(parameter.inputGain, parameterCheck.inputGain);
	BOOST_CHECK_EQUAL(parameter.targetGain, parameterCheck.targetGain);
	BOOST_CHECK_EQUAL(parameter.predictorGain, parameterCheck.predictorGain);
	BOOST_CHECK_EQUAL(parameter.adaptiveGain, parameterCheck.adaptiveGain);
	BOOST_CHECK_EQUAL(parameter.controlLawMatrixA, parameterCheck.controlLawMatrixA);
	BOOST_CHECK_EQUAL(parameter.controlLawMatrixB, parameterCheck.controlLawMatrixB);
	BOOST_CHECK_EQUAL(parameter.controlLawMatrixC, parameterCheck.controlLawMatrixC);
	BOOST_CHECK_EQUAL(parameter.controlLawMatrixD, parameterCheck.controlLawMatrixD);
	BOOST_CHECK_EQUAL(parameter.controlLawState, parameterCheck.controlLawState);
	BOOST_CHECK_EQUAL(parameter.predictorMatrixA, parameterCheck.predictorMatrixA);
	BOOST_CHECK_EQUAL(parameter.predictorMatrixB, parameterCheck.predictorMatrixB);
	BOOST_CHECK_EQUAL(parameter.predictorMatrixC, parameterCheck.predictorMatrixC);
	BOOST_CHECK_EQUAL(parameter.predictorMatrixD, parameterCheck.predictorMatrixD);
	BOOST_CHECK_EQUAL(parameter.predictorState, parameterCheck.predictorState);

	/* Pitch control level 1 */
	auto angleOfAttack = controlEnvironment.addConstant<double>(1);
	auto pitchRate = controlEnvironment.addConstant<double>(1);
	auto pitchAngle = controlEnvironment.addConstant<double>(1);

	std::vector<AdaptiveElement<double>> pitchControlInputMuxVector
	{ angleOfAttack, pitchRate, pitchAngle };

	auto pitchControlInputTrim = controlEnvironment.addConstant<Vector3>(parameter.inputTrim);

	auto pitchControlInputMux = controlEnvironment.addMux<double, Vector3>(pitchControlInputMuxVector);

	auto pitchControlInputSum = controlEnvironment.addSum<Vector3, Vector3, Vector3>(
			pitchControlInputMux, pitchControlInputTrim, false);

	auto pitchControlInputGain = controlEnvironment.addGain<Vector3, RowVector3, Scalar>(
			pitchControlInputSum, parameter.inputGain);

	/* Pitch control level 2 */
	auto pitchCommand = controlEnvironment.addConstant<double>(1);
	double pitchCommandSaturationValue = 0.523598775598299;

	auto pitchCommandSaturation = controlEnvironment.addSaturation<double>(pitchCommand,
			-pitchCommandSaturationValue, pitchCommandSaturationValue);

	auto pitchCommandGain = controlEnvironment.addGain<double, Scalar, Scalar>(pitchCommandSaturation,
			parameter.targetGain);

	auto adaptiveFeedback = controlEnvironment.addFeedback<Vector3>();

	auto controlLawStateSpace = controlEnvironment.addStateSpace<Vector4, Vector3, Matrix4,
			Matrix43, RowVector4, RowVector3, Scalar>(parameter.controlLawState, adaptiveFeedback,
			parameter.controlLawMatrixA, parameter.controlLawMatrixB, parameter.controlLawMatrixC,
			parameter.controlLawMatrixD, Scalar());

	auto controlLawSum = controlEnvironment.addSum<Scalar, Scalar, Scalar>(pitchCommandGain,
			controlLawStateSpace, false);

	/* Pitch control level 3 */
	auto predictorGain = controlEnvironment.addGain<Scalar, Vector3, Vector3>(controlLawSum,
			parameter.predictorGain);

	auto predictorInputSum = controlEnvironment.addSum<Vector3, Vector3, Vector3>(predictorGain,
			adaptiveFeedback, true);

	auto predictorStateSpace = controlEnvironment.addStateSpace<Vector3, Vector3, Matrix3, Matrix3,
			RowVector3, RowVector3, Scalar>(parameter.predictorState, predictorInputSum,
			parameter.predictorMatrixA, parameter.predictorMatrixB, parameter.predictorMatrixC,
			parameter.predictorMatrixD, Scalar());

	auto angleOfAttackConstant = controlEnvironment.addConstant<double>(0);
	auto pitchRateConstant = controlEnvironment.addConstant<double>(0);
	auto pitchAngleConstant = controlEnvironment.addConstant<double>(0);

	std::vector<std::shared_ptr<Constant<double>>> pitchControlInputDemuxVector
	{ angleOfAttackConstant, pitchRateConstant, pitchAngleConstant };

	auto longitudinalInputDemux = controlEnvironment.addDemux<Vector3, double>(pitchControlInputSum,
			pitchControlInputDemuxVector);

	std::vector<AdaptiveElement<double>> pitchAngleConstantVector
	{ pitchAngleConstant };

	auto pitchAngleConstantMux = controlEnvironment.addMux<double, Scalar>(pitchAngleConstantVector);

	auto predictorOutputSum = controlEnvironment.addSum<Scalar, Scalar, Scalar>(predictorStateSpace,
			pitchAngleConstantMux, false);

	auto adaptiveGain = controlEnvironment.addGain<Scalar, Vector3, Vector3>(predictorOutputSum,
			parameter.adaptiveGain);

	adaptiveFeedback->setInput(adaptiveGain);

	/* Pitch control output */
	auto pitchControlOutputTrim = controlEnvironment.addConstant<Scalar>(parameter.outputTrim);

	auto pitchControlOutputSumOne = controlEnvironment.addSum<Scalar, Scalar, Scalar>(
			pitchControlOutputTrim, controlLawSum, true);

	auto pitchControlOutputSumTwo = controlEnvironment.addSum<Scalar, Scalar, Scalar>(pitchControlOutputSumOne,
			pitchControlInputGain, false);

	auto pitchControlOutputConstant = controlEnvironment.addConstant<double>(0);

	std::vector<std::shared_ptr<Constant<double>>> pitchControlOutputVector
	{ pitchControlOutputConstant };

	auto pitchControlOutputDemux = controlEnvironment.addDemux<Scalar, double>(pitchControlOutputSumTwo, pitchControlOutputVector);

	double* pitchControlOutputValue = &controllerOutput.pitchOutput;

	auto pitchOutput = controlEnvironment.addOutput<double, double>(pitchControlOutputConstant,
			pitchControlOutputValue);

	controlEnvironment.evaluate();

	for (int i = 0; i < 100; i++)
	{
		controlEnvironment.evaluate();
	}

	BOOST_CHECK_CLOSE(controllerOutput.pitchOutput, 29.5, 1);
}

BOOST_AUTO_TEST_SUITE_END()
