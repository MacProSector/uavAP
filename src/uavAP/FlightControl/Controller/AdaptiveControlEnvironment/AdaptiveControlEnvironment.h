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
 * AdaptiveControlEnvironment.h
 *
 *  Created on: May 29, 2019
 *      Author: simonyu
 */

#ifndef UAVAP_FLIGHTCONTROL_CONTROLLER_ADAPTIVECONTROLENVIRONMENT_ADAPTIVECONTROLENVIRONMENT_H_
#define UAVAP_FLIGHTCONTROL_CONTROLLER_ADAPTIVECONTROLENVIRONMENT_ADAPTIVECONTROLENVIRONMENT_H_

#include <vector>

#include "uavAP/Core/Time.h"
#include "uavAP/FlightControl/Controller/AdaptiveControlEnvironment/AdaptiveControlElements/Constant.hpp"
#include "uavAP/FlightControl/Controller/AdaptiveControlEnvironment/AdaptiveControlElements/Gain.hpp"
#include "uavAP/FlightControl/Controller/AdaptiveControlEnvironment/AdaptiveControlElements/Input.hpp"
#include "uavAP/FlightControl/Controller/AdaptiveControlEnvironment/AdaptiveControlElements/ManualSwitch.hpp"
#include "uavAP/FlightControl/Controller/AdaptiveControlEnvironment/AdaptiveControlElements/Mux.hpp"
#include "uavAP/FlightControl/Controller/AdaptiveControlEnvironment/AdaptiveControlElements/Saturation.hpp"
#include "uavAP/FlightControl/Controller/AdaptiveControlEnvironment/AdaptiveControlElements/Sum.hpp"
#include "uavAP/FlightControl/Controller/AdaptiveControlEnvironment/EvaluableAdaptiveControlElements/Demux.hpp"
#include "uavAP/FlightControl/Controller/AdaptiveControlEnvironment/EvaluableAdaptiveControlElements/LowPassFilter.hpp"
#include "uavAP/FlightControl/Controller/AdaptiveControlEnvironment/EvaluableAdaptiveControlElements/Output.hpp"
#include "uavAP/FlightControl/Controller/AdaptiveControlEnvironment/EvaluableAdaptiveControlElements/PID.hpp"
#include "uavAP/FlightControl/Controller/AdaptiveControlEnvironment/EvaluableAdaptiveControlElements/StateSpace.hpp"

class AdaptiveControlEnvironment
{
public:

	AdaptiveControlEnvironment();

	AdaptiveControlEnvironment(const TimePoint* timestamp);

	void
	evaluate();

	template<typename TYPE>
	std::shared_ptr<Constant<TYPE>>
	addConstant(const TYPE& constant);

	template<typename INPUT, typename GAIN, typename OUTPUT>
	std::shared_ptr<Gain<INPUT, GAIN, OUTPUT>>
	addGain(const AdaptiveElement<INPUT>& input, const GAIN& gain);

	template<typename TYPE>
	std::shared_ptr<Input<TYPE>>
	addInput(TYPE* input);

	template<typename TYPE>
	std::shared_ptr<ManualSwitch<TYPE>>
	addManualSwitch(const AdaptiveElement<TYPE>& inputTrue, const AdaptiveElement<TYPE>& inputFalse,
			const bool& selection);

	template<typename INPUT, typename OUTPUT>
	std::shared_ptr<Mux<INPUT, OUTPUT>>
	addMux(const std::vector<AdaptiveElement<INPUT>>& input);

	template<typename TYPE>
	std::shared_ptr<Saturation<TYPE>>
	addSaturation(const AdaptiveElement<TYPE>& input, const TYPE& min, const TYPE& max);

	template<typename TYPE>
	std::shared_ptr<Saturation<TYPE>>
	addSaturation(const AdaptiveElement<TYPE>& input, const TYPE& min, const TYPE& max,
			const TYPE& hardMin, const TYPE& hardMax);

	template<typename INPUT_ONE, typename INPUT_TWO, typename OUTPUT>
	std::shared_ptr<Sum<INPUT_ONE, INPUT_TWO, OUTPUT>>
	addSum(const AdaptiveElement<INPUT_ONE>& inputOne, const AdaptiveElement<INPUT_TWO>& inputTwo,
			const bool& addition);

	template<typename INPUT, typename OUTPUT>
	std::shared_ptr<Demux<INPUT, OUTPUT>>
	addDemux(const AdaptiveElement<INPUT>& input,
			const std::vector<std::shared_ptr<Constant<OUTPUT>>>& output);

	template<typename TYPE>
	std::shared_ptr<LowPassFilter<TYPE>>
	addLowPassFilter(const AdaptiveElement<TYPE>& input, const TYPE& alpha);

	template<typename INPUT, typename OUTPUT>
	std::shared_ptr<Output<INPUT, OUTPUT>>
	addOutput(const AdaptiveElement<INPUT>& input, OUTPUT* output);

	template<typename TYPE>
	std::shared_ptr<PID<TYPE>>
	addPID(const AdaptiveElement<TYPE>& input, const AdaptiveElement<TYPE>& target,
			const PIDParameter& parameter);

	template<typename TYPE>
	std::shared_ptr<PID<TYPE>>
	addPID(const AdaptiveElement<TYPE>& input, const AdaptiveElement<TYPE>& target,
			const AdaptiveElement<TYPE>& derivative, const PIDParameter& parameter);

	template<typename STATE, typename INPUT, typename MATRIX_A, typename MATRIX_B,
			typename MATRIX_C, typename MATRIX_D, typename OUTPUT>
	std::shared_ptr<StateSpace<STATE, INPUT, MATRIX_A, MATRIX_B, MATRIX_C, MATRIX_D, OUTPUT>>
	addStateSpace(const STATE& state, const AdaptiveElement<INPUT>& input, const MATRIX_A& matrixA,
			const MATRIX_B& matrixB, const MATRIX_C& matrixC, const MATRIX_D& matrixD,
			const OUTPUT& output);

private:

	const TimePoint* timestamp_;
	TimePoint lastTimestamp_;
	Duration duration_;
	std::vector<EvaluableAdaptiveElement> evaluableAdaptiveElements_;
};

template<typename TYPE>
inline std::shared_ptr<Constant<TYPE>>
AdaptiveControlEnvironment::addConstant(const TYPE& constant)
{
	return std::make_shared<Constant<TYPE>>(constant);
}

template<typename INPUT, typename GAIN, typename OUTPUT>
inline std::shared_ptr<Gain<INPUT, GAIN, OUTPUT>>
AdaptiveControlEnvironment::addGain(const AdaptiveElement<INPUT>& input, const GAIN& gain)
{
	return std::make_shared<Gain<INPUT, GAIN, OUTPUT>>(input, gain);
}

template<typename TYPE>
inline std::shared_ptr<Input<TYPE>>
AdaptiveControlEnvironment::addInput(TYPE* input)
{
	return std::make_shared<Input<TYPE>>(input);
}

template<typename TYPE>
inline std::shared_ptr<ManualSwitch<TYPE>>
AdaptiveControlEnvironment::addManualSwitch(const AdaptiveElement<TYPE>& inputTrue,
		const AdaptiveElement<TYPE>& inputFalse, const bool& selection)
{
	return std::make_shared<ManualSwitch<TYPE>>(inputTrue, inputFalse, selection);
}

template<typename INPUT, typename OUTPUT>
inline std::shared_ptr<Mux<INPUT, OUTPUT>>
AdaptiveControlEnvironment::addMux(const std::vector<AdaptiveElement<INPUT>>& input)
{
	return std::make_shared<Mux<INPUT, OUTPUT>>(input);
}

template<typename TYPE>
inline std::shared_ptr<Saturation<TYPE>>
AdaptiveControlEnvironment::addSaturation(const AdaptiveElement<TYPE>& input, const TYPE& min,
		const TYPE& max)
{
	return std::make_shared<Saturation<TYPE>>(input, min, max);
}

template<typename TYPE>
inline std::shared_ptr<Saturation<TYPE>>
AdaptiveControlEnvironment::addSaturation(const AdaptiveElement<TYPE>& input, const TYPE& min,
		const TYPE& max, const TYPE& hardMin, const TYPE& hardMax)
{
	return std::make_shared<Saturation<TYPE>>(input, min, max, hardMin, hardMax);
}

template<typename INPUT_ONE, typename INPUT_TWO, typename OUTPUT>
inline std::shared_ptr<Sum<INPUT_ONE, INPUT_TWO, OUTPUT>>
AdaptiveControlEnvironment::addSum(const AdaptiveElement<INPUT_ONE>& inputOne,
		const AdaptiveElement<INPUT_TWO>& inputTwo, const bool& addition)
{
	return std::make_shared<Sum<INPUT_ONE, INPUT_TWO, OUTPUT>>(inputOne, inputTwo, addition);
}

template<typename INPUT, typename OUTPUT>
inline std::shared_ptr<Demux<INPUT, OUTPUT>>
AdaptiveControlEnvironment::addDemux(const AdaptiveElement<INPUT>& input,
		const std::vector<std::shared_ptr<Constant<OUTPUT>>>& output)
{
	auto demux = std::make_shared<Demux<INPUT, OUTPUT>>(input, output);
	auto demuxEvaluation = std::bind(&Demux<INPUT, OUTPUT>::evaluate, demux);
	evaluableAdaptiveElements_.push_back(demuxEvaluation);

	return demux;
}

template<typename TYPE>
inline std::shared_ptr<LowPassFilter<TYPE>>
AdaptiveControlEnvironment::addLowPassFilter(const AdaptiveElement<TYPE>& input, const TYPE& alpha)
{
	auto lowPassFilter = std::make_shared<LowPassFilter<TYPE>>(input, alpha);
	auto lowPassFilterEvaluation = std::bind(&LowPassFilter<TYPE>::evaluate, lowPassFilter);
	evaluableAdaptiveElements_.push_back(lowPassFilterEvaluation);

	return lowPassFilter;
}

template<typename INPUT, typename OUTPUT>
inline std::shared_ptr<Output<INPUT, OUTPUT>>
AdaptiveControlEnvironment::addOutput(const AdaptiveElement<INPUT>& input, OUTPUT* output)
{
	auto _output = std::make_shared<Output<INPUT, OUTPUT>>(input, output);
	auto outputEvaluation = std::bind(&Output<INPUT, OUTPUT>::evaluate, _output);
	evaluableAdaptiveElements_.push_back(outputEvaluation);

	return _output;
}

template<typename TYPE>
inline std::shared_ptr<PID<TYPE>>
AdaptiveControlEnvironment::addPID(const AdaptiveElement<TYPE>& input,
		const AdaptiveElement<TYPE>& target, const PIDParameter& parameter)
{
	auto pid = std::make_shared<PID<TYPE>>(input, target, parameter, &duration_);
	auto pidEvaluation = std::bind(&PID<TYPE>::evaluate, pid);
	evaluableAdaptiveElements_.push_back(pidEvaluation);

	return pid;
}

template<typename TYPE>
inline std::shared_ptr<PID<TYPE>>
AdaptiveControlEnvironment::addPID(const AdaptiveElement<TYPE>& input,
		const AdaptiveElement<TYPE>& target, const AdaptiveElement<TYPE>& derivative,
		const PIDParameter& parameter)
{
	auto pid = std::make_shared<PID<TYPE>>(input, target, derivative, parameter, &duration_);
	auto pidEvaluation = std::bind(&PID<TYPE>::evaluate, pid);
	evaluableAdaptiveElements_.push_back(pidEvaluation);

	return pid;
}

template<typename STATE, typename INPUT, typename MATRIX_A, typename MATRIX_B, typename MATRIX_C,
		typename MATRIX_D, typename OUTPUT>
inline std::shared_ptr<StateSpace<STATE, INPUT, MATRIX_A, MATRIX_B, MATRIX_C, MATRIX_D, OUTPUT>>
AdaptiveControlEnvironment::addStateSpace(const STATE& state, const AdaptiveElement<INPUT>& input,
		const MATRIX_A& matrixA, const MATRIX_B& matrixB, const MATRIX_C& matrixC,
		const MATRIX_D& matrixD, const OUTPUT& output)
{
	auto stateSpace = std::make_shared<
			StateSpace<STATE, INPUT, MATRIX_A, MATRIX_B, MATRIX_C, MATRIX_D, OUTPUT>>(state, input,
			matrixA, matrixB, matrixC, matrixD, output);
	auto stateSpaceEvaluation = std::bind(
			&StateSpace<STATE, INPUT, MATRIX_A, MATRIX_B, MATRIX_C, MATRIX_D, OUTPUT>::evaluate,
			stateSpace);
	evaluableAdaptiveElements_.push_back(stateSpaceEvaluation);

	return stateSpace;
}

#endif /* UAVAP_FLIGHTCONTROL_CONTROLLER_ADAPTIVECONTROLENVIRONMENT_ADAPTIVECONTROLENVIRONMENT_H_ */
