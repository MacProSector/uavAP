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
 * StateSpace.hpp
 *
 *  Created on: May 29, 2019
 *      Author: simonyu
 */

#ifndef UAVAP_FLIGHTCONTROL_CONTROLLER_ADAPTIVECONTROLENVIRONMENT_EVALUABLEADAPTIVECONTROLELEMENTS_STATESPACE_HPP_
#define UAVAP_FLIGHTCONTROL_CONTROLLER_ADAPTIVECONTROLENVIRONMENT_EVALUABLEADAPTIVECONTROLELEMENTS_STATESPACE_HPP_

#include "uavAP/FlightControl/Controller/AdaptiveControlEnvironment/IEvaluableAdaptiveControlElement.h"

template<typename STATE, typename INPUT, typename MATRIX_A, typename MATRIX_B, typename MATRIX_C,
		typename MATRIX_D, typename OUTPUT>
class StateSpace: public IEvaluableAdaptiveControlElement<OUTPUT>
{
public:

	StateSpace(const STATE& state, const AdaptiveElement<INPUT>& input, const MATRIX_A& matrixA,
			const MATRIX_B& matrixB, const MATRIX_C& matrixC, const MATRIX_D& matrixD,
			const OUTPUT& output);

	void
	evaluate() override;

	OUTPUT
	getValue() const override;

private:

	STATE state_;
	const AdaptiveElement<INPUT> input_;
	const MATRIX_A& matrixA_;
	const MATRIX_B& matrixB_;
	const MATRIX_C& matrixC_;
	const MATRIX_D& matrixD_;
	OUTPUT output_;
};

template<typename STATE, typename INPUT, typename MATRIX_A, typename MATRIX_B, typename MATRIX_C,
		typename MATRIX_D, typename OUTPUT>
inline
StateSpace<STATE, INPUT, MATRIX_A, MATRIX_B, MATRIX_C, MATRIX_D, OUTPUT>::StateSpace(
		const STATE& state, const AdaptiveElement<INPUT>& input, const MATRIX_A& matrixA,
		const MATRIX_B& matrixB, const MATRIX_C& matrixC, const MATRIX_D& matrixD,
		const OUTPUT& output) :
		state_(state), input_(input), matrixA_(matrixA), matrixB_(matrixB), matrixC_(matrixC), matrixD_(
				matrixD), output_(output)
{
}

template<typename STATE, typename INPUT, typename MATRIX_A, typename MATRIX_B, typename MATRIX_C,
		typename MATRIX_D, typename OUTPUT>
inline void
StateSpace<STATE, INPUT, MATRIX_A, MATRIX_B, MATRIX_C, MATRIX_D, OUTPUT>::evaluate()
{
	INPUT input = input_->getValue();
	output_ = matrixC_ * state_ + matrixD_ * input;
	state_ = matrixA_ * state_ + matrixB_ * input;
}

template<typename STATE, typename INPUT, typename MATRIX_A, typename MATRIX_B, typename MATRIX_C,
		typename MATRIX_D, typename OUTPUT>
inline OUTPUT
StateSpace<STATE, INPUT, MATRIX_A, MATRIX_B, MATRIX_C, MATRIX_D, OUTPUT>::getValue() const
{
	return output_;
}

#endif /* UAVAP_FLIGHTCONTROL_CONTROLLER_ADAPTIVECONTROLENVIRONMENT_EVALUABLEADAPTIVECONTROLELEMENTS_STATESPACE_HPP_ */
