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
 * Sum.hpp
 *
 *  Created on: May 29, 2019
 *      Author: simonyu
 */

#ifndef UAVAP_FLIGHTCONTROL_CONTROLLER_ADAPTIVECONTROLENVIRONMENT_ADAPTIVECONTROLELEMENTS_SUM_HPP_
#define UAVAP_FLIGHTCONTROL_CONTROLLER_ADAPTIVECONTROLENVIRONMENT_ADAPTIVECONTROLELEMENTS_SUM_HPP_

#include "uavAP/FlightControl/Controller/AdaptiveControlEnvironment/IAdaptiveControlElement.h"

template<typename INPUT_ONE, typename INPUT_TWO, typename OUTPUT>
class Sum: public IAdaptiveControlElement<OUTPUT>
{
public:

	Sum(const AdaptiveElement<INPUT_ONE>& inputOne, const AdaptiveElement<INPUT_TWO>& inputTwo,
			const bool& addition);

	OUTPUT
	getValue() const override;

	void
	setInputOne(const AdaptiveElement<INPUT_ONE>& inputOne);

	void
	setInputTwo(const AdaptiveElement<INPUT_TWO>& inputTwo);

	void
	setAddition(const bool& addition);

private:

	AdaptiveElement<INPUT_ONE> inputOne_;
	AdaptiveElement<INPUT_TWO> inputTwo_;
	bool addition_;
};

template<typename INPUT_ONE, typename INPUT_TWO, typename OUTPUT>
inline
Sum<INPUT_ONE, INPUT_TWO, OUTPUT>::Sum(const AdaptiveElement<INPUT_ONE>& inputOne,
		const AdaptiveElement<INPUT_TWO>& inputTwo, const bool& addition) :
		inputOne_(inputOne), inputTwo_(inputTwo), addition_(addition)
{
}

template<typename INPUT_ONE, typename INPUT_TWO, typename OUTPUT>
inline OUTPUT
Sum<INPUT_ONE, INPUT_TWO, OUTPUT>::getValue() const
{
	if (addition_)
	{
		return inputOne_->getValue() + inputTwo_->getValue();
	}
	else
	{
		return inputOne_->getValue() - inputTwo_->getValue();
	}
}

template<typename INPUT_ONE, typename INPUT_TWO, typename OUTPUT>
inline void
Sum<INPUT_ONE, INPUT_TWO, OUTPUT>::setInputOne(const AdaptiveElement<INPUT_ONE>& inputOne)
{
	inputOne_ = inputOne;
}

template<typename INPUT_ONE, typename INPUT_TWO, typename OUTPUT>
inline void
Sum<INPUT_ONE, INPUT_TWO, OUTPUT>::setInputTwo(const AdaptiveElement<INPUT_TWO>& inputTwo)
{
	inputTwo_ = inputTwo;
}

template<typename INPUT_ONE, typename INPUT_TWO, typename OUTPUT>
inline void
Sum<INPUT_ONE, INPUT_TWO, OUTPUT>::setAddition(const bool& addition)
{
	addition_ = addition;
}

#endif /* UAVAP_FLIGHTCONTROL_CONTROLLER_ADAPTIVECONTROLENVIRONMENT_ADAPTIVECONTROLELEMENTS_SUM_HPP_ */
