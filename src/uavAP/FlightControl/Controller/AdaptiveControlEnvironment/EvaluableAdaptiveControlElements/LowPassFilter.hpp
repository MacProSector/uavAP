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
 * LowPassFilter.hpp
 *
 *  Created on: May 31, 2019
 *      Author: simonyu
 */

#ifndef UAVAP_FLIGHTCONTROL_CONTROLLER_ADAPTIVECONTROLENVIRONMENT_EVALUABLEADAPTIVECONTROLELEMENTS_LOWPASSFILTER_HPP_
#define UAVAP_FLIGHTCONTROL_CONTROLLER_ADAPTIVECONTROLENVIRONMENT_EVALUABLEADAPTIVECONTROLELEMENTS_LOWPASSFILTER_HPP_

#include "uavAP/Core/DataPresentation/DataFilter/LowPassDataFilter/LowPassDataFilter.h"
#include "uavAP/FlightControl/Controller/AdaptiveControlEnvironment/IEvaluableAdaptiveControlElement.h"

template<typename TYPE>
class LowPassFilter: public IEvaluableAdaptiveControlElement<TYPE>
{
public:

	LowPassFilter(const AdaptiveElement<TYPE>& input, const TYPE& alpha);

	void
	evaluate() override;

	TYPE
	getValue() const override;

	void
	setInput(const AdaptiveElement<TYPE>& input);

	void
	setInitialValue(const TYPE& initialValue);

	void
	setAlpha(const TYPE& alpha);

private:

	AdaptiveElement<TYPE> input_;
	LowPassDataFilter filter_;
};

template<typename TYPE>
inline
LowPassFilter<TYPE>::LowPassFilter(const AdaptiveElement<TYPE>& input, const TYPE& alpha) :
		input_(input), filter_(input_->getValue(), alpha)
{
}

template<typename TYPE>
inline void
LowPassFilter<TYPE>::evaluate()
{
	filter_.filterData(input_->getValue());
}

template<typename TYPE>
inline TYPE
LowPassFilter<TYPE>::getValue() const
{
	return filter_.getFilteredData();
}

template<typename TYPE>
inline void
LowPassFilter<TYPE>::setInput(const AdaptiveElement<TYPE>& input)
{
	input_ = input;
}

template<typename TYPE>
inline void
LowPassFilter<TYPE>::setInitialValue(const TYPE& initialValue)
{
	filter_.initialize(initialValue);
}

template<typename TYPE>
inline void
LowPassFilter<TYPE>::setAlpha(const TYPE& alpha)
{
	filter_.tune(alpha);
}

#endif /* UAVAP_FLIGHTCONTROL_CONTROLLER_ADAPTIVECONTROLENVIRONMENT_EVALUABLEADAPTIVECONTROLELEMENTS_LOWPASSFILTER_HPP_ */
