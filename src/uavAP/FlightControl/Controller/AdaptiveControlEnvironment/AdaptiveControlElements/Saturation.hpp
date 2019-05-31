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
 * Saturation.hpp
 *
 *  Created on: May 30, 2019
 *      Author: simonyu
 */

#ifndef UAVAP_FLIGHTCONTROL_CONTROLLER_ADAPTIVECONTROLENVIRONMENT_ADAPTIVECONTROLELEMENTS_SATURATION_HPP_
#define UAVAP_FLIGHTCONTROL_CONTROLLER_ADAPTIVECONTROLENVIRONMENT_ADAPTIVECONTROLELEMENTS_SATURATION_HPP_

#include "uavAP/FlightControl/Controller/AdaptiveControlEnvironment/IAdaptiveControlElement.h"

template<typename TYPE>
class Saturation: public IAdaptiveControlElement<TYPE>
{
public:

	Saturation(const AdaptiveElement<TYPE>& input, const TYPE& min, const TYPE& max);

	Saturation(const AdaptiveElement<TYPE>& input, const TYPE& min, const TYPE& max,
			const TYPE& hardMin, const TYPE& hardMax);

	TYPE
	getValue() const override;

	void
	setInput(const AdaptiveElement<TYPE>& input);

	void
	setHardSaturationValue(const TYPE& hardMinMax);

	void
	setHardSaturationValue(const TYPE& hardMin, const TYPE& hardMax);

	void
	setSaturationValue(const TYPE& minMax);

	void
	setSaturationValue(const TYPE& min, const TYPE& max);

	void
	overrideSaturationValue(const TYPE& overrideMinMax);

	void
	overrideSaturationValue(const TYPE& overrideMin, const TYPE& overrideMax);

	void
	disableOverride();

private:

	AdaptiveElement<TYPE> input_;
	double min_;
	double max_;
	double hardMin_;
	double hardMax_;
	double overrideMin_;
	double overrideMax_;
	bool override_;
};

template<typename TYPE>
inline
Saturation<TYPE>::Saturation(const AdaptiveElement<TYPE>& input, const TYPE& min, const TYPE& max) :
		input_(input), min_(0), max_(0), hardMin_(0), hardMax_(0), overrideMin_(0), overrideMax_(0), override_(
				false)
{
	setHardSaturationValue(min, max);
	setSaturationValue(min, max);
}

template<typename TYPE>
inline
Saturation<TYPE>::Saturation(const AdaptiveElement<TYPE>& input, const TYPE& min, const TYPE& max,
		const TYPE& hardMin, const TYPE& hardMax) :
		input_(input), min_(0), max_(0), hardMin_(0), hardMax_(0), overrideMin_(0), overrideMax_(0), override_(
				false)
{
	setHardSaturationValue(hardMin, hardMax);
	setSaturationValue(min, max);
}

template<typename TYPE>
inline TYPE
Saturation<TYPE>::getValue() const
{
	double input = input_->getValue();
	double saturation = 0;

	if (override_)
	{
		saturation = input > overrideMax_ ? overrideMax_ :
						input < overrideMin_ ? overrideMin_ : input;
	}
	else
	{
		saturation = input > max_ ? max_ : input < min_ ? min_ : input;
	}

	return saturation;
}

template<typename TYPE>
inline void
Saturation<TYPE>::setInput(const AdaptiveElement<TYPE>& input)
{
	input_ = input;
}

template<typename TYPE>
inline void
Saturation<TYPE>::setHardSaturationValue(const TYPE& hardMinMax)
{
	hardMax_ = hardMinMax;
	hardMin_ = -hardMinMax;
}

template<typename TYPE>
inline void
Saturation<TYPE>::setHardSaturationValue(const TYPE& hardMin, const TYPE& hardMax)
{
	hardMax_ = hardMax;
	hardMin_ = hardMin;
}

template<typename TYPE>
inline void
Saturation<TYPE>::setSaturationValue(const TYPE& minMax)
{
	max_ = minMax > hardMax_ ? hardMax_ : minMax;
	min_ = -minMax < hardMin_ ? hardMin_ : -minMax;
}

template<typename TYPE>
inline void
Saturation<TYPE>::setSaturationValue(const TYPE& min, const TYPE& max)
{
	max_ = max > hardMax_ ? hardMax_ : max;
	min_ = min < hardMin_ ? hardMin_ : min;
}

template<typename TYPE>
inline void
Saturation<TYPE>::overrideSaturationValue(const TYPE& overrideMinMax)
{
	overrideMax_ = overrideMinMax > hardMax_ ? hardMax_ : overrideMinMax;
	overrideMin_ = -overrideMinMax < hardMin_ ? hardMin_ : -overrideMinMax;
	override_ = true;
}

template<typename TYPE>
inline void
Saturation<TYPE>::overrideSaturationValue(const TYPE& overrideMin, const TYPE& overrideMax)
{
	overrideMax_ = overrideMax > hardMax_ ? hardMax_ : overrideMax;
	overrideMin_ = overrideMin < hardMin_ ? hardMin_ : overrideMin;
	override_ = true;
}

template<typename TYPE>
inline void
Saturation<TYPE>::disableOverride()
{
	override_ = false;
}

#endif /* UAVAP_FLIGHTCONTROL_CONTROLLER_ADAPTIVECONTROLENVIRONMENT_ADAPTIVECONTROLELEMENTS_SATURATION_HPP_ */
