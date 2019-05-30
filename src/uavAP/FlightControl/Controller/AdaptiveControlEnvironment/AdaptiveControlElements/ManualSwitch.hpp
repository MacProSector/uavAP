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
 * ManualSwitch.hpp
 *
 *  Created on: May 30, 2019
 *      Author: simonyu
 */

#ifndef UAVAP_FLIGHTCONTROL_CONTROLLER_ADAPTIVECONTROLENVIRONMENT_ADAPTIVECONTROLELEMENTS_MANUALSWITCH_HPP_
#define UAVAP_FLIGHTCONTROL_CONTROLLER_ADAPTIVECONTROLENVIRONMENT_ADAPTIVECONTROLELEMENTS_MANUALSWITCH_HPP_

#include "uavAP/FlightControl/Controller/AdaptiveControlEnvironment/IAdaptiveControlElement.h"

template<typename TYPE>
class ManualSwitch: public IAdaptiveControlElement<TYPE>
{
public:

	ManualSwitch(const AdaptiveElement<TYPE>& inputTrue, const AdaptiveElement<TYPE>& inputFalse,
			const bool& selection);

	TYPE
	getValue() const override;

	void
	setInputTrue(const AdaptiveElement<TYPE>& inputTrue);

	void
	setInputFalse(const AdaptiveElement<TYPE>& inputFalse);

	void
	setSelection(const bool& selection);

private:

	AdaptiveElement<TYPE> inputTrue_;
	AdaptiveElement<TYPE> inputFalse_;
	bool selection_;
};

template<typename TYPE>
inline
ManualSwitch<TYPE>::ManualSwitch(const AdaptiveElement<TYPE>& inputTrue,
		const AdaptiveElement<TYPE>& inputFalse, const bool& selection) :
		inputTrue_(inputTrue), inputFalse_(inputFalse), selection_(selection)
{
}

template<typename TYPE>
inline TYPE
ManualSwitch<TYPE>::getValue() const
{
	if (selection_)
	{
		return inputTrue_->getValue();
	}
	else
	{
		return inputFalse_->getValue();
	}
}

template<typename TYPE>
inline void
ManualSwitch<TYPE>::setInputTrue(const AdaptiveElement<TYPE>& inputTrue)
{
	inputTrue_ = inputTrue;
}

template<typename TYPE>
inline void
ManualSwitch<TYPE>::setInputFalse(const AdaptiveElement<TYPE>& inputFalse)
{
	inputFalse_ = inputFalse;
}

template<typename TYPE>
inline void
ManualSwitch<TYPE>::setSelection(const bool& selection)
{
	selection_ = selection;
}

#endif /* UAVAP_FLIGHTCONTROL_CONTROLLER_ADAPTIVECONTROLENVIRONMENT_ADAPTIVECONTROLELEMENTS_MANUALSWITCH_HPP_ */
