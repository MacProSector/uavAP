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
 * Gain.h
 *
 *  Created on: May 29, 2019
 *      Author: simonyu
 */

#ifndef UAVAP_FLIGHTCONTROL_CONTROLLER_ADAPTIVECONTROLENVIRONMENT_ADAPTIVECONTROLELEMENTS_GAIN_HPP_
#define UAVAP_FLIGHTCONTROL_CONTROLLER_ADAPTIVECONTROLENVIRONMENT_ADAPTIVECONTROLELEMENTS_GAIN_HPP_

#include "uavAP/FlightControl/Controller/AdaptiveControlEnvironment/IAdaptiveControlElement.h"

template<typename INPUT, typename GAIN, typename OUTPUT>
class Gain: public IAdaptiveControlElement<OUTPUT>
{
public:

	Gain(const AdaptiveElement<INPUT>& input, const GAIN& gain);

	OUTPUT
	getValue() const override;

private:

	const AdaptiveElement<INPUT> input_;
	const GAIN& gain_;
};

template<typename INPUT, typename GAIN, typename OUTPUT>
inline
Gain<INPUT, GAIN, OUTPUT>::Gain(const AdaptiveElement<INPUT>& input, const GAIN& gain) :
		input_(input), gain_(gain)
{
}

template<typename INPUT, typename GAIN, typename OUTPUT>
inline OUTPUT
Gain<INPUT, GAIN, OUTPUT>::getValue() const
{
	return gain_ * input_->getValue();
}

#endif /* UAVAP_FLIGHTCONTROL_CONTROLLER_ADAPTIVECONTROLENVIRONMENT_ADAPTIVECONTROLELEMENTS_GAIN_HPP_ */
