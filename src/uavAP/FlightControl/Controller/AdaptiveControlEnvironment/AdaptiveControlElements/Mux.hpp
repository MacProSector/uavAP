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
 * Mux.hpp
 *
 *  Created on: Jun 12, 2019
 *      Author: simonyu
 */

#ifndef UAVAP_FLIGHTCONTROL_CONTROLLER_ADAPTIVECONTROLENVIRONMENT_ADAPTIVECONTROLELEMENTS_MUX_HPP_
#define UAVAP_FLIGHTCONTROL_CONTROLLER_ADAPTIVECONTROLENVIRONMENT_ADAPTIVECONTROLELEMENTS_MUX_HPP_

#include <vector>

#include "uavAP/FlightControl/Controller/AdaptiveControlEnvironment/IAdaptiveControlElement.h"

template<typename INPUT, typename OUTPUT>
class Mux: public IAdaptiveControlElement<OUTPUT>
{
public:

	Mux(const std::vector<AdaptiveElement<INPUT>>& input);

	OUTPUT
	getValue() const override;

	void
	setInput(const std::vector<AdaptiveElement<INPUT>>& input);

private:

	std::vector<AdaptiveElement<INPUT>> input_;
};

template<typename INPUT, typename OUTPUT>
inline
Mux<INPUT, OUTPUT>::Mux(const std::vector<AdaptiveElement<INPUT> >& input) :
		input_(input)
{
}

template<typename INPUT, typename OUTPUT>
inline OUTPUT
Mux<INPUT, OUTPUT>::getValue() const
{
	OUTPUT output;
	int i = 0;

	for (const auto& it : input_)
	{
		output[i ++] = it->getValue();
	}

	return output;
}

template<typename INPUT, typename OUTPUT>
inline void
Mux<INPUT, OUTPUT>::setInput(const std::vector<AdaptiveElement<INPUT> >& input)
{
	input_ = input;
}

#endif /* UAVAP_FLIGHTCONTROL_CONTROLLER_ADAPTIVECONTROLENVIRONMENT_ADAPTIVECONTROLELEMENTS_MUX_HPP_ */
