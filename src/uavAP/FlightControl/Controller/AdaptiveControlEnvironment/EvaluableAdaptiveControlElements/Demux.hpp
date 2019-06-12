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
 * Demux.hpp
 *
 *  Created on: Jun 12, 2019
 *      Author: simonyu
 */

#ifndef UAVAP_FLIGHTCONTROL_CONTROLLER_ADAPTIVECONTROLENVIRONMENT_EVALUABLEADAPTIVECONTROLELEMENTS_DEMUX_HPP_
#define UAVAP_FLIGHTCONTROL_CONTROLLER_ADAPTIVECONTROLENVIRONMENT_EVALUABLEADAPTIVECONTROLELEMENTS_DEMUX_HPP_

#include <vector>

#include "uavAP/FlightControl/Controller/AdaptiveControlEnvironment/IEvaluableAdaptiveControlElement.h"

using std::vector;

template<typename INPUT, typename OUTPUT>
class Demux: public IEvaluableAdaptiveControlElement<OUTPUT>
{
public:

	Demux(const AdaptiveElement<INPUT>& input,
			const std::vector<std::shared_ptr<Constant<OUTPUT>>>& output);

	void
	evaluate() override;

	OUTPUT
	getValue() const override;

	void
	setInput(const AdaptiveElement<INPUT>& input);

	void
	setOutput(const std::vector<std::shared_ptr<Constant<OUTPUT>>>& output);

private:

	AdaptiveElement<INPUT> input_;
	std::vector<std::shared_ptr<Constant<OUTPUT>>> output_;
};

template<typename INPUT, typename OUTPUT>
inline
Demux<INPUT, OUTPUT>::Demux(const AdaptiveElement<INPUT>& input,
		const std::vector<std::shared_ptr<Constant<OUTPUT>>>& output) :
		input_(input), output_(output)
{
}

template<typename INPUT, typename OUTPUT>
inline void
Demux<INPUT, OUTPUT>::evaluate()
{
	INPUT input = input_->getValue();

	for (unsigned i = 0; i < input.size(); i ++)
	{
		output_[i]->setConstant(input[i]);
	}
}

template<typename INPUT, typename OUTPUT>
inline OUTPUT
Demux<INPUT, OUTPUT>::getValue() const
{
	return OUTPUT();
}

template<typename INPUT, typename OUTPUT>
inline void
Demux<INPUT, OUTPUT>::setInput(const AdaptiveElement<INPUT>& input)
{
	input_ = input;
}

template<typename INPUT, typename OUTPUT>
inline void
Demux<INPUT, OUTPUT>::setOutput(const std::vector<std::shared_ptr<Constant<OUTPUT>>>& output)
{
	output_ = output;
}

#endif /* UAVAP_FLIGHTCONTROL_CONTROLLER_ADAPTIVECONTROLENVIRONMENT_EVALUABLEADAPTIVECONTROLELEMENTS_DEMUX_HPP_ */
