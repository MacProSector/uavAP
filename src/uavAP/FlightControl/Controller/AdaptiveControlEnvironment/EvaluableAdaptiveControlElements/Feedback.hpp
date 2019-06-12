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
 * Feedback.hpp
 *
 *  Created on: Jun 12, 2019
 *      Author: simonyu
 */

#ifndef UAVAP_FLIGHTCONTROL_CONTROLLER_ADAPTIVECONTROLENVIRONMENT_EVALUABLEADAPTIVECONTROLELEMENTS_FEEDBACK_HPP_
#define UAVAP_FLIGHTCONTROL_CONTROLLER_ADAPTIVECONTROLENVIRONMENT_EVALUABLEADAPTIVECONTROLELEMENTS_FEEDBACK_HPP_

#include "uavAP/Core/Logging/APLogger.h"
#include "uavAP/FlightControl/Controller/AdaptiveControlEnvironment/IEvaluableAdaptiveControlElement.h"

template<typename TYPE>
class Feedback: public IEvaluableAdaptiveControlElement<TYPE>
{
public:

	Feedback();

	Feedback(const AdaptiveElement<TYPE>& input);

	void
	evaluate() override;

	TYPE
	getValue() const override;

	void
	setInput(const AdaptiveElement<TYPE>& input);

	void
	setOutput(const TYPE& output);

private:

	AdaptiveElement<TYPE> input_;
	TYPE output_;
};

template<typename TYPE>
inline
Feedback<TYPE>::Feedback() :
		input_(nullptr), output_()
{
}

template<typename TYPE>
inline
Feedback<TYPE>::Feedback(const AdaptiveElement<TYPE>& input) :
		input_(input), output_()
{
}

template<typename TYPE>
inline void
Feedback<TYPE>::evaluate()
{
	if (!input_)
	{
		APLOG_WARN << "Feedback: Incomplete Feedback";
		return;
	}

	output_ = input_->getValue();
}

template<typename TYPE>
inline TYPE
Feedback<TYPE>::getValue() const
{
	return output_;
}

template<typename TYPE>
inline void
Feedback<TYPE>::setInput(const AdaptiveElement<TYPE>& input)
{
	input_ = input;
}

template<typename TYPE>
inline void
Feedback<TYPE>::setOutput(const TYPE& output)
{
	output_ = output;
}

#endif /* UAVAP_FLIGHTCONTROL_CONTROLLER_ADAPTIVECONTROLENVIRONMENT_EVALUABLEADAPTIVECONTROLELEMENTS_FEEDBACK_HPP_ */
