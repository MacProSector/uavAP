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
 * PID.hpp
 *
 *  Created on: Jun 10, 2019
 *      Author: simonyu
 */

#ifndef UAVAP_FLIGHTCONTROL_CONTROLLER_ADAPTIVECONTROLENVIRONMENT_EVALUABLEADAPTIVECONTROLELEMENTS_PID_HPP_
#define UAVAP_FLIGHTCONTROL_CONTROLLER_ADAPTIVECONTROLENVIRONMENT_EVALUABLEADAPTIVECONTROLELEMENTS_PID_HPP_

#include "uavAP/FlightControl/Controller/AdaptiveControlEnvironment/IEvaluableAdaptiveControlElement.h"
#include "uavAP/FlightControl/Controller/PIDController/PIDHandling.h"
#include "uavAP/FlightControl/Controller/PIDController/PIDParameter.h"

template<typename TYPE>
class PID: public IEvaluableAdaptiveControlElement<TYPE>
{
public:

	PID(const AdaptiveElement<TYPE>& input, const AdaptiveElement<TYPE>& target,
			const PIDParameter& parameter, Duration* duration);

	PID(const AdaptiveElement<TYPE>& input, const AdaptiveElement<TYPE>& target,
			const AdaptiveElement<TYPE>& derivative, const PIDParameter& parameter,
			Duration* duration);

	void
	evaluate() override;

	TYPE
	getValue() const override;

	PIDStatus
	getStatus() const;

	void
	setInput(const AdaptiveElement<TYPE>& input);

	void
	setTarget(const AdaptiveElement<TYPE>& target);

	void
	setDerivative(const AdaptiveElement<TYPE>& derivative);

	void
	setParameter(const PIDParameter& parameter);

	void
	setDuration(Duration* duration);

	void
	overrideTarget(const TYPE& target);

	void
	disableOverride();

private:

	void
	calculateProportionalControl();

	void
	calculateIntegralControl();

	void
	calculateDerivativeControl();

	void
	calculateFeedForwardControl();

	AdaptiveElement<TYPE> input_;
	AdaptiveElement<TYPE> target_;
	AdaptiveElement<TYPE> derivative_;

	PIDParameter parameter_;
	Duration* duration_;

	TYPE error_;
	TYPE lastError_;
	TYPE targetValue_;
	TYPE integrator_;
	TYPE output_;
	TYPE overrideTarget_;
	bool override_;
};

template<typename TYPE>
inline
PID<TYPE>::PID(const AdaptiveElement<TYPE>& input, const AdaptiveElement<TYPE>& target,
		const PIDParameter& parameter, Duration* duration) :
		input_(input), target_(target), derivative_(nullptr), parameter_(parameter), duration_(
				duration), error_(), lastError_(), targetValue_(), integrator_(), output_(), overrideTarget_(), override_(
				false)
{
}

template<typename TYPE>
inline
PID<TYPE>::PID(const AdaptiveElement<TYPE>& input, const AdaptiveElement<TYPE>& target,
		const AdaptiveElement<TYPE>& derivative, const PIDParameter& parameter, Duration* duration) :
		input_(input), target_(target), derivative_(derivative), parameter_(parameter), duration_(
				duration), error_(), lastError_(), targetValue_(), integrator_(), output_(), overrideTarget_(), override_(
				false)
{
}

template<typename TYPE>
inline void
PID<TYPE>::evaluate()
{
	if (!input_ || !target_)
	{
		return;
	}

	output_ = 0;
	targetValue_ = override_ ? overrideTarget_ : target_->getValue();
	error_ = targetValue_ - input_->getValue();

	calculateProportionalControl();
	calculateIntegralControl();
	calculateDerivativeControl();
	calculateFeedForwardControl();

	lastError_ = error_;
}

template<typename TYPE>
inline TYPE
PID<TYPE>::getValue() const
{
	return output_;
}

template<typename TYPE>
inline PIDStatus
PID<TYPE>::getStatus() const
{
	PIDStatus status;

	status.target = targetValue_;
	status.value = input_->getValue();

	return status;
}

template<typename TYPE>
inline void
PID<TYPE>::setInput(const AdaptiveElement<TYPE>& input)
{
	input_ = input;
}

template<typename TYPE>
inline void
PID<TYPE>::setTarget(const AdaptiveElement<TYPE>& target)
{
	target_ = target;
}

template<typename TYPE>
inline void
PID<TYPE>::setDerivative(const AdaptiveElement<TYPE>& derivative)
{
	derivative_ = derivative;
}

template<typename TYPE>
inline void
PID<TYPE>::setParameter(const PIDParameter& parameter)
{
	parameter_ = parameter;
}

template<typename TYPE>
inline void
PID<TYPE>::setDuration(Duration* duration)
{
	duration_ = duration;
}

template<typename TYPE>
inline void
PID<TYPE>::overrideTarget(const TYPE& target)
{
	overrideTarget_ = target;
	override_ = true;
}

template<typename TYPE>
inline void
PID<TYPE>::disableOverride()
{
	override_ = false;
}

template<typename TYPE>
inline void
PID<TYPE>::calculateProportionalControl()
{
	if (parameter_.kp == 0)
	{
		return;
	}

	output_ += parameter_.kp * error_;
}

template<typename TYPE>
inline void
PID<TYPE>::calculateIntegralControl()
{
	if (parameter_.ki == 0 || !duration_)
	{
		return;
	}

	TYPE durationSecond = duration_->total_microseconds() / 1e6;
	integrator_ += error_ * durationSecond;

	if (integrator_ > 0)
	{
		integrator_ = std::min(integrator_, parameter_.imax);
	}
	else
	{
		integrator_ = std::max(integrator_, -parameter_.imax);
	}

	output_ += parameter_.ki * integrator_;
}

template<typename TYPE>
inline void
PID<TYPE>::calculateDerivativeControl()
{
	if (parameter_.kd == 0)
	{
		return;
	}

	if (derivative_)
	{
		output_ -= parameter_.kd * derivative_->getValue();
	}
	else if (duration_ && duration_->total_microseconds() > 0)
	{
		TYPE durationSecond = duration_->total_microseconds() / 1e6;
		TYPE derivative = (error_ - lastError_) / durationSecond;
		output_ -= parameter_.kd * derivative;
	}
}

template<typename TYPE>
inline void
PID<TYPE>::calculateFeedForwardControl()
{
	if (parameter_.ff == 0)
	{
		return;
	}

	output_ += parameter_.ff * targetValue_;
}

#endif /* UAVAP_FLIGHTCONTROL_CONTROLLER_ADAPTIVECONTROLENVIRONMENT_EVALUABLEADAPTIVECONTROLELEMENTS_PID_HPP_ */
