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
 * Output.hpp
 *
 *  Created on: May 31, 2019
 *      Author: simonyu
 */

#ifndef UAVAP_FLIGHTCONTROL_CONTROLLER_ADAPTIVECONTROLENVIRONMENT_EVALUABLEADAPTIVECONTROLELEMENTS_OUTPUT_HPP_
#define UAVAP_FLIGHTCONTROL_CONTROLLER_ADAPTIVECONTROLENVIRONMENT_EVALUABLEADAPTIVECONTROLELEMENTS_OUTPUT_HPP_

#include "uavAP/Core/Time.h"
#include "uavAP/FlightControl/Controller/ControllerOutput.h"
#include "uavAP/FlightControl/Controller/AdaptiveControlEnvironment/IEvaluableAdaptiveControlElement.h"

template<typename INPUT, typename OUTPUT>
class Output: public IEvaluableAdaptiveControlElement<OUTPUT>
{
public:

	Output(const AdaptiveElement<INPUT>& input, OUTPUT* output);

	void
	evaluate() override;

	OUTPUT
	getValue() const override;

	void
	setInput(const AdaptiveElement<INPUT>& input);

	void
	setWaveform(const Waveforms& waveform);

	void
	setPeriod(const OUTPUT& period);

	void
	setPhase(const OUTPUT& phase);

	void
	setOutput(OUTPUT* output);

	void
	overrideOutput(const OUTPUT& amplitude);

	void
	disableOverride();

private:

	OUTPUT
	getWaveformOutput();

	AdaptiveElement<INPUT> input_;
	TimePoint timestamp_;
	Waveforms waveform_;
	OUTPUT period_;
	OUTPUT phase_;
	OUTPUT* output_;
	OUTPUT amplitude_;
	bool override_;
};

template<typename INPUT, typename OUTPUT>
inline
Output<INPUT, OUTPUT>::Output(const AdaptiveElement<INPUT>& input, OUTPUT* output) :
		input_(input), timestamp_(), waveform_(Waveforms::NONE), period_(), phase_(), output_(
				output), amplitude_(), override_(false)
{
}

template<typename INPUT, typename OUTPUT>
inline void
Output<INPUT, OUTPUT>::evaluate()
{
	*output_ = override_ ? getWaveformOutput() : input_->getValue();
}

template<typename INPUT, typename OUTPUT>
inline OUTPUT
Output<INPUT, OUTPUT>::getValue() const
{
	return *output_;
}

template<typename INPUT, typename OUTPUT>
inline void
Output<INPUT, OUTPUT>::setInput(const AdaptiveElement<INPUT>& input)
{
	input_ = input;
}

template<typename INPUT, typename OUTPUT>
inline void
Output<INPUT, OUTPUT>::setWaveform(const Waveforms& waveform)
{
	waveform_ = waveform;
}

template<typename INPUT, typename OUTPUT>
inline void
Output<INPUT, OUTPUT>::setPeriod(const OUTPUT& period)
{
	period_ = period;
}

template<typename INPUT, typename OUTPUT>
inline void
Output<INPUT, OUTPUT>::setPhase(const OUTPUT& phase)
{
	phase_ = phase;
}

template<typename INPUT, typename OUTPUT>
inline void
Output<INPUT, OUTPUT>::setOutput(OUTPUT* output)
{
	output_ = output;
}

template<typename INPUT, typename OUTPUT>
inline void
Output<INPUT, OUTPUT>::overrideOutput(const OUTPUT& amplitude)
{
	amplitude_ = amplitude;
	timestamp_ = boost::posix_time::microsec_clock::local_time();
	override_ = true;
}

template<typename INPUT, typename OUTPUT>
inline void
Output<INPUT, OUTPUT>::disableOverride()
{
	override_ = false;
	timestamp_ = TimePoint();
	waveform_ = Waveforms::NONE;
	period_ = OUTPUT();
	phase_ = OUTPUT();
}

template<typename INPUT, typename OUTPUT>
inline OUTPUT
Output<INPUT, OUTPUT>::getWaveformOutput()
{
	OUTPUT waveformOutput = OUTPUT();

	switch (waveform_)
	{
	case Waveforms::NONE:
	{
		waveformOutput = amplitude_;

		break;
	}
	case Waveforms::SINE:
	{
		TimePoint current = boost::posix_time::microsec_clock::local_time();
		double time = (current - timestamp_).total_milliseconds();
		double period = (2 * M_PI) / period_;

		waveformOutput = amplitude_ * sin(period * (time + phase_));

		break;
	}
	case Waveforms::SQUARE:
	{
		TimePoint current = boost::posix_time::microsec_clock::local_time();
		double time = (current - timestamp_).total_milliseconds();
		double period = (2 * M_PI) / period_;
		double sine = sin(period * (time + phase_));

		if (sine > 0)
		{
			waveformOutput = amplitude_;
		}
		else if (sine == 0)
		{
			waveformOutput = 0;
		}
		else if (sine < 0)
		{
			waveformOutput = -amplitude_;
		}

		break;
	}
	case Waveforms::RAMP:
	{
		TimePoint current = boost::posix_time::microsec_clock::local_time();
		double time = (current - timestamp_).total_milliseconds();
		double time_mod = fmod(time + phase_, period_);
		double slope = (2 * amplitude_) / period_;
		double intercept = -amplitude_;

		waveformOutput = slope * time_mod + intercept;

		break;
	}
	case Waveforms::SAWTOOTH:
	{
		TimePoint current = boost::posix_time::microsec_clock::local_time();
		double time = (current - timestamp_).total_milliseconds();
		double time_mod = fmod(time + phase_, period_);
		double slope = -(2 * amplitude_) / period_;
		double intercept = amplitude_;

		waveformOutput = slope * time_mod + intercept;

		break;
	}
	case Waveforms::TRIANGULAR:
	{
		waveformOutput = amplitude_;

		break;
	}
	default:
	{
		waveformOutput = amplitude_;

		break;
	}
	}

	return waveformOutput;
}

#endif /* UAVAP_FLIGHTCONTROL_CONTROLLER_ADAPTIVECONTROLENVIRONMENT_EVALUABLEADAPTIVECONTROLELEMENTS_OUTPUT_HPP_ */
