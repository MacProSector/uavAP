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
 * AdaptiveControlEnvironment.cpp
 *
 *  Created on: May 29, 2019
 *      Author: simonyu
 */

#include "uavAP/Core/Logging/APLogger.h"
#include "uavAP/FlightControl/Controller/AdaptiveControlEnvironment/AdaptiveControlEnvironment.h"

AdaptiveControlEnvironment::AdaptiveControlEnvironment() :
		autopilotActive_(nullptr), timestamp_(nullptr), lastAutopilotActive_(false)
{
}

AdaptiveControlEnvironment::AdaptiveControlEnvironment(const TimePoint* timestamp) :
		autopilotActive_(nullptr), timestamp_(timestamp), lastAutopilotActive_(false)
{
}

AdaptiveControlEnvironment::AdaptiveControlEnvironment(const bool* autopilotActive,
		const TimePoint* timestamp) :
		autopilotActive_(autopilotActive), timestamp_(timestamp), lastAutopilotActive_(false)
{
}

void
AdaptiveControlEnvironment::evaluate()
{
	if (timestamp_)
	{
		if (lastTimestamp_.is_not_a_date_time())
		{
			duration_ = Microseconds(0);
		}
		else
		{
			duration_ = *timestamp_ - lastTimestamp_;
		}

		lastTimestamp_ = *timestamp_;
	}

	if (autopilotActive_)
	{
		bool autopilotActive = *autopilotActive_;

		if (!lastAutopilotActive_ && autopilotActive)
		{
			resetIntegrator();
			resetState();
		}

		lastAutopilotActive_ = autopilotActive;
	}

	for (auto& it : evaluableAdaptiveElements_)
	{
		it();
	}
}

void
AdaptiveControlEnvironment::resetIntegrator()
{
	for (auto& it : pidElements_)
	{
		it();
	}
}

void
AdaptiveControlEnvironment::resetState()
{
	for (auto& it : stateSpaceElements_)
	{
		it();
	}
}
