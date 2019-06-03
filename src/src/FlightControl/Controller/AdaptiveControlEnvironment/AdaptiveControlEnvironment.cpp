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

AdaptiveControlEnvironment::AdaptiveControlEnvironment() : timestamp_(nullptr)
{
}

AdaptiveControlEnvironment::AdaptiveControlEnvironment(const TimePoint* timestamp) :
		timestamp_(timestamp)
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

	for (auto& it : evaluableAdaptiveElements_)
	{
		it();
	}
}
