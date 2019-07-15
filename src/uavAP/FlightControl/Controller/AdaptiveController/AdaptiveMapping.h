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
 * AdaptiveMapping.h
 *
 *  Created on: Jun 12, 2019
 *      Author: simonyu
 */

#ifndef UAVAP_FLIGHTCONTROL_CONTROLLER_ADAPTIVECONTROLLER_ADAPTIVEMAPPING_H_
#define UAVAP_FLIGHTCONTROL_CONTROLLER_ADAPTIVECONTROLLER_ADAPTIVEMAPPING_H_

#include "uavAP/Core/EnumMap.hpp"

enum class Adaptives
{
	INVALID = 0, ROLL, PITCH, YAW, NUM_ADAPTIVE
};

ENUMMAP_INIT(Adaptives, {{Adaptives::ROLL, "roll"}, {Adaptives::PITCH, "pitch"}, {Adaptives::YAW,
		"yaw"}});

#endif /* UAVAP_FLIGHTCONTROL_CONTROLLER_ADAPTIVECONTROLLER_ADAPTIVEMAPPING_H_ */
