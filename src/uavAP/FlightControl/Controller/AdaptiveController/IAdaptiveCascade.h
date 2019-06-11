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
 * IAdaptiveCascade.h
 *
 *  Created on: Jun 3, 2019
 *      Author: simonyu
 */

#ifndef UAVAP_FLIGHTCONTROL_CONTROLLER_ADAPTIVECONTROLLER_IADAPTIVECASCADE_H_
#define UAVAP_FLIGHTCONTROL_CONTROLLER_ADAPTIVECONTROLLER_IADAPTIVECASCADE_H_

#include <map>
#include <boost/property_tree/ptree.hpp>

#include "uavAP/FlightControl/Controller/PIDController/PIDHandling.h"
#include "uavAP/FlightControl/Controller/PIDController/PIDMapping.h"

class IAdaptiveCascade
{
public:

	virtual
	~IAdaptiveCascade() = default;

	virtual bool
	configure(const boost::property_tree::ptree& config) = 0;

	virtual std::map<PIDs, PIDStatus>
	getPIDStatus() const = 0;

	virtual void
	evaluate() = 0;

	virtual bool
	tunePID(PIDs pid, const PIDParameter& params) = 0;
};

#endif /* UAVAP_FLIGHTCONTROL_CONTROLLER_ADAPTIVECONTROLLER_IADAPTIVECASCADE_H_ */
