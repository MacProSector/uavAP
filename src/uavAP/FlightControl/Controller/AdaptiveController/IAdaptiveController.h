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
 * IAdaptiveController.h
 *
 *  Created on: Jun 3, 2019
 *      Author: simonyu
 */

#ifndef UAVAP_FLIGHTCONTROL_CONTROLLER_ADAPTIVECONTROLLER_IADAPTIVECONTROLLER_H_
#define UAVAP_FLIGHTCONTROL_CONTROLLER_ADAPTIVECONTROLLER_IADAPTIVECONTROLLER_H_

#include "uavAP/FlightControl/Controller/IController.h"
#include "uavAP/FlightControl/Controller/ControllerOutput.h"
#include "uavAP/FlightControl/Controller/AdaptiveController/IAdaptiveCascade.h"

class IAdaptiveController: public IController
{
public:

	virtual
	~IAdaptiveController() = default;

	virtual bool
	configure(const boost::property_tree::ptree& config) = 0;

	virtual ControllerOutput
	getControllerOutput() const = 0;

	virtual std::shared_ptr<IAdaptiveCascade>
	getCascade() const = 0;
};

#endif /* UAVAP_FLIGHTCONTROL_CONTROLLER_ADAPTIVECONTROLLER_IADAPTIVECONTROLLER_H_ */
