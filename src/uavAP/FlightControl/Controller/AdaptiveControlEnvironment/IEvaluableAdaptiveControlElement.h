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
 * IEvaluableAdaptiveControlElement.h
 *
 *  Created on: May 29, 2019
 *      Author: simonyu
 */

#ifndef UAVAP_FLIGHTCONTROL_CONTROLLER_ADAPTIVECONTROLENVIRONMENT_IEVALUABLEADAPTIVECONTROLELEMENT_H_
#define UAVAP_FLIGHTCONTROL_CONTROLLER_ADAPTIVECONTROLENVIRONMENT_IEVALUABLEADAPTIVECONTROLELEMENT_H_

#include "uavAP/FlightControl/Controller/AdaptiveControlEnvironment/IAdaptiveControlElement.h"

template<typename TYPE>
class IEvaluableAdaptiveControlElement : public IAdaptiveControlElement<TYPE>
{
public:

	virtual
	~IEvaluableAdaptiveControlElement() = default;

	virtual void
	evaluate() = 0;
};

template<typename TYPE>
using EvaluableAdaptiveElement = std::shared_ptr<IEvaluableAdaptiveControlElement<TYPE>>;

#endif /* UAVAP_FLIGHTCONTROL_CONTROLLER_ADAPTIVECONTROLENVIRONMENT_IEVALUABLEADAPTIVECONTROLELEMENT_H_ */
