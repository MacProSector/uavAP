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
 * GeofencingModelFactory.h
 *
 *  Created on: Aug 15, 2018
 *      Author: mircot
 */

#ifndef UAVAP_MISSIONCONTROL_EMERGENCYPLANNER_EMERGENCYGEOFENCINGPLANNER_GEOFENCINGMODEL_GEOFENCINGMODELFACTORY_H_
#define UAVAP_MISSIONCONTROL_EMERGENCYPLANNER_EMERGENCYGEOFENCINGPLANNER_GEOFENCINGMODEL_GEOFENCINGMODELFACTORY_H_

#include "uavAP/Core/Framework/Factory.h"
#include "uavAP/MissionControl/EmergencyPlanner/EmergencyGeofencingPlanner/GeofencingModel/IGeofencingModel.h"
#include "uavAP/MissionControl/EmergencyPlanner/EmergencyGeofencingPlanner/GeofencingModel/ConstRollRateGeofencingModel/ConstRollRateGeofencingModel.h"
#include "uavAP/MissionControl/EmergencyPlanner/EmergencyGeofencingPlanner/GeofencingModel/DirectRollGeofencingModel/DirectRollGeofencingModel.h"

class GeofencingModelFactory: public Factory<IGeofencingModel>
{
public:

	GeofencingModelFactory()
	{
		addCreator<ConstRollRateGeofencingModel>();
		addCreator<DirectRollGeofencingModel>();
	}
};

#endif /* UAVAP_MISSIONCONTROL_EMERGENCYPLANNER_EMERGENCYGEOFENCINGPLANNER_GEOFENCINGMODEL_GEOFENCINGMODELFACTORY_H_ */
