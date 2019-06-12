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
/**
 * @file Content.h
 * @brief Definitions of possible data contents of packets.
 * @date Aug 8, 2017
 * @author Mirco Theile, mirco.theile@tum.de
 */

#ifndef UAVAP_CORE_DATAPRESENTATION_CONTENTMAPPING_H_
#define UAVAP_CORE_DATAPRESENTATION_CONTENTMAPPING_H_

#include <utility>

#include "uavAP/Core/DataPresentation/Content.h"
#include "uavAP/API/ChannelMixing.h"
#include "uavAP/Core/protobuf/messages/Shapes.pb.h"
#include "uavAP/Core/protobuf/messages/LocalPlanner.pb.h"
#include "uavAP/Core/DataPresentation/APDataPresentation/BasicSerialization.h"
#include "uavAP/Core/SensorData.h"
#include "uavAP/FlightAnalysis/StateAnalysis/Metrics.h"
#include "uavAP/FlightControl/Controller/PIDController/PIDHandling.h"
#include "uavAP/FlightControl/Controller/ControllerOutput.h"
#include "uavAP/FlightControl/Controller/AdvancedControl.h"
#include "uavAP/MissionControl/ManeuverPlanner/Override.h"
#include "uavAP/MissionControl/GlobalPlanner/Trajectory.h"
#include "uavAP/MissionControl/MissionPlanner/Mission.h"
#include "uavAP/Core/Frames/VehicleOneFrame.h"

/**
 * Defines how to write the case labels in the switch case of the data presentation.
 */
#define MAP(CONTENT, TYPE, FUNC) case Content::CONTENT: {FUNC(TYPE);}

/**
 * Add Mapping for Content and DataType here.
 * Format:
 * 	MAP(<CONTENT_ENUM>, <DATA_TYPE>, FUNC)
 */
#define MAPPING(FUNC)		MAP(SENSOR_DATA, SensorData, FUNC) 												\
							MAP(MISSION, Mission, FUNC)														\
							MAP(TRAJECTORY, Trajectory, FUNC)												\
							MAP(PID_STATUS, PIDStati, FUNC)													\
							MAP(INSPECTING_METRICS, SteadyStateMetrics, FUNC)								\
							MAP(LOCAL_PLANNER_STATUS, LocalPlannerStatus, FUNC)								\
							MAP(SAFETY_BOUNDS, Rectanguloid, FUNC)											\
							MAP(CONTROLLER_OUTPUT, ControllerOutput, FUNC)									\
							MAP(LOCAL_FRAME, VehicleOneFrame, FUNC)											\
							MAP(TUNE_PID, PIDTuning, FUNC)													\
							MAP(TUNE_PITCH_CONSTRAINT, ConstraintParams, FUNC)								\
							MAP(TUNE_ROLL_CONSTRAINT, ConstraintParams, FUNC)								\
							MAP(TUNE_LOCAL_PLANNER, LocalPlannerParams, FUNC) 								\
							MAP(REQUEST_DATA, DataRequest, FUNC)											\
							MAP(ADVANCED_CONTROL, AdvancedControl, FUNC)									\
							MAP(OVERRIDE, Override, FUNC)													\
							MAP(SELECT_MANEUVER_SET, std::string, FUNC)										\
							MAP(SELECT_MISSION, std::string, FUNC)											\
							MAP(SELECT_INSPECTING_METRICS, InspectingMetricsPair, FUNC)						\

#endif /* UAVAP_CORE_DATAPRESENTATION_CONTENTMAPPING_H_ */
