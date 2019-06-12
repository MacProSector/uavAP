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
 *  @file         DataHandlingIO.h
 *  @author Simon Yu
 *  @date      26 July 2017
 *  @brief      UAV Autopilot Flight Control Data Handling IO Header File
 *
 *  Description
 */

#ifndef UAVAP_FLIGHTCONTROL_DATAHANDLING_FLIGHTCONTROLDATAHANDLING_H_
#define UAVAP_FLIGHTCONTROL_DATAHANDLING_FLIGHTCONTROLDATAHANDLING_H_

#include <boost/property_tree/ptree.hpp>

#include "uavAP/Core/Object/IAggregatableObject.h"
#include "uavAP/Core/DataPresentation/Packet.h"
#include "uavAP/Core/Runner/IRunnableObject.h"
#include "uavAP/Core/Object/ObjectHandle.h"
#include "uavAP/Core/IPC/IPC.h"
#include "uavAP/Core/DataPresentation/IDataPresentation.h"
#include "uavAP/FlightControl/Controller/PIDController/ManeuverPIDController/ManeuverPIDController.h"
#include "uavAP/FlightControl/Controller/PIDController/SimplePIDController/SimplePIDController.h"
#include "uavAP/FlightControl/Controller/PIDController/PIDHandling.h"
#include "uavAP/FlightControl/Controller/PIDController/PIDMapping.h"

class IGlobalPlanner;
class ILocalPlanner;
class IController;
class IScheduler;
enum class Content
;
enum class Target
;

template<typename C, typename T>
class IDataPresentation;
struct ControllerOutput;
class LocalPlannerParams;
class ISensingActuationIO;

class FlightControlDataHandling: public IAggregatableObject, public IRunnableObject
{
public:

	static constexpr TypeId typeId = "fc_data_handling";

	FlightControlDataHandling();

	static std::shared_ptr<FlightControlDataHandling>
	create(const Configuration& configuration);

	bool
	configure(const Configuration& configuration);

	bool
	run(RunStage stage) override;

	void
	notifyAggregationOnUpdate(const Aggregator& agg) override;

private:

	void
	collectAndSend();

	void
	receiveAndDistribute(const Packet& packet);

	void
	collectAndSendSensorData(std::shared_ptr<IDataPresentation<Content, Target>> dp);

	void
	collectAndSendTrajectory(std::shared_ptr<IDataPresentation<Content, Target>> dp);

	void
	collectAndSendPIDStatus(std::shared_ptr<IDataPresentation<Content, Target>> dp);

	void
	collectAndSendLocalPlannerStatus(std::shared_ptr<IDataPresentation<Content, Target>> dp);

	void
	tunePID(const PIDTuning& params);

	void
	tuneLocalPlanner(const LocalPlannerParams& params);

	ObjectHandle<IPC> ipc_;
	ObjectHandle<ILocalPlanner> localPlanner_;
	ObjectHandle<IController> controller_;
	ObjectHandle<ISensingActuationIO> sensActIO_;
	ObjectHandle<IScheduler> scheduler_;
	ObjectHandle<IDataPresentation<Content, Target>> dataPresentation_;
	Subscription flightControlSubscription_;
	Publisher publisher_;
	Publisher pidStatiPublisher_;
	Publisher advancedControlPublisher_;

	Duration period_;
	bool compressSensorData_;
};

#endif /* UAVAP_FLIGHTCONTROL_DATAHANDLING_FLIGHTCONTROLDATAHANDLING_H_ */
