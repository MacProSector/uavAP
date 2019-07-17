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
 * MissionControlDataHandling.h
 *
 *  Created on: Sep 6, 2017
 *      Author: mircot
 */

#ifndef UAVAP_MISSIONCONTROL_DATAHANDLING_MISSIONCONTROLDATAHANDLING_H_
#define UAVAP_MISSIONCONTROL_DATAHANDLING_MISSIONCONTROLDATAHANDLING_H_

#include <boost/property_tree/ptree.hpp>

#include "uavAP/Core/DataPresentation/Packet.h"
#include "uavAP/Core/IPC/Publisher.h"
#include "uavAP/Core/IPC/Subscription.h"
#include "uavAP/Core/Object/IAggregatableObject.h"
#include "uavAP/Core/Object/ObjectHandle.h"
#include "uavAP/Core/Runner/IRunnableObject.h"
#include "uavAP/Core/Time.h"

enum class Content
;
enum class Target
;

template<typename C, typename T>
class IDataPresentation;

class IPC;
class IScheduler;
class IGlobalPlanner;
class IMissionPlanner;
class ConditionManager;
class EmergencyGeofencingPlanner;
class LocalFrameManager;
class ManeuverPlanner;

class MissionControlDataHandling: public IAggregatableObject, public IRunnableObject
{
public:

	static constexpr TypeId typeId = "mc_data_handling";

	MissionControlDataHandling();

	static std::shared_ptr<MissionControlDataHandling>
	create(const boost::property_tree::ptree& configuration);

	bool
	configure(const boost::property_tree::ptree& configuration);

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
	collectAndSendMission(std::shared_ptr<IDataPresentation<Content, Target>> dp);

	void
	collectAndSendSafetyBounds(std::shared_ptr<IDataPresentation<Content, Target>> dp);

	void
	collectAndSendLocalFrame(std::shared_ptr<IDataPresentation<Content, Target>> dp);

	ObjectHandle<IPC> ipc_;
	ObjectHandle<IScheduler> scheduler_;
	ObjectHandle<IGlobalPlanner> globalPlanner_;
	ObjectHandle<ManeuverPlanner> maneuverPlanner_;
	ObjectHandle<IMissionPlanner> missionPlanner_;
	ObjectHandle<IDataPresentation<Content, Target>> dataPresentation_;
	ObjectHandle<LocalFrameManager> localFrameManager_;
	ObjectHandle<ConditionManager> conditionManager_;
	ObjectHandle<EmergencyGeofencingPlanner> emergencyGeofencingPlanner_;

	Subscription missionControlSubscription_;
	Publisher publisher_;
	Publisher overridePublisher_;

	unsigned int lastOverrideSeqNr_;
	Duration period_;
};

#endif /* UAVAP_MISSIONCONTROL_DATAHANDLING_MISSIONCONTROLDATAHANDLING_H_ */
