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
 * L1AdaptiveController.h
 *
 *  Created on: Jun 3, 2019
 *      Author: simonyu
 */

#ifndef UAVAP_FLIGHTCONTROL_CONTROLLER_ADAPTIVECONTROLLER_L1ADAPTIVECONTROLLER_L1ADAPTIVECONTROLLER_H_
#define UAVAP_FLIGHTCONTROL_CONTROLLER_ADAPTIVECONTROLLER_L1ADAPTIVECONTROLLER_L1ADAPTIVECONTROLLER_H_

#include <mutex>

#include "uavAP/Core/IPC/Publisher.h"
#include "uavAP/Core/IPC/Subscription.h"
#include "uavAP/Core/Object/ObjectHandle.h"
#include "uavAP/Core/Object/IAggregatableObject.h"
#include "uavAP/Core/Runner/IRunnableObject.h"
#include "uavAP/FlightControl/Controller/AdaptiveController/IAdaptiveController.h"

class IPC;
class IScheduler;
class ISensingActuationIO;
class IAdaptiveCascade;
class L1AdaptiveCascade;
class Packet;
class SensorData;
class ControllerTarget;
class ControllerOutput;

class L1AdaptiveController: public IAdaptiveController, public IAggregatableObject, public IRunnableObject
{
public:

	static constexpr TypeId typeId = "l1";

	L1AdaptiveController();

	static std::shared_ptr<L1AdaptiveController>
	create(const boost::property_tree::ptree& config);

	bool
	configure(const boost::property_tree::ptree& config) override;

	bool
	run(RunStage stage) override;

	void
	notifyAggregationOnUpdate(const Aggregator& agg) override;

	ControllerOutput
	getControllerOutput() const override;

	std::shared_ptr<IAdaptiveCascade>
	getCascade() const override;

	void
	setControllerTarget(const ControllerTarget& target) override;

private:

	void
	calculateControl();

	void
	onOverridePacket(const Packet& packet);

	ObjectHandle<IPC> ipc_;
	ObjectHandle<IScheduler> scheduler_;
	ObjectHandle<ISensingActuationIO> sensingActuationIO_;

	std::shared_ptr<L1AdaptiveCascade> l1AdaptiveCascade_;

	SensorData sensorData_;

	ControllerTarget controllerTarget_;
	std::mutex controllerTargetMutex_;

	ControllerOutput controllerOutput_;

	Publisher controllerOutputPublisherMP_;
	Publisher controllerOutputPublisherTA_;
	Publisher controllerOutputPublisherMA_;
	Subscription overrideSubscription_;
};

#endif /* UAVAP_FLIGHTCONTROL_CONTROLLER_ADAPTIVECONTROLLER_L1ADAPTIVECONTROLLER_L1ADAPTIVECONTROLLER_H_ */
