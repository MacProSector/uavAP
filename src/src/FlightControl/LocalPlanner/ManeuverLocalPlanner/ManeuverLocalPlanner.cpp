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
 * @file ManeuverLocalPlanner.cpp
 * @date Aug 2, 2018
 * @author Mirco Theile, mirco.theile@tum.de
 * @brief Description
 */
#include <uavAP/Core/LockTypes.h>
#include <uavAP/Core/Scheduler/IScheduler.h>
#include <uavAP/FlightControl/Controller/IController.h>
#include <uavAP/FlightControl/SensingActuationIO/SensingActuationIO.h>
#include "uavAP/FlightControl/LocalPlanner/ManeuverLocalPlanner/ManeuverLocalPlanner.h"
#include <uavAP/MissionControl/ManeuverPlanner/Override.h>
#include <uavAP/Core/PropertyMapper/ConfigurableObjectImpl.hpp>
#include <uavAP/Core/Object/AggregatableObjectImpl.hpp>
#include <uavAP/Core/DataHandling/DataHandling.h>
#include <uavAP/Core/DataPresentation/DataPresentation.h>
#include <uavAP/Core/IPC/IPC.h>

ManeuverLocalPlanner::ManeuverLocalPlanner()
{
}

void
ManeuverLocalPlanner::setTrajectory(const Trajectory& traj)
{
	Lock lock(trajectoryMutex_);
	trajectory_ = traj;
	currentSection_ = trajectory_.pathSections.begin();
	lock.unlock();

	Lock lockStatus(statusMutex_);
	status_.currentPathSection = 0;
	status_.isInApproach = trajectory_.approachSection != nullptr;
	lockStatus.unlock();

	APLOG_DEBUG << "Trajectory set.";
}

Trajectory
ManeuverLocalPlanner::getTrajectory() const
{
	LockGuard lock(trajectoryMutex_);
	return trajectory_;
}

ManeuverLocalPlannerStatus
ManeuverLocalPlanner::getStatus() const
{
	return status_;
}

bool
ManeuverLocalPlanner::run(RunStage stage)
{
	switch (stage)
	{
	case RunStage::INIT:
	{
		if (!checkIsSet<IController, ISensingActuationIO, IScheduler, IPC, DataPresentation>())
		{
			APLOG_ERROR << "LinearLocalPlanner: Dependency missing";
			return true;
		}
		if (!isSet<DataHandling>())
		{
			APLOG_DEBUG << "ManeuverLocalPlanner: DataHandling not set. Debugging disabled.";
		}

		break;
	}
	case RunStage::NORMAL:
	{

		//Directly calculate local plan when sensor data comes in
		if (params.period() == 0)
		{
			APLOG_DEBUG << "Calculate control on sensor data trigger";
			auto sensing = get<ISensingActuationIO>();
			sensing->subscribeOnSensorData(
					boost::bind(&ManeuverLocalPlanner::onSensorData, this, _1));
		}
		else
		{
			APLOG_DEBUG << "Calculate control with period " << params.period();
			auto scheduler = get<IScheduler>();
			scheduler->schedule(std::bind(&ManeuverLocalPlanner::update, this),
					Milliseconds(params.period()), Milliseconds(params.period()));
		}

		auto ipc = get<IPC>();

		ipc->subscribeOnPackets("trajectory",
				std::bind(&ManeuverLocalPlanner::onTrajectoryPacket, this, std::placeholders::_1));

		ipc->subscribeOnPackets("override",
				std::bind(&ManeuverLocalPlanner::onOverridePacket, this, std::placeholders::_1));

		if (auto dh = get<DataHandling>())
		{
			dh->addStatusFunction<ManeuverLocalPlannerStatus>(
					std::bind(&ManeuverLocalPlanner::getStatus, this),
					Content::MANEUVER_LOCAL_PLANNER_STATUS);
			dh->subscribeOnCommand<ManeuverLocalPlannerParams>(Content::TUNE_MANEUVER_LOCAL_PLANNER,
					std::bind(&ManeuverLocalPlanner::setParams, this, std::placeholders::_1));
			dh->addTriggeredStatusFunction<Trajectory, DataRequest>(
					std::bind(&ManeuverLocalPlanner::trajectoryRequest, this,
							std::placeholders::_1), Content::TRAJECTORY, Content::REQUEST_DATA);
		}

		break;
	}
	case RunStage::FINAL:
	{
		break;
	}
	default:
	{
		break;
	}
	}
	return false;
}

void
ManeuverLocalPlanner::createLocalPlan(const Vector3& position, double heading, bool hasGPSFix,
		uint32_t seqNum)
{
	bool safety = false;

	Lock lock(trajectoryMutex_);
	auto currentSection = updatePathSection(position);
	if (!currentSection)
	{
		APLOG_ERROR << "No current pathsection. Fly safety procedure.";
		safety = true;
	}

	if (!hasGPSFix)
	{
		APLOG_ERROR << "Lost GPS fix. LocalPlanner safety procedure.";
		safety = true;
	}

	Lock plannerLock(overrideMutex_);

	if (safety)
	{
		controllerTarget_.velocity = params.safetyVelocity();
		controllerTarget_.yawRate = params.safetyYawRate();
		controllerTarget_.climbAngle = 0;
	}
	else
	{
		controllerTarget_ = calculateControllerTarget(position, heading, currentSection);
	}

	//Do control overrides
	if (auto it = findInMap(targetOverrides_, ControllerTargets::VELOCITY))
		controllerTarget_.velocity = it->second;
	if (auto it = findInMap(targetOverrides_, ControllerTargets::CLIMB_ANGLE))
		controllerTarget_.climbAngle = it->second;
	if (auto it = findInMap(targetOverrides_, ControllerTargets::YAW_RATE))
		controllerTarget_.yawRate = it->second;

	plannerLock.unlock();

	controllerTarget_.sequenceNr = seqNum;
	status_.climbAngleTarget = controllerTarget_.climbAngle;
	status_.velocityTarget = controllerTarget_.velocity;
	status_.yawRateTarget = controllerTarget_.yawRate;

	auto controller = get<IController>();
	if (!controller)
	{
		APLOG_ERROR << "LinearLocalPlanner: Controller missing";
		return;
	}

	controller->setControllerTarget(controllerTarget_);
}

std::shared_ptr<IPathSection>
ManeuverLocalPlanner::updatePathSection(const Vector3& position)
{
	std::shared_ptr<IPathSection> currentSection;

	if (!status_.isInApproach)
	{
		if (currentSection_ == trajectory_.pathSections.end())
		{
			APLOG_ERROR << "Trajectory at the end.";
			return nullptr;
		}
		currentSection = *currentSection_;
	}
	else
	{
		currentSection = trajectory_.approachSection;
	}

	if (!currentSection)
	{
		APLOG_ERROR << "Current Section is nullptr. Abort.";
		return nullptr;
	}
	currentSection->updatePosition(position);

	if (currentSection->inTransition())
	{
		nextSection();

		if (currentSection_ == trajectory_.pathSections.end())
		{
			APLOG_ERROR << "Trajectory at the end.";
			return nullptr;
		}

		currentSection = *currentSection_;
		if (!currentSection)
		{
			APLOG_ERROR << "Current Section is nullptr. Abort.";
			return nullptr;
		}
		currentSection->updatePosition(position);
	}

	return currentSection;

}

void
ManeuverLocalPlanner::nextSection()
{
	//Status mutex is locked already
	if (status_.isInApproach)
	{
		//We were approaching the first waypoint
		currentSection_ = trajectory_.pathSections.begin();
		status_.isInApproach = false;
		return;
	}

	if (currentSection_ == trajectory_.pathSections.end())
	{
		return;
	}
	++currentSection_;
	++status_.currentPathSection;

	if (currentSection_ == trajectory_.pathSections.end() && trajectory_.infinite)
	{
		currentSection_ = trajectory_.pathSections.begin();
		status_.currentPathSection = 0;
	}
}

ControllerTarget
ManeuverLocalPlanner::calculateControllerTarget(const Vector3& position, double heading,
		std::shared_ptr<IPathSection> section)
{
	ControllerTarget controllerTarget;

	double vel = section->getVelocity();

	if (auto it = findInMap(plannerOverrides_, LocalPlannerTargets::VELOCITY))
		vel = it->second;

	controllerTarget.velocity = vel;
	auto positionDeviation = section->getPositionDeviation();

	if (auto it = findInMap(plannerOverrides_, LocalPlannerTargets::POSITION_X))
		positionDeviation[0] = it->second - position[0];
	if (auto it = findInMap(plannerOverrides_, LocalPlannerTargets::POSITION_Y))
		positionDeviation[1] = it->second - position[1];
	if (auto it = findInMap(plannerOverrides_, LocalPlannerTargets::POSITION_Z))
		positionDeviation[2] = it->second - position[2];

	// Climb Rate
	double slope = section->getSlope();
	double climbRate = vel * slope * sqrt(1 / (1 + slope * slope))
			+ params.kAltitude() * positionDeviation.z();

	if (auto it = findInMap(plannerOverrides_, LocalPlannerTargets::SLOPE))
	{
		//Override climbrate, shall not approach altitude
		double slope = it->second;
		climbRate = vel * slope * sqrt(1 / (1 + slope * slope));
	}

	if (auto it = findInMap(plannerOverrides_, LocalPlannerTargets::CLIMB_RATE))
		climbRate = it->second;

	climbRate = climbRate > vel ? vel : climbRate < -vel ? -vel : climbRate;

	//Climb angle
	controllerTarget.climbAngle = asin(climbRate / vel);

	// Heading
	Vector3 direction = section->getDirection();
	if (auto it = findInMap(plannerOverrides_, LocalPlannerTargets::DIRECTION_X))
		direction[0] = it->second;
	if (auto it = findInMap(plannerOverrides_, LocalPlannerTargets::DIRECTION_Y))
		direction[1] = it->second;
	if (auto it = findInMap(plannerOverrides_, LocalPlannerTargets::DIRECTION_Z))
		direction[2] = it->second;

	Vector2 directionTarget = params.kConvergence() * positionDeviation.head(2)
			+ direction.head(2).normalized();
	double headingTarget = headingFromENU(directionTarget);

	double curvature = section->getCurvature();
	if (auto it = findInMap(plannerOverrides_, LocalPlannerTargets::HEADING))
	{
		headingTarget = it->second;
		curvature = 0;
	}

	double headingError = boundAngleRad(headingTarget - heading);

	// Yaw Rate

	controllerTarget.yawRate = vel * curvature + params.kYawRate() * headingError;

	return controllerTarget;
}

void
ManeuverLocalPlanner::onTrajectoryPacket(const Packet& packet)
{
	APLOG_DEBUG << "On Trajectory packet";
	auto dp = get<DataPresentation>();

	try
	{
		setTrajectory(dp->deserialize<Trajectory>(packet));
	} catch (ArchiveError& err)
	{
		APLOG_ERROR << "Invalid Trajectory packet: " << err.what();
		return;
	}
}

void
ManeuverLocalPlanner::onSensorData(const SensorData& sd)
{
	//TODO Lock?
	Vector3 position = sd.position;
	double heading = sd.attitude.z();
	bool hasFix = sd.hasGPSFix;
	uint32_t seq = sd.sequenceNr;

	createLocalPlan(position, heading, hasFix, seq);
}

void
ManeuverLocalPlanner::onOverridePacket(const Packet& packet)
{
	auto override = dp::deserialize<Override>(packet);

	std::unique_lock<std::mutex> plannerLock(overrideMutex_);
	plannerOverrides_ = override.localPlanner;
	targetOverrides_ = override.controllerTarget;
}

void
ManeuverLocalPlanner::update()
{
	auto sensing = get<ISensingActuationIO>();

	if (!sensing)
	{
		APLOG_ERROR << "ManeuverLocalPlanner: sensing missing";
		return;
	}

	SensorData data = sensing->getSensorData();
	createLocalPlan(data.position, data.attitude.z(), data.hasGPSFix, data.sequenceNr);
}

Trajectory
ManeuverLocalPlanner::trajectoryRequest(const DataRequest& request)
{
	APLOG_DEBUG << "Called trajectoryRequest with " << (int) request;
	if (request == DataRequest::TRAJECTORY)
		return getTrajectory();
	return Trajectory();
}
