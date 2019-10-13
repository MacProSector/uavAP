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
 * EmergencyLandingPlanner.cpp
 *
 *  Created on: Jul 9, 2019
 *      Author: simonyu
 */

#include "uavAP/Core/IPC/IPC.h"
#include "uavAP/Core/PropertyMapper/PropertyMapper.h"
#include "uavAP/MissionControl/EmergencyPlanner/EmergencyLandingPlanner/EmergencyLandingPlan.h"
#include "uavAP/MissionControl/EmergencyPlanner/EmergencyLandingPlanner/EmergencyLandingPlanner.h"
#include "uavAP/MissionControl/EmergencyPlanner/EmergencyLandingPlanner/EmergencyLandingStatus.h"
#include "uavAP/MissionControl/ManeuverPlanner/ManeuverPlanner.h"

EmergencyLandingPlanner::EmergencyLandingPlanner()
{
}

std::shared_ptr<EmergencyLandingPlanner>
EmergencyLandingPlanner::create(const boost::property_tree::ptree& config)
{
	auto emergencyLandingPlanner = std::make_shared<EmergencyLandingPlanner>();

	if (!emergencyLandingPlanner->configure(config))
	{
		APLOG_ERROR << "EmergencyLandingPlanner: Failed to Load Config.";
	}

	return emergencyLandingPlanner;
}

bool
EmergencyLandingPlanner::configure(const boost::property_tree::ptree& config)
{
	PropertyMapper pm(config);
	boost::property_tree::ptree landingSiteConfig;

	pm.add("landing_sites", landingSiteConfig, true);

	PropertyMapper landingSitePm(landingSiteConfig);

	for (const auto& it : landingSiteConfig)
	{
		if (it.first == "default")
		{
			EmergencyLandingParameter landingParameter;

			landingParameter.name = it.first;
			landingParameter.isDefault = true;

			landingSitePm.addStruct<EmergencyLandingParameter>(it.first, landingParameter, true);

			landingParameters_.push_back(landingParameter);
		}
	}

	if (landingSitePm.map())
	{
		for (const auto& it : landingSiteConfig)
		{
			if (it.first != "default")
			{
				EmergencyLandingParameter landingParameter;

				landingParameter = landingParameters_[0];
				landingParameter.name = it.first;
				landingParameter.isDefault = false;

				landingSitePm.addStruct<EmergencyLandingParameter>(it.first, landingParameter,
						true);

				landingParameters_.push_back(landingParameter);
			}
		}
	}

	return pm.map() && landingSitePm.map();
}

void
EmergencyLandingPlanner::notifyAggregationOnUpdate(const Aggregator& agg)
{
	ipc_.setFromAggregationIfNotSet(agg);
	scheduler_.setFromAggregationIfNotSet(agg);
	maneuverPlanner_.setFromAggregationIfNotSet(agg);
}

bool
EmergencyLandingPlanner::run(RunStage stage)
{
	switch (stage)
	{
	case RunStage::INIT:
	{
		if (!ipc_.isSet())
		{
			APLOG_ERROR << "EmergencyLandingPlanner: IPC Missing.";
			return true;
		}

		if (!scheduler_.isSet())
		{
			APLOG_ERROR << "EmergencyLandingPlanner: Scheduler Missing.";
			return true;
		}

		if (!maneuverPlanner_.isSet())
		{
			APLOG_ERROR << "EmergencyLandingPlanner: Maneuver Planner Missing.";
			return true;
		}

		break;
	}
	case RunStage::NORMAL:
	{
		auto ipc = ipc_.get();

		if (!ipc)
		{
			APLOG_ERROR << "EmergencyLandingPlanner: IPC Missing.";
			return true;
		}

		sensorDataSubscription_ = ipc->subscribeOnSharedMemory<SensorData>("sensor_data",
				std::bind(&EmergencyLandingPlanner::onSensorData, this, std::placeholders::_1));

		if (!sensorDataSubscription_.connected())
		{
			APLOG_ERROR << "EmergencyLandingPlanner: Sensor Data Subscription Missing.";
			return true;
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

std::vector<EmergencyLandingParameter>
EmergencyLandingPlanner::getEmergencyLandingParameters() const
{
	return landingParameters_;
}

void
EmergencyLandingPlanner::setEmergencyLandingParameters(
		std::vector<EmergencyLandingParameter> landingParameters)
{
	landingParameters_ = landingParameters;
}

void
EmergencyLandingPlanner::onSensorData(const SensorData& sensorData)
{
	std::unique_lock<std::mutex> lock(sensorDataMutex_);
	if (!sensorData_.autopilotActive && sensorData.autopilotActive)
	{
		onFault(true);
	}
	else if (sensorData_.autopilotActive && !sensorData.autopilotActive)
	{
		onFault(false);
	}

	sensorData_ = sensorData;
	lock.unlock();
}

void
EmergencyLandingPlanner::onFault(bool fault)
{
	if (fault)
	{
		double period = landingParameters_[0].planningParameter.period;

		auto scheduler = scheduler_.get();

		if (!scheduler)
		{
			APLOG_ERROR << "EmergencyLandingPlanner: Scheduler Missing.";
			return;
		}

		landingParameters_[0].schedulingEvent = scheduler->schedule(
				std::bind(&EmergencyLandingPlanner::calculateEmergencyLandingPlan, this,
						landingParameters_[0]), Milliseconds(period), Milliseconds(period));
	}
	else
	{
		auto maneuverPlanner = maneuverPlanner_.get();

		if (!maneuverPlanner)
		{
			APLOG_ERROR << "EmergencyLandingPlanner: Maneuver Planner Missing.";
			return;
		}

		landingParameters_[0].schedulingEvent.cancel();
		maneuverPlanner->cancelEmergencyPlan();
	}
}

EmergencyLandingStatus
EmergencyLandingPlanner::getEmergencyLandingStatus() const
{
	std::unique_lock<std::mutex> lock(sensorDataMutex_);
	SensorData sensorData = sensorData_;
	lock.unlock();

	EmergencyLandingStatus landingStatus;

	landingStatus.position = sensorData.position;
	landingStatus.velocity = sensorData.velocity;
	landingStatus.airSpeed = sensorData.airSpeed;
	landingStatus.climbAngle = sensorData.attitude[1] - sensorData.angleOfAttack;
	landingStatus.yawAngle = sensorData.attitude[2];
	landingStatus.angleOfSideslip = sensorData.angleOfSideslip;

	return landingStatus;
}

void
EmergencyLandingPlanner::calculateEmergencyLandingPlan(EmergencyLandingParameter& landingParameter)
{
	double velocityDelta = 0;
	double minimumCost = 0;
	double currentCost = 0;
	EmergencyLandingPhases currentPhase;
	EmergencyLandingPlan landingPlan;
	EmergencyLandingStatus initialLandingStatus;
	EmergencyLandingStatus currentLandingStatus;

	APLOG_DEBUG << "EmergencyLandingPlanner: "
			<< EnumMap<EmergencyLandingPhases>::convert(landingParameter.phase) << " Phase.";

	initialLandingStatus = getEmergencyLandingStatus();

	velocityDelta = landingParameter.planningParameter.acceleration
			* landingParameter.planningParameter.periodSecond
			* (initialLandingStatus.climbAngle
					- landingParameter.planningParameter.glidingClimbAngle)
			/ fabs(landingParameter.planningParameter.glidingClimbAngle);

	currentLandingStatus.position.x() = initialLandingStatus.position.x()
			+ initialLandingStatus.velocity.x()
			- (velocityDelta / 2) * landingParameter.planningParameter.periodSecond
					* cos(initialLandingStatus.climbAngle) * cos(initialLandingStatus.yawAngle);

	currentLandingStatus.position.y() = initialLandingStatus.position.y()
			+ initialLandingStatus.velocity.y()
			- (velocityDelta / 2) * landingParameter.planningParameter.periodSecond
					* cos(initialLandingStatus.climbAngle) * sin(initialLandingStatus.yawAngle);

	currentLandingStatus.position.z() = initialLandingStatus.position.z()
			+ initialLandingStatus.velocity.z()
			- (velocityDelta / 2) * landingParameter.planningParameter.periodSecond
					* sin(initialLandingStatus.climbAngle);

	currentLandingStatus.airSpeed = initialLandingStatus.airSpeed - velocityDelta;

	currentLandingStatus.climbAngle = initialLandingStatus.climbAngle;

	currentLandingStatus.yawAngle = atan2(sin(initialLandingStatus.yawAngle),
			cos(initialLandingStatus.yawAngle));

	currentLandingStatus.angleOfSideslip = initialLandingStatus.angleOfSideslip;

	auto evaluation = evaluateCost(landingParameter, currentLandingStatus);

	minimumCost = evaluation.first;

	landingPlan.climbAngle = initialLandingStatus.climbAngle;
	landingPlan.yawRate = 0;

	for (double i = -1; i <= 1; i += 0.1)
	{
		for (double j = -0.5; j <= 0.5; j += 0.1)
		{
			velocityDelta = landingParameter.planningParameter.acceleration
					* landingParameter.planningParameter.periodSecond
					* (initialLandingStatus.climbAngle
							+ i * landingParameter.planningParameter.climbRate
									* landingParameter.planningParameter.periodSecond / 2
							- landingParameter.planningParameter.glidingClimbAngle)
					/ fabs(landingParameter.planningParameter.glidingClimbAngle);

			currentLandingStatus.position.x() =
					initialLandingStatus.position.x() + initialLandingStatus.velocity.x()
							- (velocityDelta / 2) * landingParameter.planningParameter.periodSecond
									* cos(
											initialLandingStatus.climbAngle
													+ i
															* landingParameter.planningParameter.climbRate
															* landingParameter.planningParameter.periodSecond
															/ 2)
									* cos(
											initialLandingStatus.yawAngle
													+ j * landingParameter.planningParameter.yawRate
															* landingParameter.planningParameter.periodSecond
															/ 2);

			currentLandingStatus.position.y() =
					initialLandingStatus.position.y() + initialLandingStatus.velocity.y()
							- (velocityDelta / 2) * landingParameter.planningParameter.periodSecond
									* cos(
											initialLandingStatus.climbAngle
													+ i
															* landingParameter.planningParameter.climbRate
															* landingParameter.planningParameter.periodSecond
															/ 2)
									* sin(
											initialLandingStatus.yawAngle
													+ j * landingParameter.planningParameter.yawRate
															* landingParameter.planningParameter.periodSecond
															/ 2);

			currentLandingStatus.position.z() =
					initialLandingStatus.position.z() + initialLandingStatus.velocity.z()
							- (velocityDelta / 2) * landingParameter.planningParameter.periodSecond
									* sin(
											initialLandingStatus.climbAngle
													+ i
															* landingParameter.planningParameter.climbRate
															* landingParameter.planningParameter.periodSecond
															/ 2);

			currentLandingStatus.airSpeed = initialLandingStatus.airSpeed - velocityDelta;

			currentLandingStatus.climbAngle = initialLandingStatus.climbAngle
					+ i * landingParameter.planningParameter.climbRate
							* landingParameter.planningParameter.periodSecond;

			currentLandingStatus.yawAngle = atan2(
					sin(
							initialLandingStatus.yawAngle
									+ j * landingParameter.planningParameter.yawRate
											* landingParameter.planningParameter.periodSecond),
					cos(
							initialLandingStatus.yawAngle
									+ j * landingParameter.planningParameter.yawRate
											* landingParameter.planningParameter.periodSecond));

			currentLandingStatus.angleOfSideslip = initialLandingStatus.angleOfSideslip;

			auto evaluation = evaluateCost(landingParameter, currentLandingStatus);

			currentCost = evaluation.first;
			currentPhase = evaluation.second;

			if (currentCost < minimumCost)
			{
				minimumCost = currentCost;
				landingParameter.phase = currentPhase;

				landingPlan.climbAngle = initialLandingStatus.climbAngle
						+ i * landingParameter.planningParameter.climbRate
								* landingParameter.planningParameter.periodSecond;
				landingPlan.yawRate = j * landingParameter.planningParameter.yawRate;
			}
		}
	}

	publishEmergencyLandingPlan(landingParameter, landingPlan);
}

void
EmergencyLandingPlanner::publishEmergencyLandingPlan(
		const EmergencyLandingParameter& landingParameter, const EmergencyLandingPlan& landingPlan)
{
	Override override;

	std::pair<ControllerTargets, double> climbAngleOverride = std::make_pair(
			ControllerTargets::CLIMB_ANGLE, landingPlan.climbAngle);
	std::pair<ControllerTargets, double> yawRateOverride = std::make_pair(
			ControllerTargets::YAW_RATE, landingPlan.yawRate);
	std::pair<PIDs, double> rudderOverride = std::make_pair(PIDs::RUDDER,
			std::fabs(boundAngleRad(landingParameter.approachingParameter.yawAngle)));

	override.controllerTarget.insert(climbAngleOverride);
	override.controllerTarget.insert(yawRateOverride);
	override.pid.insert(rudderOverride);

	auto maneuverPlanner = maneuverPlanner_.get();

	if (!maneuverPlanner)
	{
		APLOG_ERROR << "EmergencyLandingPlanner: Maneuver Planner Missing.";
		return;
	}

	maneuverPlanner->publishEmergencyPlan(override);
}

std::pair<double, EmergencyLandingPhases>
EmergencyLandingPlanner::evaluateCost(const EmergencyLandingParameter& landingParameter,
		const EmergencyLandingStatus& landingStatus)
{
	double targetPositionX = 0;
	double targetPositionY = 0;
	double yawAngleDelta = 0;
	double carrotChasingRadius = 0;
	double carrotChasingRadiusU = 0;
	double carrotChasingThetaU = 0;
	double carrotChasingBeta = 0;
	double loiterCenterX = 0;
	double loiterCenterY = 0;
	double approachDistanceLoiter = 0;
	double approachDistance = 0;
	double approachAngle = 0;
	double lineTheta = 0;
	double cost = INFINITY;
	EmergencyLandingPhases phase = landingParameter.phase;

	loiterCenterX = landingParameter.approachingParameter.position.x()
			+ landingParameter.searchingParameter.loiterRadiusOffset
					* landingParameter.searchingParameter.loiterRadius
					* cos(landingParameter.approachingParameter.yawAngle - M_PI);

	loiterCenterY = landingParameter.approachingParameter.position.y()
			+ landingParameter.searchingParameter.loiterRadiusOffset
					* landingParameter.searchingParameter.loiterRadius
					* sin(landingParameter.approachingParameter.yawAngle - M_PI);

	approachDistanceLoiter = sqrt(
			pow((landingStatus.position.x() - loiterCenterX), 2)
					+ pow((landingStatus.position.y() - loiterCenterY), 2));

	approachDistance = sqrt(
			pow((landingStatus.position.x() - landingParameter.approachingParameter.position.x()),
					2)
					+ pow(
							(landingStatus.position.y()
									- landingParameter.approachingParameter.position.y()), 2));

	approachAngle = -atan2(
			landingStatus.position.z() - landingParameter.approachingParameter.position.z(),
			approachDistance);

	lineTheta = atan2(landingStatus.position.y() - loiterCenterY,
			landingStatus.position.x() - loiterCenterX);

	if (landingStatus.position.z() < landingParameter.searchingParameter.approachAltitude)
	{
		cost = 0;

		carrotChasingRadiusU =
				sqrt(
						pow(
								loiterCenterX
										+ landingParameter.searchingParameter.loiterRadius
												* cos(
														landingParameter.approachingParameter.yawAngle
																- M_PI)
										- landingStatus.position.x(), 2)
								+ pow(
										loiterCenterY
												+ landingParameter.searchingParameter.loiterRadius
														* sin(
																landingParameter.approachingParameter.yawAngle
																		- M_PI)
												- landingStatus.position.y(), 2));

		carrotChasingThetaU = atan2(
				landingStatus.position.y() - loiterCenterY
						- landingParameter.searchingParameter.loiterRadius
								* sin(landingParameter.approachingParameter.yawAngle - M_PI),
				landingStatus.position.x() - loiterCenterX
						- landingParameter.searchingParameter.loiterRadius
								* cos(landingParameter.approachingParameter.yawAngle - M_PI));

		carrotChasingBeta = landingParameter.approachingParameter.yawAngle - carrotChasingThetaU;

		carrotChasingRadius = sqrt(
				pow(carrotChasingRadiusU, 2)
						- pow(carrotChasingRadiusU * sin(carrotChasingBeta), 2));

		targetPositionX = loiterCenterX
				+ landingParameter.searchingParameter.loiterRadius
						* cos(landingParameter.approachingParameter.yawAngle - M_PI)
				+ (carrotChasingRadius + landingParameter.searchingParameter.lineDelta)
						* cos(landingParameter.approachingParameter.yawAngle);

		targetPositionY = loiterCenterY
				+ landingParameter.searchingParameter.loiterRadius
						* sin(landingParameter.approachingParameter.yawAngle - M_PI)
				+ (carrotChasingRadius + landingParameter.searchingParameter.lineDelta)
						* sin(landingParameter.approachingParameter.yawAngle);

		yawAngleDelta = atan2(targetPositionY - landingStatus.position.y(),
				targetPositionX - landingStatus.position.x());

		cost += std::min(fabs(landingStatus.yawAngle - yawAngleDelta),
				fabs(fabs(landingStatus.yawAngle - yawAngleDelta) - 2 * M_PI))
				/ landingParameter.approachingParameter.yawRate;

		if (approachDistance < landingParameter.searchingParameter.loiterRadius)
		{
			cost += fabs(landingStatus.airSpeed - landingParameter.approachingParameter.velocity)
					/ landingParameter.searchingParameter.acceleration;

			cost += fabs(
					landingStatus.climbAngle - landingParameter.approachingParameter.climbAngle)
					/ landingParameter.searchingParameter.climbRate;
		}
		else
		{
			cost += fabs(
					landingStatus.airSpeed - landingParameter.planningParameter.glidingVelocity)
					/ landingParameter.searchingParameter.acceleration;

			cost += fabs(approachAngle - landingParameter.approachingParameter.climbAngle)
					/ landingParameter.searchingParameter.climbRate;
		}

		phase = EmergencyLandingPhases::APPROACHING;
	}
	else if (approachDistanceLoiter > landingParameter.searchingParameter.cruiseRadius
			&& landingParameter.phase != EmergencyLandingPhases::DESCENDING
			&& landingParameter.phase != EmergencyLandingPhases::APPROACHING)
	{
		cost = 0;

		yawAngleDelta = atan2(loiterCenterY - landingStatus.position.y(),
				loiterCenterX - landingStatus.position.x());

		for (const auto& it : landingParameter.approachingParameter.obstacles)
		{
			cost += exp(
					it.second.z()
							/ sqrt(
									pow((landingStatus.position.x() - it.second.x()), 2)
											+ pow((landingStatus.position.y() - it.second.y()), 2)))
					- 1;
		}

		cost += std::min(fabs(landingStatus.yawAngle - yawAngleDelta),
				fabs(fabs(landingStatus.yawAngle - yawAngleDelta) - 2 * M_PI))
				/ landingParameter.searchingParameter.yawRate;

		cost += fabs(landingStatus.airSpeed - landingParameter.planningParameter.glidingVelocity)
				/ landingParameter.searchingParameter.acceleration;

		phase = EmergencyLandingPhases::CRUISING;
	}
	else if (landingParameter.phase != EmergencyLandingPhases::APPROACHING)
	{
		cost = 0;

		targetPositionX = loiterCenterX
				+ landingParameter.searchingParameter.loiterRadius
						* cos(lineTheta + landingParameter.searchingParameter.loiterLambda);

		targetPositionY = loiterCenterY
				+ landingParameter.searchingParameter.loiterRadius
						* sin(lineTheta + landingParameter.searchingParameter.loiterLambda);

		yawAngleDelta = atan2(targetPositionY - landingStatus.position.y(),
				targetPositionX - landingStatus.position.x());

		cost += std::min(fabs(landingStatus.yawAngle - yawAngleDelta),
				fabs(fabs(landingStatus.yawAngle - yawAngleDelta) - 2 * M_PI))
				/ landingParameter.searchingParameter.yawRate;

		cost += fabs(landingStatus.airSpeed - landingParameter.planningParameter.glidingVelocity)
				/ landingParameter.searchingParameter.acceleration;

		phase = EmergencyLandingPhases::DESCENDING;
	}

	return std::make_pair(cost, phase);
}
