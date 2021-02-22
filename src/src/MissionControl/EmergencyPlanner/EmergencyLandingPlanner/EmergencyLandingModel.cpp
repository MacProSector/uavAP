/*
 * EmergencyLandingModel.cpp
 *
 *  Created on: Feb 22, 2021
 *      Author: simonyu
 */

#include "uavAP/MissionControl/EmergencyPlanner/EmergencyLandingPlanner/EmergencyLandingModel.h"

void
EmergencyLandingModel::configure(const double& period, const EmergencyLandingStatus& landingStatus)
{
	period_ = period;

	landingStatus_ = landingStatus;

	internalStatesLongitudinal_ << 0, 0, 0, 0, 0;
	internalStatesLateral_ << 0, 0, 0, 0;

	matrixALongitudinal_ << 4.81656187883257, -2.31933707527225, 1.11655190344296, -0.537379151488548, 0.413702202939676, 4, 0, 0, 0, 0, 0, 2, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0.250000000000000, 0;
	vectorBLongitudinal_ << 0.0156250000000000, 0, 0, 0, 0;
	vectorCLongitudinal_ << 0.00355238503265512, 0.00164787850126744, -0.00252811991830636, 0.000860387264760633, 0.00159910056333754;

	matrixALateral_ << 3.86328199220340, -2.79635360696289, 1.79772016349541, -0.866015128123800, 2, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0.500000000000000, 0;
	vectorBLateral_ << 0.125000000000000, 0, 0, 0;
	vectorCLateral_ << 0.0287786320892559, -0.0148061580263057, -0.0131580831960478, 0.0271947975723820;
}

void
EmergencyLandingModel::update(const double& aileronDeflection, const double& elevatorDeflection)
{
	// Position
	landingStatus_.position.x() += period_ * landingStatus_.airSpeed
			* cos(landingStatus_.climbAngle) * cos(landingStatus_.yawAngle);

	landingStatus_.position.y() += period_ * landingStatus_.airSpeed
			* cos(landingStatus_.climbAngle) * sin(landingStatus_.yawAngle);

	landingStatus_.position.z() += period_ * landingStatus_.airSpeed
			* sin(landingStatus_.climbAngle);

	// Longitudinal dynamics
	landingStatus_.airSpeed = -1 * (period_ / 0.174) * vectorCLongitudinal_
			* internalStatesLongitudinal_ + landingStatus_.airSpeed - (period_ / 2);

	landingStatus_.climbAngle = vectorCLongitudinal_ * internalStatesLongitudinal_;

	internalStatesLongitudinal_ = matrixALongitudinal_ * internalStatesLongitudinal_
			+ vectorBLongitudinal_ * aileronDeflection;

	// Lateral dynamics
	landingStatus_.yawAngle = 0.2 * period_ * vectorCLateral_ * internalStatesLateral_
			+ landingStatus_.yawAngle;

	internalStatesLateral_ = matrixALateral_ * internalStatesLateral_
			+ vectorBLateral_ * elevatorDeflection;
}

EmergencyLandingStatus
EmergencyLandingModel::getLandingStatus() const
{
	return landingStatus_;
}
