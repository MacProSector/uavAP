/*
 * EmergencyLandingModel.h
 *
 *  Created on: Feb 22, 2021
 *      Author: simonyu
 */

#ifndef UAVAP_MISSIONCONTROL_EMERGENCYPLANNER_EMERGENCYLANDINGPLANNER_EMERGENCYLANDINGMODEL_H_
#define UAVAP_MISSIONCONTROL_EMERGENCYPLANNER_EMERGENCYLANDINGPLANNER_EMERGENCYLANDINGMODEL_H_

#include "uavAP/Core/LinearAlgebra.h"
#include "uavAP/MissionControl/EmergencyPlanner/EmergencyLandingPlanner/EmergencyLandingStatus.h"

class EmergencyLandingModel
{
public:

	void
	configure(const double& period, const EmergencyLandingStatus& landingStatus);

	void
	update(const double& aileronDeflection, const double& elevatorDeflection);

	EmergencyLandingStatus
	getLandingStatus() const;

private:

	double period_;

	EmergencyLandingStatus landingStatus_;

	Vector5 internalStatesLongitudinal_;
	Vector4 internalStatesLateral_;

	Matrix5 matrixALongitudinal_;
	Vector5 vectorBLongitudinal_;
	RowVector5 vectorCLongitudinal_;

	Matrix4 matrixALateral_;
	Vector4 vectorBLateral_;
	RowVector4 vectorCLateral_;
};

#endif /* UAVAP_MISSIONCONTROL_EMERGENCYPLANNER_EMERGENCYLANDINGPLANNER_EMERGENCYLANDINGMODEL_H_ */
