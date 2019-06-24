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
 * L1AdaptiveCascade.h
 *
 *  Created on: Jun 3, 2019
 *      Author: simonyu
 */

#ifndef UAVAP_FLIGHTCONTROL_CONTROLLER_ADAPTIVECONTROLLER_L1ADAPTIVECONTROLLER_DETAIL_L1ADAPTIVECASCADE_H_
#define UAVAP_FLIGHTCONTROL_CONTROLLER_ADAPTIVECONTROLLER_L1ADAPTIVECONTROLLER_DETAIL_L1ADAPTIVECASCADE_H_

#include "uavAP/FlightControl/Controller/ControllerConstraint.h"
#include "uavAP/FlightControl/Controller/AdaptiveControlEnvironment/AdaptiveControlEnvironment.h"
#include "uavAP/FlightControl/Controller/AdaptiveController/IAdaptiveCascade.h"
#include "uavAP/FlightControl/Controller/AdaptiveController/AdaptiveMapping.h"
#include "uavAP/FlightControl/Controller/AdaptiveController/L1AdaptiveController/L1AdaptiveParameter.h"

class SensorData;
class ControllerTarget;
class ControllerOutput;
class Override;

class L1AdaptiveCascade: public IAdaptiveCascade
{
public:

	L1AdaptiveCascade(SensorData& sensorData, ControllerTarget& controllerTarget,
			ControllerOutput& controllerOutput);

	bool
	configure(const boost::property_tree::ptree& config) override;

	PIDStati
	getPIDStatus() const override;

	void
	evaluate() override;

	bool
	tunePID(PIDs pid, const PIDParameter& parameter) override;

	void
	overrideCascade(const Override& override);

private:

	bool
	configureAdaptive(const boost::property_tree::ptree& config);

	bool
	configurePID(const boost::property_tree::ptree& config);

	bool
	configureSaturation(const boost::property_tree::ptree& config);

	template<typename ENUM, typename PARAM>
	PARAM
	getParameter(const std::map<ENUM, PARAM>& parameterMap, const ENUM& parameterEnum);

	void
	setSaturation(const ControllerConstraints& saturationEnum,
			std::shared_ptr<Saturation<double>> saturation, double& hardSaturation,
			double& softSaturation);

	void
	createCascade();

	void
	calculateControl();

	SensorData& sensorData_;
	ControllerTarget& controllerTarget_;
	ControllerOutput& controllerOutput_;
	AdaptiveControlEnvironment controlEnvironment_;

	std::map<PIDs, std::shared_ptr<PID<double>>> pids_;
	std::map<ControllerOutputs, std::shared_ptr<Output<double, double>>> outputs_;
	std::map<ControllerOutputsWaveforms, std::shared_ptr<Output<double, double>>> outputWaveforms_;
	std::map<ControllerConstraints, std::shared_ptr<Saturation<double>>> saturations_;

	PitchL1AdaptiveParameter pitchAdaptiveParameter_;
	std::map<PIDs, PIDParameter> pidParameters_;

	double beta_;
	double rollTarget_;
};

template<typename ENUM, typename PARAM>
inline PARAM
L1AdaptiveCascade::getParameter(const std::map<ENUM, PARAM>& parameterMap,
		const ENUM& parameterEnum)
{
	if (auto parameterPair = findInMap(parameterMap, parameterEnum))
	{
		return parameterPair->second;
	}
	else
	{
		return PARAM();
	}
}

#endif /* UAVAP_FLIGHTCONTROL_CONTROLLER_ADAPTIVECONTROLLER_L1ADAPTIVECONTROLLER_DETAIL_L1ADAPTIVECASCADE_H_ */
