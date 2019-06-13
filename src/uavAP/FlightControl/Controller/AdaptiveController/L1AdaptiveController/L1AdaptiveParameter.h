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
 * L1AdaptiveParameter.h
 *
 *  Created on: Jun 11, 2019
 *      Author: simonyu
 */

#ifndef UAVAP_FLIGHTCONTROL_CONTROLLER_ADAPTIVECONTROLLER_L1ADAPTIVECONTROLLER_L1ADAPTIVEPARAMETER_H_
#define UAVAP_FLIGHTCONTROL_CONTROLLER_ADAPTIVECONTROLLER_L1ADAPTIVECONTROLLER_L1ADAPTIVEPARAMETER_H_

#include <boost/property_tree/ptree.hpp>

#include "uavAP/Core/LinearAlgebra.h"
#include "uavAP/Core/PropertyMapper/PropertyMapper.h"

struct L1AdaptiveParameter
{
	MatrixX inputTrim;
	MatrixX outputTrim;
	MatrixX inputGain;
	MatrixX targetGain;
	MatrixX predictorGain;
	MatrixX adaptiveGain;
	MatrixX controlLawMatrixA;
	MatrixX controlLawMatrixB;
	MatrixX controlLawMatrixC;
	MatrixX controlLawMatrixD;
	MatrixX controlLawState;
	MatrixX predictorMatrixA;
	MatrixX predictorMatrixB;
	MatrixX predictorMatrixC;
	MatrixX predictorMatrixD;
	MatrixX predictorState;

	bool
	configure(const boost::property_tree::ptree& config)
	{
		PropertyMapper pm(config);

		pm.add<double>("input_trim", inputTrim, true);
		pm.add<double>("output_trim", outputTrim, true);
		pm.add<double>("input_gain", inputGain, true);
		pm.add<double>("target_gain", targetGain, true);
		pm.add<double>("predictor_gain", predictorGain, true);
		pm.add<double>("adaptive_gain", adaptiveGain, true);
		pm.add<double>("control_law_matrix_a", controlLawMatrixA, true);
		pm.add<double>("control_law_matrix_b", controlLawMatrixB, true);
		pm.add<double>("control_law_matrix_c", controlLawMatrixC, true);
		pm.add<double>("control_law_matrix_d", controlLawMatrixD, true);
		pm.add<double>("control_law_state", controlLawState, true);
		pm.add<double>("predictor_matrix_a", predictorMatrixA, true);
		pm.add<double>("predictor_matrix_b", predictorMatrixB, true);
		pm.add<double>("predictor_matrix_c", predictorMatrixC, true);
		pm.add<double>("predictor_matrix_d", predictorMatrixD, true);
		pm.add<double>("predictor_state", predictorState, true);

		return pm.map();
	}
};

#endif /* UAVAP_FLIGHTCONTROL_CONTROLLER_ADAPTIVECONTROLLER_L1ADAPTIVECONTROLLER_L1ADAPTIVEPARAMETER_H_ */
