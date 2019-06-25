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

template<typename INPUT_TRIM, typename OUTPUT_TRIM, typename INPUT_GAIN, typename TARGET_GAIN,
		typename PREDICTOR_GAIN, typename ADAPTIVE_GAIN, typename CONTROL_LAW_MATRIX_A,
		typename CONTROL_LAW_MATRIX_B, typename CONTROL_LAW_MATRIX_C, typename CONTROL_LAW_MATRIX_D,
		typename CONTROL_LAW_STATE, typename PREDICTOR_MATRIX_A, typename PREDICTOR_MATRIX_B,
		typename PREDICTOR_MATRIX_C, typename PREDICTOR_MATRIX_D, typename PREDICTOR_STATE>
struct L1AdaptiveParameter
{
	INPUT_TRIM inputTrim;
	OUTPUT_TRIM outputTrim;
	INPUT_GAIN inputGain;
	TARGET_GAIN targetGain;
	PREDICTOR_GAIN predictorGain;
	ADAPTIVE_GAIN adaptiveGain;
	CONTROL_LAW_MATRIX_A controlLawMatrixA;
	CONTROL_LAW_MATRIX_B controlLawMatrixB;
	CONTROL_LAW_MATRIX_C controlLawMatrixC;
	CONTROL_LAW_MATRIX_D controlLawMatrixD;
	CONTROL_LAW_STATE controlLawState;
	PREDICTOR_MATRIX_A predictorMatrixA;
	PREDICTOR_MATRIX_B predictorMatrixB;
	PREDICTOR_MATRIX_C predictorMatrixC;
	PREDICTOR_MATRIX_D predictorMatrixD;
	PREDICTOR_STATE predictorState;

	bool
	configure(const boost::property_tree::ptree& config)
	{
		PropertyMapper pm(config);

		pm.addMatrix<INPUT_TRIM, double>("input_trim", inputTrim, true);
		pm.addMatrix<OUTPUT_TRIM, double>("output_trim", outputTrim, true);
		pm.addMatrix<INPUT_GAIN, double>("input_gain", inputGain, true);
		pm.addMatrix<TARGET_GAIN, double>("target_gain", targetGain, true);
		pm.addMatrix<PREDICTOR_GAIN, double>("predictor_gain", predictorGain, true);
		pm.addMatrix<ADAPTIVE_GAIN, double>("adaptive_gain", adaptiveGain, true);
		pm.addMatrix<CONTROL_LAW_MATRIX_A, double>("control_law_matrix_a", controlLawMatrixA, true);
		pm.addMatrix<CONTROL_LAW_MATRIX_B, double>("control_law_matrix_b", controlLawMatrixB, true);
		pm.addMatrix<CONTROL_LAW_MATRIX_C, double>("control_law_matrix_c", controlLawMatrixC, true);
		pm.addMatrix<CONTROL_LAW_MATRIX_D, double>("control_law_matrix_d", controlLawMatrixD, true);
		pm.addMatrix<CONTROL_LAW_STATE, double>("control_law_state", controlLawState, true);
		pm.addMatrix<PREDICTOR_MATRIX_A, double>("predictor_matrix_a", predictorMatrixA, true);
		pm.addMatrix<PREDICTOR_MATRIX_B, double>("predictor_matrix_b", predictorMatrixB, true);
		pm.addMatrix<PREDICTOR_MATRIX_C, double>("predictor_matrix_c", predictorMatrixC, true);
		pm.addMatrix<PREDICTOR_MATRIX_D, double>("predictor_matrix_d", predictorMatrixD, true);
		pm.addMatrix<PREDICTOR_STATE, double>("predictor_state", predictorState, true);

		return pm.map();
	}
};

using RollL1AdaptiveParameter = L1AdaptiveParameter<Vector4, Vector2, Matrix2x4, Matrix2, Matrix4x2,
Matrix4x2, Matrix22, Matrix22x4, Matrix2x22, Matrix2x4, Vector22, Matrix4, Matrix4, Matrix2x4, Matrix2x4, Vector4>;

using PitchL1AdaptiveParameter = L1AdaptiveParameter<Vector3, Scalar, RowVector3, Scalar, Vector3,
Vector3, Matrix4, Matrix4x3, RowVector4, RowVector3, Vector4, Matrix3, Matrix3, RowVector3, RowVector3, Vector3>;

#endif /* UAVAP_FLIGHTCONTROL_CONTROLLER_ADAPTIVECONTROLLER_L1ADAPTIVECONTROLLER_L1ADAPTIVEPARAMETER_H_ */
