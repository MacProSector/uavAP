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
 *  @file         LinearAlgebra.h
 *  @author  Mirco Theile, mirco.theile@tum.de
 *  @date      23 June 2017
 *  @brief      Vector handling, conversions and rotations
 */

#ifndef UAVAP_CORE_LINEARALGEBRA_H_
#define UAVAP_CORE_LINEARALGEBRA_H_

#define EIGEN_DONT_ALIGN_STATICALLY

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "uavAP/Core/protobuf/messages/Attitudes.pb.h"
#include "uavAP/Core/protobuf/messages/Positions.pb.h"
#include "uavAP/Core/DataPresentation/APDataPresentation/BasicSerialization.h"

using Scalar = Eigen::Matrix<double, 1, 1>;
using Vector2 = Eigen::Vector2d;
using Vector3 = Eigen::Vector3d;
using Vector4 = Eigen::Vector4d;
using Vector5 = Eigen::Matrix<double, 5, 1>;
using Vector20 = Eigen::Matrix<double, 20, 1>;
using Vector22 = Eigen::Matrix<double, 22, 1>;
using Vector25 = Eigen::Matrix<double, 25, 1>;
using Vector28 = Eigen::Matrix<double, 28, 1>;
using VectorX = Eigen::VectorXd;
using RowVector2 = Eigen::RowVector2d;
using RowVector3 = Eigen::RowVector3d;
using RowVector4 = Eigen::RowVector4d;
using RowVector5 = Eigen::Matrix<double, 1, 5>;
using RowVectorX = Eigen::RowVectorXd;
using Matrix2 = Eigen::Matrix2d;
using Matrix3 = Eigen::Matrix3d;
using Matrix4 = Eigen::Matrix4d;
using Matrix5 = Eigen::Matrix<double, 5, 5>;
using Matrix20 = Eigen::Matrix<double, 20, 20>;
using Matrix22 = Eigen::Matrix<double, 22, 22>;
using Matrix25 = Eigen::Matrix<double, 25, 25>;
using Matrix28 = Eigen::Matrix<double, 28, 28>;
using Matrix2x4 = Eigen::Matrix<double, 2, 4>;
using Matrix2x20 = Eigen::Matrix<double, 2, 20>;
using Matrix2x22 = Eigen::Matrix<double, 2, 22>;
using Matrix2x25 = Eigen::Matrix<double, 2, 25>;
using Matrix2x28 = Eigen::Matrix<double, 2, 28>;
using Matrix4x2 = Eigen::Matrix<double, 4, 2>;
using Matrix4x3 = Eigen::Matrix<double, 4, 3>;
using Matrix20x4 = Eigen::Matrix<double, 20, 4>;
using Matrix22x4 = Eigen::Matrix<double, 22, 4>;
using Matrix25x4 = Eigen::Matrix<double, 25, 4>;
using Matrix28x4 = Eigen::Matrix<double, 28, 4>;
using MatrixX = Eigen::MatrixXd;
using MatrixXx3 = Eigen::Matrix<double, Eigen::Dynamic, 3>;
using Rotation2 = Eigen::Rotation2Dd;
using EigenLine = Eigen::ParametrizedLine<double, 3>;
using EigenLine2 = Eigen::ParametrizedLine<double, 2>;
using EigenHyperplane = Eigen::Hyperplane<double, 3, Eigen::DontAlign>;

class PositionENU;

/**
 * @brief Convert protobuf ENUPosition into Eigen::Vector3d
 * @param position Position in ENU
 * @return Position as Eigen::Vector3d
 */
Vector3
toVector(const PositionENU& position);

/**
 * @brief Rotate 2D vector counter clockwise
 * @param vec Vector to be rotated
 * @param rad Rotation angle in radians
 * @return Rotated vector
 */
Vector2
rotate2Drad(const Vector2& vec, const double& rad);

/**
 * @brief Rotate 3D vector counter clockwise
 * @param vector Vector to be rotated
 * @param attitude Euler attitude angles in radians
 * @return Rotated vector
 */
Vector3
rotate3Drad(const Vector3& vector, const AttitudeEuler& attitude);

/**
 * @brief Caculate the Heading from a Vector3 in ENU
 * @param vec Vector3 in ENU
 * @return Heading in radians. North is pi/2, East is 0.
 */
double
headingFromENU(const Vector3& vec);

/**
 * @brief Caculate the Heading from a Vector2 in EN(U)
 * @param vec Vector2 in EN(U)
 * @return Heading in radians. North is pi/2, East is 0.
 */
double
headingFromENU(const Vector2& vec);

/**
 * @brief Get the angle in a range between -PI and PI
 * @param angle Angle in radians
 * @return angle in (-PI, PI]
 */
double
boundAngleRad(double angle);

/**
 * @brief Convert euler angles to quaternion
 * @param euler Vector with [roll, pitch, yaw]
 * @return Quaternion
 */
Eigen::Quaterniond
eulerToQuaternion(const Vector3& euler);

/**
 * @brief Convert quaternion to euler angles
 * @param quaternion
 * @return euler angles [roll, pitch, yaw]
 */
Vector3
quaternionToEuler(const Eigen::Quaterniond& quaternion);

Vector3
directionFromAttitude(const Vector3& att);

Vector3&
degToRadRef(Vector3& vec);

double&
degToRadRef(double& deg);

Vector3&
radToDegRef(Vector3& vec);

double&
radToDegRef(double& rad);

Vector3
degToRad(const Vector3& vec);

double
degToRad(const double& deg);

Vector3
radToDeg(const Vector3& vec);

double
radToDeg(const double& rad);

template<typename TYPE>
TYPE
sign(const TYPE& input);

std::istream&
operator>>(std::istream& is, Vector3& obj);
std::ostream&
operator<<(std::ostream& os, const Vector3& obj);
std::istream&
operator>>(std::istream& is, EigenLine& obj);
std::ostream&
operator<<(std::ostream& os, const EigenLine& obj);
std::istream&
operator>>(std::istream& is, EigenHyperplane& obj);
std::ostream&
operator<<(std::ostream& os, const EigenHyperplane& obj);

template<typename TYPE>
inline TYPE
sign(const TYPE& input)
{
	if (input < 0)
	{
		return TYPE(-1);
	}

	return TYPE(1);
}

namespace dp
{
template<class Archive, typename Type>
inline void
serialize(Archive& ar, Vector3& t)
{
	ar & t[0];
	ar & t[1];
	ar & t[2];
}

template<class Archive, typename Type>
inline void
serialize(Archive& ar, Vector2& t)
{
	ar & t[0];
	ar & t[1];
}

template<class Archive, typename Type>
inline void
serialize(Archive& ar, Eigen::Vector3f& t)
{
	ar & t[0];
	ar & t[1];
	ar & t[2];
}

template<class Archive, typename Type>
void
store(Archive& ar, const EigenLine& t)
{
	ar << t.origin();
	ar << t.direction();
}

template<class Archive, typename Type>
void
load(Archive& ar, EigenLine& t)
{
	Vector3 origin;
	Vector3 direction;
	ar >> origin;
	ar >> direction;
	t = EigenLine(origin, direction);
}

template<class Archive, typename Type>
inline void
serialize(Archive& ar, EigenLine& t)
{
	split(ar, t);
}
}

#endif /* UAVAP_CORE_LINEARALGEBRA_H_ */
