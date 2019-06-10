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
 * PIDParameter.h
 *
 *  Created on: Jun 10, 2019
 *      Author: simonyu
 */

#ifndef UAVAP_FLIGHTCONTROL_CONTROLLER_PIDCONTROLLER_PIDPARAMETER_H_
#define UAVAP_FLIGHTCONTROL_CONTROLLER_PIDCONTROLLER_PIDPARAMETER_H_

#include <cmath>
#include <boost/property_tree/ptree.hpp>

struct PIDParameter
{
	double kp, ki, kd, imax, ff;

	PIDParameter() :
			kp(0), ki(0), kd(0), imax(INFINITY), ff(0)
	{
	}

	bool
	configure(const boost::property_tree::ptree& p)
	{
		PropertyMapper pm(p);
		pm.add<double>("kp", kp, true);
		pm.add<double>("ki", ki, false);
		pm.add<double>("kd", kd, false);
		pm.add<double>("imax", imax, false);
		pm.add<double>("ff", ff, false);
		return pm.map();
	}
};

namespace dp
{
template<class Archive, typename Type>
inline void
serialize(Archive& ar, PIDParameter& t)
{
	ar & t.kp;
	ar & t.ki;
	ar & t.kd;
	ar & t.imax;
	ar & t.ff;
}
}

#endif /* UAVAP_FLIGHTCONTROL_CONTROLLER_PIDCONTROLLER_PIDPARAMETER_H_ */
