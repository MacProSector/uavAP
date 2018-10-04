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
 *  @file         IComm.h
 *  @author Simon Yu
 *  @date      31 July 2017
 *  @brief      UAV Autopilot Communication Comm Interface Header File
 *
 *  Description
 */

#ifndef UAVAP_COMMUNICATION_COMM_ICOMM_H_
#define UAVAP_COMMUNICATION_COMM_ICOMM_H_

class IComm
{
public:
	virtual
	~IComm() = default;
};

#endif /* UAVAP_COMMUNICATION_COMM_ICOMM_H_ */