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
 * @file FileFromArchive.cpp
 * @date Nov 13, 2018
 * @author Mirco Theile, mirco.theile@tum.de
 * @brief
 */
#include "uavAP/Core/DataPresentation/APDataPresentation/FileFromArchive.h"
#include "uavAP/Core/DataPresentation/APDataPresentation/BasicSerialization.h"
#include <iostream>
#include <sstream>
#include <fstream>

FileFromArchive::FileFromArchive(std::ifstream& file, const ArchiveOptions& opts) :
		options_(opts), file_(file)
{
}

void
FileFromArchive::setOptions(const ArchiveOptions& opts)
{
	options_ = opts;
}

FileFromArchive&
FileFromArchive::operator >>(double& doub)
{
	if (options_.compressDouble_)
	{
		float flo;
		dp::load(*this, reinterpret_cast<char*>(&flo), sizeof(float));
		doub = static_cast<double>(flo);
	}
	else
	{
		dp::load(*this, reinterpret_cast<char*>(&doub), sizeof(double));
	}
	return *this;
}

void
FileFromArchive::read(char* val, unsigned long bytes)
{
	file_.read(val, bytes);
}
