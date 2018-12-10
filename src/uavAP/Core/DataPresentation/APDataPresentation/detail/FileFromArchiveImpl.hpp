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
 * @file FileFromArchiveImpl.hpp
 * @date Nov 13, 2018
 * @author Mirco Theile, mirco.theile@tum.de
 * @brief
 */

#ifndef UAVAP_CORE_DATAPRESENTATION_APDATAPRESENTATION_DETAIL_FILEFROMARCHIVEIMPL_HPP_
#define UAVAP_CORE_DATAPRESENTATION_APDATAPRESENTATION_DETAIL_FILEFROMARCHIVEIMPL_HPP_
#include "uavAP/Core/DataPresentation/APDataPresentation/FileFromArchive.h"
#include "uavAP/Core/DataPresentation/APDataPresentation/BasicSerialization.h"
#include <iostream>
#include <sstream>
#include <fstream>

template<class Type>
typename std::enable_if<!std::is_base_of<google::protobuf::Message, Type>::value,
		FileFromArchive>::type&
FileFromArchive::operator >>(Type& val)
{
	dp::serialize<FileFromArchive, Type>(*this, val);
	return *this;
}

template<class Type>
typename std::enable_if<std::is_base_of<google::protobuf::Message, Type>::value,
		FileFromArchive>::type&
FileFromArchive::operator >>(Type& message)
{
	uint16_t size;
	*this >> size;
	std::string input(' ', size);
	read(&input[0], size);
	message.ParseFromString(input);
	return *this;
}

template<class Type>
void
FileFromArchive::operator &(Type& val)
{
	*this >> val;
}

template<class Type>
FileFromArchive&
FileFromArchive::operator <<(Type& val)
{
	return *this;
}



#endif /* UAVAP_CORE_DATAPRESENTATION_APDATAPRESENTATION_DETAIL_FILEFROMARCHIVEIMPL_HPP_ */
