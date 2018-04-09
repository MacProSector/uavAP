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
 * @file BinaryToArchive.h
 * @brief Defines the BinaryRoArchive
 * @date Aug 22, 2017
 * @author Mirco Theile, mircot@illinois.edu
 */

#ifndef UAVAP_CORE_DATAPRESENTATION_APDATAPRESENTATION_BINARYTOARCHIVE_H_
#define UAVAP_CORE_DATAPRESENTATION_APDATAPRESENTATION_BINARYTOARCHIVE_H_
#include <google/protobuf/message.h>
#include <string>

/**
 * @brief Archive wrapper around a string to serialize data
 */
class BinaryToArchive
{
public:

	/**
	 * @brief Constructor wrapping around a string
	 * @param str String to be wrapped around and filled with serialization
	 */
	BinaryToArchive(std::string& str);

	/**
	 * @brief Append the string with length characters from c
	 * @param c Characters to be appended
	 * @param length Number of characters to be appended
	 */
	void
	append(const char* c, size_t length);

	/**
	 * @brief Flush in operator for non protobuf objects.
	 *
	 * Serializes the object that is not a protobuf object.
	 * Uses serialization from BasicSerialization.h and CustomSerialization.h
	 * @param cval Data to be serialized
	 * @return The archive itself
	 */
	template<typename Type>
	typename std::enable_if<!std::is_base_of<google::protobuf::Message, Type>::value,
			BinaryToArchive>::type&
	operator <<(const Type& cval);

	/**
	 * @brief Flush in operator for protobuf objects.
	 *
	 * Uses the serialization function of a protobuf object to create a string from that object.
	 * Then appends the current string with the serialization of the object.
	 * @param message Protobuf object
	 * @return The archive itself
	 */
	template<class Type>
	typename std::enable_if<std::is_base_of<google::protobuf::Message, Type>::value, BinaryToArchive>::type&
	operator <<(const Type& message);

	/**
	 * @brief Operator & for flush in
	 * @param val Data to be flushed in
	 */
	template<class Type>
	void
	operator &(const Type& val);

	/**
	 * @brief Flush out operator. Does nothing
	 * @param val Data to be flushed out.
	 * @return The archive itself
	 */
	template<class Type>
	BinaryToArchive&
	operator >>(const Type& val);

private:

	std::string& string_; //!< String that is wrapped around and filled with serialized data.
};


#endif /* UAVAP_CORE_DATAPRESENTATION_APDATAPRESENTATION_BINARYTOARCHIVE_H_ */
