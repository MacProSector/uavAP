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
 * Publisher.h
 *
 *  Created on: Jul 18, 2017
 *      Author: mircot
 */

#ifndef UAVAP_CORE_IPC_PUBLISHER_H_
#define UAVAP_CORE_IPC_PUBLISHER_H_

#include <uavAP/Core/IPC/detail/IPublisherImpl.h>
#include <uavAP/Core/Object/ObjectHandle.h>
#include "uavAP/Core/Logging/APLogger.h"

#include <memory>
#include <vector>

template<typename Pub>
class Publisher
{
public:

	Publisher() = default;

	Publisher(std::shared_ptr<IPublisherImpl> impl, std::function<Packet(const Pub&)> forwarding);

	void
	publish(const Pub& obj);

private:

	std::weak_ptr<IPublisherImpl> publisherImpl_;

	std::function<Packet(const Pub&)> forwarding_;
};

template<typename Pub>
inline
Publisher<Pub>::Publisher(std::shared_ptr<IPublisherImpl> impl, std::function<Packet(const Pub&)> forwarding) :

		publisherImpl_(impl), forwarding_(forwarding)
{
}

template<typename Pub>
inline void
Publisher<Pub>::publish(const Pub& obj)
{
	if (auto impl = publisherImpl_.lock())
	{
		impl->publish(forwarding_(obj));
	}
	else
	{
		APLOG_ERROR << "Cannot publish because Impl is not set.";
	}
}

#endif /* UAVAP_CORE_IPC_PUBLISHER_H_ */
