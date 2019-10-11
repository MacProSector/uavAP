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
 * ControlElements.h
 *
 *  Created on: Jun 19, 2017
 *      Author: mircot
 */

#ifndef CONTROL_CONTROLELEMENTS_H_
#define CONTROL_CONTROLELEMENTS_H_

#include "uavAP/FlightControl/Controller/ControlElements/IControlElement.h"

namespace Control
{

class Constant: public IControlElement
{
public:
	Constant(FloatingType val);

	FloatingType
	getValue() override;

private:

	FloatingType val_;
};

class Constraint: public IControlElement
{

public:

	Constraint(Element in, FloatingType min, FloatingType max);

	FloatingType
	getValue() override;

	void
	setContraintValue(FloatingType minmax);

	void
	setContraintValue(FloatingType min, FloatingType max);

	void
	overrideContraintValue(FloatingType overrideMinmax);

	void
	overrideContraintValue(FloatingType overrideMin, FloatingType overrideMax);

	void
	disableOverride();

	Element in_;
	FloatingType min_;
	FloatingType max_;
	bool override_;
	FloatingType overrideMin_;
	FloatingType overrideMax_;
};

class Difference: public IControlElement
{
public:

	Difference(Element in1, Element in2);

	FloatingType
	getValue() override;

private:

	Element in1_;
	Element in2_;
};

class Gain: public IControlElement
{

public:

	Gain(Element in, FloatingType gain);

	FloatingType
	getValue() override;

private:

	Element in_;

	FloatingType gain_;

};

class Input: public IControlElement
{
public:

	Input(const FloatingType* in);

	FloatingType
	getValue() override;

private:

	const FloatingType* in_;
};

class Sum: public IControlElement
{
public:

	Sum(Element in1, Element in2);

	FloatingType
	getValue() override;

private:

	Element in1_;
	Element in2_;
};

class ManualSwitch: public IControlElement
{
public:

	ManualSwitch(Element inTrue, Element inFalse);

	FloatingType
	getValue() override;

	void
	switchTo(bool state);

private:

	Element inTrue_;
	Element inFalse_;

	bool state_;
};

class CustomFunction: public IControlElement
{
public:

	CustomFunction(Element input, std::function<FloatingType(FloatingType)> function);

	FloatingType
	getValue() override;

private:

	Element input_;
	std::function<FloatingType(FloatingType)> function_;
};

class CustomFunction2: public IControlElement
{
public:

	CustomFunction2(Element input, Element input2, std::function<FloatingType(FloatingType, FloatingType)> function);

	FloatingType
	getValue() override;

private:

	Element input_;
	Element input2_;
	std::function<FloatingType(FloatingType, FloatingType)> function_;
};

}

#endif /* CONTROL_CONTROLELEMENTS_H_ */
