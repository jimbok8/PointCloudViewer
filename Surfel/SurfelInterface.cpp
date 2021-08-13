// Title:   SurfelInterface.cpp
// Created: Thu Sep 25 14:16:34 2003
// Authors: Richard Keiser, Oliver Knoll, Mark Pauly, Matthias Zwicker
//
// Copyright (c) 2001, 2002, 2003 Computer Graphics Lab, ETH Zurich
//
// This file is part of the Pointshop3D system.
// See http://www.pointshop3d.com/ for more information.
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License as
// published by the Free Software Foundation; either version 2 of
// the License, or (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public
// License along with this program; if not, write to the Free
// Software Foundation, Inc., 59 Temple Place - Suite 330, Boston,
// MA 02111-1307, USA.
//
// Contact info@pointshop3d.com if any conditions of this
// licensing are not clear to you.
//

//#include "stdafx.h"
#include "SurfelInterface.h"

// *********
// constants
// *********

const SurfelInterface::PropertyDescriptor SurfelInterface::BaseSurfel_PropertyDescriptor = SurfelInterface::POSITION |
	SurfelInterface::NORMAL |
	SurfelInterface::RADIUS |
	SurfelInterface::DIFFUSE_COLOR |
	SurfelInterface::SPECULAR_COLOR |
	SurfelInterface::FLAGS;

const SurfelInterface::PropertyDescriptor SurfelInterface::ElSurfel_PropertyDescriptor = SurfelInterface::POSITION |
	SurfelInterface::NORMAL |
	SurfelInterface::TANGENT_AXES |
	SurfelInterface::DIFFUSE_COLOR |
	SurfelInterface::SPECULAR_COLOR |
	SurfelInterface::FLAGS |
	SurfelInterface::AMBIENT_COEFFICIENT |
	SurfelInterface::DIFFUSE_COEFFICIENT |
	SurfelInterface::SPECULAR_COEFFICIENT |
	SurfelInterface::SHININESS |
	SurfelInterface::TEXTURE_COORDINATE;

const SurfelInterface::PropertyDescriptor SurfelInterface::EnhElSurfel_PropertyDescriptor = ElSurfel_PropertyDescriptor |
	SurfelInterface::DETAIL |
	SurfelInterface::ASSOCIATED_SURFELS;

const SurfelInterface::PropertyDescriptor SurfelInterface::Surfel1_PropertyDescriptor = SurfelInterface::POSITION |
	SurfelInterface::NORMAL |
	SurfelInterface::RADIUS |
	SurfelInterface::DIFFUSE_COLOR |
	SurfelInterface::SPECULAR_COLOR |
	SurfelInterface::FLAGS |
	SurfelInterface::AMBIENT_COEFFICIENT |
	SurfelInterface::DIFFUSE_COEFFICIENT |
	SurfelInterface::SPECULAR_COEFFICIENT |
	SurfelInterface::SHININESS |
	SurfelInterface::TEXTURE_COORDINATE;


// total number of selections
static const int NUM_SELECTIONS = 3;


// note: must not contain spaces if used in brush because they are used as filenames
// for brushes!!

static const std::string USER_NO_PROPERTIES        = std::string ("No property");
static const std::string USER_POSITION             = std::string ("Position");
static const std::string USER_NORMAL               = std::string ("Normal");
static const std::string USER_TANGENT_AXES         = std::string ("Tangent Axes");
static const std::string USER_RADIUS               = std::string ("Radius");
static const std::string USER_DIFFUSE_COLOR        = std::string ("Diff.Color");
static const std::string USER_SPECULAR_COLOR       = std::string ("Spec.Color");
static const std::string USER_FLAGS                = std::string ("Surfel flags");
static const std::string USER_AMBIENT_COEFFICIENT  = std::string ("Ambient");
static const std::string USER_DIFFUSE_COEFFICIENT  = std::string ("Diffuse");
static const std::string USER_SPECULAR_COEFFICIENT = std::string ("Specular");
static const std::string USER_SHININESS            = std::string ("Shininess");
static const std::string USER_TEXTURE_COORDINATE   = std::string ("Texture coordinate");
static const std::string USER_ALL_PROPERTIES       = std::string ("All properties");
static const std::string USER_NOT_SUPPORTED        = std::string ("Not supported property");

// **************
// static methods
// **************

std::string SurfelInterface::getUserReadableProperty (const Property property) {

	switch (property) {

	case SurfelInterface::NO_PROPERTIES:
		return USER_NO_PROPERTIES;
		break;

	case SurfelInterface::POSITION:
		return USER_POSITION;
		break;

	case SurfelInterface::NORMAL:
		return USER_NORMAL;
		break;

	case SurfelInterface::TANGENT_AXES:
		return USER_TANGENT_AXES;
		break;

	case SurfelInterface::RADIUS:
		return USER_RADIUS;
		break;

	case SurfelInterface::DIFFUSE_COLOR:
		return USER_DIFFUSE_COLOR;
		break;

	case SurfelInterface::SPECULAR_COLOR:
		return USER_SPECULAR_COLOR;
		break;

	case SurfelInterface::FLAGS:
		return USER_FLAGS;
		break;

	case SurfelInterface::AMBIENT_COEFFICIENT:
		return USER_AMBIENT_COEFFICIENT;
		break;

	case SurfelInterface::DIFFUSE_COEFFICIENT:
		return USER_DIFFUSE_COEFFICIENT;
		break;

	case SurfelInterface::SPECULAR_COEFFICIENT:
		return USER_SPECULAR_COEFFICIENT;
		break;

	case SurfelInterface::SHININESS:
		return USER_SHININESS;
		break;

	case SurfelInterface::TEXTURE_COORDINATE:
		return USER_TEXTURE_COORDINATE;
		break;

	case SurfelInterface::ALL_PROPERTIES:
		return USER_ALL_PROPERTIES;
		break;

	default:
		// please upgrade user 8b
		return USER_NOT_SUPPORTED;
		break;

	}  // switch


}

SurfelInterface::Property SurfelInterface::getSystemProperty (const std::string &userReadableProperty) {

	if (userReadableProperty == USER_NO_PROPERTIES) {
		return SurfelInterface::NO_PROPERTIES;
	}
	else if (userReadableProperty == USER_POSITION) {
		return SurfelInterface::POSITION;
	}
	else if (userReadableProperty == USER_NORMAL) {
		return SurfelInterface::NORMAL;
	}
	else if (userReadableProperty == USER_RADIUS) {
		return SurfelInterface::RADIUS;
	}
	else if (userReadableProperty == USER_DIFFUSE_COLOR) {
		return SurfelInterface::DIFFUSE_COLOR;
	}
	else if (userReadableProperty == USER_SPECULAR_COLOR) {
		return SurfelInterface::SPECULAR_COLOR;
	}
	else if (userReadableProperty == USER_FLAGS) {
		return SurfelInterface::FLAGS;
	}
	else if (userReadableProperty == USER_AMBIENT_COEFFICIENT) {
		return SurfelInterface::AMBIENT_COEFFICIENT;
	}
	else if (userReadableProperty == USER_DIFFUSE_COEFFICIENT) {
		return SurfelInterface::DIFFUSE_COEFFICIENT;
	}
	else if (userReadableProperty == USER_SPECULAR_COEFFICIENT) {
		return SurfelInterface::SPECULAR_COEFFICIENT;
	}
	else if (userReadableProperty == USER_SHININESS) {
		return SurfelInterface::SHININESS;
	}
	else if (userReadableProperty == USER_TEXTURE_COORDINATE) {
		return SurfelInterface::TEXTURE_COORDINATE;
	}
	else {
		return SurfelInterface::NO_PROPERTIES;
	}

}

void SurfelInterface::getEditableProperties(std::vector<std::string> &vProps) {

	vProps.resize(0);

	vProps.push_back(USER_DIFFUSE_COLOR);
	vProps.push_back(USER_POSITION);
	vProps.push_back(USER_SPECULAR_COLOR);
	vProps.push_back(USER_AMBIENT_COEFFICIENT);
	vProps.push_back(USER_DIFFUSE_COEFFICIENT);
	vProps.push_back(USER_SPECULAR_COEFFICIENT);
	vProps.push_back(USER_SHININESS);
}

SurfelInterface::PropertyDescriptor SurfelInterface::getEditablePropertyDescriptor() {

	SurfelInterface::PropertyDescriptor editableProperties;

	editableProperties = SurfelInterface::POSITION | SurfelInterface::DIFFUSE_COLOR | SurfelInterface::SPECULAR_COLOR |
		SurfelInterface::AMBIENT_COEFFICIENT | SurfelInterface::DIFFUSE_COEFFICIENT | SurfelInterface::SPECULAR_COEFFICIENT |
		SurfelInterface::SHININESS;

	return editableProperties;

}

int SurfelInterface::getNumOfEditableProperties () 
{
	std::vector<std::string> vProps;
	getEditableProperties(vProps);
	return (int)vProps.size();
}

int SurfelInterface::getNumOfPropertyBrushComponents(Property property) {

	switch (property) {

	case SurfelInterface::NO_PROPERTIES:
		return 0;
		break;

		// the brush modifies surfel positions in a displacement mapping manner, i.e., it displaces
		// the position along some direction. the brush stores only the distance along this direction.
		// there is a second component that can be used by the brush tool as a temporary displacement
		// map.
	case SurfelInterface::POSITION:
		return 2;
		break;

	case SurfelInterface::NORMAL:
		return 3;
		break;

	case SurfelInterface::RADIUS:
		return 1;
		break;

	case SurfelInterface::DIFFUSE_COLOR:
		return 3;
		break;

	case SurfelInterface::SPECULAR_COLOR:
		return 3;
		break;

	case SurfelInterface::FLAGS:
		return 0;
		break;

	case SurfelInterface::AMBIENT_COEFFICIENT:
		return 1;
		break;

	case SurfelInterface::DIFFUSE_COEFFICIENT:
		return 1;
		break;

	case SurfelInterface::SPECULAR_COEFFICIENT:
		return 1;
		break;

	case SurfelInterface::SHININESS:
		return 1;
		break;

	case SurfelInterface::TEXTURE_COORDINATE:
		return 2;
		break;

	case SurfelInterface::ALL_PROPERTIES:
		return 0;
		break;

	default:
		return 0;
		break;

	}  // switch

}


int SurfelInterface::getNumOfSelections () {

	return NUM_SELECTIONS;
}


SurfelInterface::Flags SurfelInterface::getSelectionFlags() {

	return SurfelInterface::SELECTED1 | SurfelInterface::SELECTED2 | SurfelInterface::SELECTED3;
}



// Some Emacs-Hints -- please don't remove:
//
//  Local Variables:
//  mode:C++
//  tab-width:4
//  End:
