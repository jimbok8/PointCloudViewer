// Title:   StringTools.cpp
// Created: Thu Sep 25 15:01:33 2003
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
#include <math.h>

#include "StringTools.h"

std::string StringTools::appendSequenceNumber (const std::string &str,
										const unsigned int sequenceNumber,
										const unsigned int nofDigits) 
{
		int     nofNeededDigits,
			nofZeros,
			i;
		std::string newString;

		nofNeededDigits = (int)log10 ((float)sequenceNumber) + 1;
		nofZeros  = nofDigits - nofNeededDigits;

		newString = str;
		for (i = 0; i < nofZeros; i++) {
			newString.append ("0");
		}
		char buf[256];
		_itoa_s (sequenceNumber,buf,10);
		newString.append (buf);
		return newString;
}

int StringTools::getSequenceNumber (const std::string &str, 
			const unsigned int nofDigits, const unsigned int position) 
{
	std::string sequenceNumberString;
	int     sequenceNumber;

	if (str.length() < nofDigits + position) {
		return -1;
	}

	sequenceNumberString = str.substr(str.length()-nofDigits);

	sequenceNumber = atoi(sequenceNumberString.c_str());
	if (sequenceNumber < INT_MIN || sequenceNumber > INT_MAX) 
	{
		return -1;
	}
	return sequenceNumber;
}

// Some Emacs-Hints -- please don't remove:
//
//  Local Variables:
//  mode:C++
//  tab-width:4
//  End:
