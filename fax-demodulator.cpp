#
/*
 *    Copyright (C) 2011, 2012, 2013
 *    Jan van Katwijk (J.vanKatwijk@gmail.com)
 *    Lazy Chair programming
 *
 *    This file is part of the SDRunoPlugin_fax
 *
 *    fax plugin is free software; you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License as published by
 *    the Free Software Foundation; either version 2 of the License, or
 *    (at your option) any later version.
 *
 *    fax plugin is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License for more details.
 *
 *    You should have received a copy of the GNU General Public License
 *    along with fax plugin; if not, write to the Free Software
 *    Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#include	"fax-demodulator.h"
#include	"utilities.h"
#include	"SDRunoPlugin_fax.h"

#define  _USE_MATH_DEFINES
#include        <math.h>

static inline
std::complex<float> cdiv (std::complex<float> z, float v) {
	return std::complex<float>(real(z) / v, imag(z) / v);
}

	faxDemodulator::faxDemodulator	(int8_t	mode,
	                                 int32_t rate,
	                                 int32_t dev) {

	this	-> mode	= mode;
	this	-> Rate	= rate;
	deviation	= dev;
	prevSample	= std::complex<float> (0, 0);
}

	faxDemodulator::~faxDemodulator	() {
}

void	faxDemodulator::setMode		(int8_t m) {
	mode = m;
}

void	faxDemodulator::setDeviation	(int16_t dev) {
	deviation	= dev;
}
//
//	demodulate returns a value in the range -127 .. 127
int16_t	faxDemodulator::demodulate (std::complex<float> z) {
float	res;

	z		= cdiv (z, abs (z));
	if (mode == FAX_AM)
	   return abs (z) * 255.0;

	res		= arg (conj (prevSample) * z) / (2 * M_PI) * Rate;
	res		= clamp (res, - deviation, + deviation);
	prevSample	= z;
	return (int16_t)(res / deviation * 128 + 127);
}

