/*
 * Coord.cpp
 *
 *  Created on: Oct 5, 2013
 *      Author: robo
 */

#include "Coord.h"

Coord::~Coord() {
}

Coord operator+(const Coord& c1, const Coord& c2){
	return Coord(c1.y+c2.y, c1.x+c2.x);
}

Coord operator-(const Coord& c1, const Coord& c2){
	return Coord(c1.y-c2.y, c1.x-c2.x);
}

