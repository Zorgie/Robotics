/*
 * Coord.h
 *
 *  Created on: Oct 5, 2013
 *      Author: Lucas
 */

#ifndef COORD_H_
#define COORD_H_

class Coord {
public:
	Coord() : y(0), x(0) {}
	Coord(double y, double x) : y(y), x(x) {}
	~Coord();

	friend Coord operator+(const Coord& c1, const Coord& c2);
	friend Coord operator-(const Coord& c1, const Coord& c2);
public:
	double y;
	double x;
};

#endif /* COORD_H_ */
