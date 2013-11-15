/*
 * NavMap.cpp
 *
 *  Created on: Nov 15, 2013
 *      Author: robo
 */

#include "NavMap.h"

NavMap::NavMap() {/*
 neighbours = map<int, vector<int> >;
 walls = vector<Wall>;
 nodes = vector<Node>;*/
}

NavMap::~NavMap() {
}

void NavMap::addNode(int x, int y, int type) {
	nodes.push_back( { nodes.size(), x, y, type });
}

vector<int> NavMap::getNeighbours(int nodeId) {
	return neighbours[nodeId];
}

void NavMap::addWall(int x1, int y1, int x2, int y2) {
	walls.push_back( { x1, y1, x2, y2 });
}

bool NavMap::intersectsWithWall(int x1, int y1, int x2, int y2,
		Point2f &intersect) {
	Point2f o1 = Point(x1, y1);
	Point2f p1 = Point(x2, y2);
	Point2f crossPoint;
	for (int i = 0; i < walls.size(); i++) {
		Wall w = walls[i];
		Point2f o2 = Point(w.x1, w.y1);
		Point2f p2 = Point(w.x2, w.y2);
		if (intersection(o1, p1, o2, p2, crossPoint)) {
			return true;
		}
	}
	return false;
}

int NavMap::getClosestReachableNode(int x, int y) {
	//TODO implement.
	return -1;
}

int NavMap::getDistanceToNode(int nodeId) {
	//TODO implement.
	return -1;
}

// Finds the intersection of two lines, or returns false.
// The lines are defined by (o1, p1) and (o2, p2).
bool NavMap::intersection(Point2f o1, Point2f p1, Point2f o2, Point2f p2,
		Point2f &r) {
	Point2f x = o2 - o1;
	Point2f d1 = p1 - o1;
	Point2f d2 = p2 - o2;

	float cross = d1.x * d2.y - d1.y * d2.x;
	if (abs(cross) < /*EPS*/1e-8)
		return false;

	double t1 = (x.x * d2.y - x.y * d2.x) / cross;
	r = o1 + d1 * t1;
	return true;
}
