/*
 * NavMap.cpp
 *
 *  Created on: Nov 15, 2013
 *      Author: robo
 */

#include "NavMap.h"

NavMap::NavMap() {
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
	int snapped;
	if (x2 - x1 > y2 - y1) {
		// Horizontal wall
		snapped = y1 + (y2 - y1) / 2;
		walls.push_back( { min(x1, x2), snapped, max(x1, x2), snapped, true });
	} else {
		// Vertical wall
		snapped = x1 + (x2 - x1) / 2;
		walls.push_back( { snapped, min(y1, y2), snapped, max(y1, y2), false });
	}
}

void NavMap::extendWall(int x, int y, bool horizontal) {
	double closest = 100000;
	int index = -1;
	int align = -1;
	Point ep(x, y);
	for (int i = 0; i < walls.size(); i++) {
		Wall w = walls[i];
		if (w.horizontal != horizontal)
			continue;
		Point wp(w.x1, w.y2);
		double dist = norm(wp - ep);
		if (dist < closest) {
			index = i;
			align = 1;
			closest = dist;
		}
		wp = Point(walls[i].x2, walls[i].y2);
		dist = norm(wp - ep);
		if (dist < closest) {
			index = i;
			align = 2;
			closest = dist;
		}
	}
	if (closest > WALL_SNAP_DISTANCE) {
		if (horizontal)
			addWall(x, y, x + 1, y);
		else
			addWall(x, y, x, y + 1);
		return;
	}
	if (align == 1) {
		if (horizontal) // Horizontal wall
			walls[index].x1 = x;
		else // Vertical wall
			walls[index].y1 = y;
	} else {
		if (horizontal)// Horizontal wall
			walls[index].x2 = x;
		else// Vertical wall
			walls[index].y2 = y;
	}
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

void NavMap::draw(Mat& img) {
	img.setTo(cv::Scalar(255, 255, 255));
	Scalar black = Scalar(0, 0, 0);
	Scalar grey = Scalar(100, 100, 100);
	Scalar nodeColors[] = { Scalar(255, 0, 0), Scalar(0, 255, 0), Scalar(0, 0,
			255) };
	int colorCount = 3;
	// Drawing walls.
	for (int i = 0; i < walls.size(); i++) {
		Wall w = walls[i];
		line(img, Point(w.x1, w.y1), Point(w.x2, w.y2),
				(w.horizontal ? black : grey), 3, 8);
	}
	// Drawing nodes.
	for (int i = 0; i < nodes.size(); i++) {
		circle(img, Point(nodes[i].x, nodes[i].y), 5,
				nodeColors[nodes[i].type % colorCount], 3, 8);
	}
}

Point NavMap::getCalibratedPos(Point approximatePos,
		Point relativeWallPos) {
	Point p = approximatePos;
	Point o = p + Point(relativeWallPos.x * 10, relativeWallPos.y * 10);
	bool horizontal = relativeWallPos.y == 0;
	for (int i = 0; i < walls.size(); i++) {
		Wall w = walls[i];
		if (w.horizontal == horizontal)
			continue;
		Point2f intersect;
		if (intersection(Point(w.x1, w.y1), Point(w.x2, w.y2), p, o,
				intersect)) {
			if (horizontal && w.x1 < max(p.x, o.x) && w.x1 > min(p.x, o.x)) {
				return Point2i(intersect.x,intersect.y) - relativeWallPos;
			} else if (!horizontal && w.y1 < max(p.y, o.y)
					&& w.y1 > min(p.y, o.y)) {
				return Point2i(intersect.x,intersect.y) - relativeWallPos;
			}
		}
	}
	return p;
}
