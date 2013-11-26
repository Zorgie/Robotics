/*
 * NavMap.cpp
 *
 *  Created on: Nov 15, 2013
 *      Author: robo
 */

#include "../include/NavMap.h"

void NavMap::addNode(double x, double y, int type) {
	nodes.push_back( { nodes.size(), x, y, type });
}

vector<int> NavMap::getNeighbours(int nodeId) {
	return neighbours[nodeId];
}

void NavMap::addWall(double x1, double y1, double x2, double y2) {
	double snapped;
	if (fabs(x2 - x1) > fabs(y2 - y1)) {
		// Horizontal wall
		snapped = y1 + (y2 - y1) / 2;
		walls.push_back( { min(x1, x2), snapped, max(x1, x2), snapped, true });
	} else {
		// Vertical wall
		snapped = x1 + (x2 - x1) / 2;
		walls.push_back( { snapped, min(y1, y2), snapped, max(y1, y2), false });
	}
}

void NavMap::extendWall(double x, double y, bool horizontal) {
	double closest = 100000;
	int index = -1;
	int align = -1;
	Point2d ep(x, y);
	for (int i = 0; i < walls.size(); i++) {
		Wall w = walls[i];
		if (w.horizontal != horizontal)
			continue;
		printf("WY: (%.2f, %.2f)\n",w.y1, w.y2);
		double dist;
		if(horizontal){
			if(x >=w.x1 && x <= w.x2){
				dist = fabs(w.y1-y);
				if(dist < closest){
					index = i;
					align = 0;
					closest = dist;
					continue;
				}
			}
		}else{
			if(y >= w.y1 && y <= w.y2){ // WHY
				dist = fabs(w.x1-x);
				if(dist < closest){
					index = i;
					align = 0;
					closest = dist;
					continue;
				}
			}
		}
		Point2d wp(w.x1, w.y1);
		dist = norm(wp - ep);
		if (dist < closest) {
			index = i;
			align = 1;
			closest = dist;
		}
		wp = Point2d(w.x2, w.y2);
		dist = norm(wp - ep);
		if (dist < closest) {
			index = i;
			align = 2;
			closest = dist;
		}
	}
	if (closest > WALL_SNAP_DISTANCE) {
		cerr << "Closest is "<< closest << endl;
		if (horizontal)
			addWall(x, y, x + 0.01, y);
		else
			addWall(x, y, x, y + 0.01);
		return;
	}
	if (align == 1) {
		if (horizontal) // Horizontal wall
			walls[index].x1 = min(x,walls[index].x1);
		else // Vertical wall
			walls[index].y1 = min(y,walls[index].y1);
	} else if(align == 2){
		if (horizontal) // Horizontal wall
			walls[index].x2 = max(x,walls[index].x2);
		else // Vertical wall
			walls[index].y2 = max(y,walls[index].y2);
	}
}

bool NavMap::intersectsWithWall(double x1, double y1, double x2, double y2,
		Point2f &intersect) {
	Point2f o1 = Point2f(x1, y1);
	Point2f p1 = Point2f(x2, y2);
	Point2f crossPoint;
	for (int i = 0; i < walls.size(); i++) {
		Wall w = walls[i];
		Point2f o2 = Point2f(w.x1, w.y1);
		Point2f p2 = Point2f(w.x2, w.y2);
		if (intersection(o1, p1, o2, p2, crossPoint)) {
			return true;
		}
	}
	return false;
}

int NavMap::getClosestReachableNode(double x, double y) {
	//TODO implement.
	return -1;
}

double NavMap::getDistanceToNode(int nodeId) {
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
	// TODO Fix general case of sizing.
	for (int i = 0; i < walls.size(); i++) {
		Wall w = walls[i];
		if(norm(Point2d(w.x1,w.y1) - Point2d(w.x2,w.y2)) < WALL_MIN_LENGTH){
			continue;
		}
		line(img, Point((int)(200 - 100*w.x1), (int)(200 + 100*w.y1)), Point((int)(200 - 100*w.x2), (int)(200 + 100*w.y2)),
				(w.horizontal ? black : grey), 3, 8);
	}
	// Drawing nodes.
	for (int i = 0; i < nodes.size(); i++) {
		circle(img, Point((int)(200 - 100*nodes[i].x), (int)(200 + 100*nodes[i].y)), 5,
				nodeColors[nodes[i].type % colorCount], 3, 8);
	}
}

Point2d NavMap::getCalibratedPos(Point2d approximatePos,
		Point2d relativeWallPos) {
	Point2d p = approximatePos;
	Point2d o = p + Point2d(relativeWallPos.x * 10, relativeWallPos.y * 10);
	bool horizontal = relativeWallPos.y == 0;
	for (int i = 0; i < walls.size(); i++) {
		Wall w = walls[i];
		if (w.horizontal == horizontal)
			continue;
//		if(norm(Point2d(w.x1,w.y1) - Point2d(w.x2,w.y2)) < WALL_MIN_LENGTH){
//			continue;
//		}
		Point2f intersect;
		if (intersection(Point2d(w.x1, w.y1), Point2d(w.x2, w.y2), p, o,
				intersect)) {
			if (horizontal && w.x1 < max(p.x, o.x) && w.x1 > min(p.x, o.x)) {
				return Point2d(intersect.x,intersect.y) - relativeWallPos;
			} else if (!horizontal && w.y1 < max(p.y, o.y)
					&& w.y1 > min(p.y, o.y)) {
				return Point2d(intersect.x,intersect.y) - relativeWallPos;
			}
		}
	}
	return p;
}

Point2d NavMap::pointConversion(Point2d origin, Point2d relativePos, double angle){
		Point2d world;
		world.x = origin.x + cos(angle)*relativePos.x - sin(angle)*relativePos.y;
		world.y = origin.y + sin(angle)*relativePos.x + cos(angle)*relativePos.y;
		return world;
	}
