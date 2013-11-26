/*
 * NavMap.h
 *
 *  Created on: Nov 15, 2013
 *      Author: robo
 */

#ifndef NAVMAP_H_
#define NAVMAP_H_

#include <vector>
#include <map>
#include <cv.h>

using namespace std;
using namespace cv;

typedef struct Node{
	int index;
	double x, y;
	int type;
};
typedef struct Wall{
	double x1, y1, x2, y2;
	bool horizontal;
};

class NavMap {
private:
	map<int, vector<int> > neighbours;
	vector<Wall> walls;
	vector<Node> nodes;
	bool intersection(Point2f o1, Point2f p1, Point2f o2, Point2f p2,
            Point2f &r);
	const static double WALL_SNAP_DISTANCE = 0.2;
	const static double WALL_MIN_LENGTH = 0.03;
public:
	NavMap(){}
	virtual ~NavMap(){}

	void addNode(double x, double y, int type);
	vector<int> getNeighbours(int nodeId);
	void addWall(double x1, double y1, double x2, double y2);
	void extendWall(double x, double y, bool horizontal);
	bool intersectsWithWall(double x1, double y1, double x2, double y2, Point2f &intersect);
	int getClosestReachableNode(double x, double y);
	double getDistanceToNode(int nodeId);
	void draw(Mat& img);

	/**
	 * Method for calibrating absolute position.
	 * @param approximatePos the x,y position that the robot thinks it has.
	 * @param relativeWallPoss The direction and distance
	 */
	Point2d getCalibratedPos(Point2d approximatePos, Point2d relativeWallPos);


	Point2d pointConversion(Point2d origin, Point2d relativePos, double angle);
};

#endif /* NAVMAP_H_ */
