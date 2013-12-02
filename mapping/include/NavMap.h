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
#include <queue>
#include <set>
#include <cv.h>
#include <navigation/RobotActions.h>

using namespace std;
using namespace cv;

typedef struct Node{
	int index;
	double x, y;
	int type;
	bool walls[4];
};
typedef struct Wall{
	double x1, y1, x2, y2;
	bool horizontal;
};
typedef struct Edge{
	int from, to, type;
};

class NavMap {
private:
	map<int, vector<Edge> > neighbours;
	vector<Wall> walls;
	vector<Node> nodes;
	Node lastVisitedNode;
	bool intersection(Point2f o1, Point2f p1, Point2f o2, Point2f p2,
            Point2f &r);
	const static double SNAP_DISTANCE = 0.2;
	const static double WALL_MIN_LENGTH = 0.03;

	Point drawOffset;
	Point2d drawScaling;
	Point getVisualCoord(double x, double y);
	Point2d robotPos;
public:
	/* ----------- Constructors, destructors. -----------*/
	NavMap(){
		drawOffset = Point(200,200);
		drawScaling = Point2d(100,100);
	}
	virtual ~NavMap(){}
	/* ----------- Basic getters / setters. -------------*/
	void setRobotPos(Point2d p){robotPos = p;}
	int getNodeCount(){return nodes.size();}

	/* ----------- Error checking -------------------------*/
	bool validNode(int n);

	/* ----------  Functions  -----------------------------*/
	void addNode(double x, double y, int type);
	void addEdge(int from, int to, int type);
	vector<Edge> getNeighbours(int nodeId);
	vector<Edge> getPath(int from, int to);
	void addWall(double x1, double y1, double x2, double y2);
	void extendWall(double x, double y, bool horizontal);
	bool intersectsWithWall(double x1, double y1, double x2, double y2, Point2f &intersect);
	int getClosestReachableNode(double x, double y);
	double getDistanceToNode(int nodeId);
	void draw(Mat& img);

	double getDistanceToWall(Point2d origin, int direction);
	/**
	 * Method for calibrating absolute position.
	 * @param approximatePos the x,y position that the robot thinks it has.
	 * @param relativeWallPoss The direction and distance
	 */
	bool getCalibratedPos(Point2d approximatePos, Point2d relativeWallPos, Point2d& calibratedPos);


	Point2d pointConversion(Point2d origin, Point2d relativePos, double angle);
};

#endif /* NAVMAP_H_ */
