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
	int x, y;
	int type;
};
typedef struct Wall{
	int x1, y1, x2, y2;
};

class NavMap {
private:
	map<int, vector<int> > neighbours;
	vector<Wall> walls;
	vector<Node> nodes;
	bool intersection(Point2f o1, Point2f p1, Point2f o2, Point2f p2,
            Point2f &r);
public:
	NavMap();
	virtual ~NavMap();

	void addNode(int x, int y, int type);
	vector<int> getNeighbours(int nodeId);
	void addWall(int x1, int y1, int x2, int y2);
	void extendWall(int x, int y);
	bool intersectsWithWall(int x1, int y1, int x2, int y2, Point2f &intersect);
	int getClosestReachableNode(int x, int y);
	int getDistanceToNode(int nodeId);
	void drawWalls(Mat& img, Scalar& color);
};

#endif /* NAVMAP_H_ */
