/*/
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
#include <mapping/robot_pose.h>

using namespace std;
using namespace cv;

typedef struct Node{
	int index;
	double x, y;
	int type;
	bool walls[4];
	int object;
};
typedef struct Wall{
	double x1, y1, x2, y2;
	bool horizontal;
};
typedef struct Edge{
	int from, to, type;
};
typedef struct GraphNode{
	Node n;
	double dist;
};
struct GraphNodeCmp
{
  bool operator()(const GraphNode& lhs, const GraphNode& rhs)
  {
	  return lhs.dist > rhs.dist;
  }
};


class NavMap {
private:
	map<int, vector<Edge> > neighbours;
	map<int, int> objectsFound;
	vector<Wall> walls;
	vector<Node> nodes;
	vector<Edge> lastPath;
	Node lastVisitedNode;
	bool intersection(Point2d o1, Point2d p1, Point2d o2, Point2d p2,
            Point2d &r);
	const static double SNAP_DISTANCE = 0.2;
	const static double WALL_MIN_LENGTH = 0.1;
	const static double CALIBRATE_MULTIPLIER = 1.5;
	const static double IDENTICAL_NODE_DIST = 0.3;
	const static double NODE_DIST_FROM_CORNER = 0.15;
	const static double NODE_WALL_CHECK = 0.35;

	Point drawOffset;
	Point2d drawScaling;
	Point2d robotPos;
	Point getVisualCoord(double x, double y);
	bool pointOnLine(Point2d pt, Point2d line1, Point2d line2);
public:
	map<int, string> objectIdToStr;
	map<string, int> objectStrToId;
	/* ----------- Constructors, destructors. -----------*/
	NavMap(){
		drawOffset = Point(100, 100);
		drawScaling = Point2d(50, 50);
		initObjectMap();
	}
	virtual ~NavMap(){}
	void init();
	void initObjectMap();
	/* ----------- Basic getters / setters. -------------*/
	void setRobotPos(Point2d p){robotPos = p;}
	int getNodeCount(){return nodes.size();}
	Node getNode(int index){return nodes[index];}
	Node getLastVisitedNode(){return lastVisitedNode;}

	/* ----------- Error checking -------------------------*/
	bool validNode(int n);

	/* ----------  Functions  -----------------------------*/
	/* ----------  Wall functionality ---------------------*/
	void addWall(double x1, double y1, double x2, double y2);
	void extendWall(double x, double y, bool horizontal);
	bool intersectsWithWall(double x1, double y1, double x2, double y2, Point2d &intersect);
	double getDistanceToWall(Point2d origin, int direction);
	/* ----------  Node functionality ---------------------*/
	bool addObject(double x, double y, int id, int direction);
	bool addNode(double x, double y, int type, mapping::robot_pose& calibratedPose);
	bool addNode(double x, double y, int type);
	bool nodeMatch(Node in, Node& res);
	void nodeMerge(Node& to, Node& from);
	void addEdge(int from, int to, int type);
	vector<Edge> getNeighbours(int nodeId);
	vector<Edge> getPath(int to);
	vector<Edge> getPath(int from, int to);
	double getPath(int to, vector<Edge>& path);
	double getPath(int from, int to, vector<Edge>& path);
	vector<Edge> visitAllObjects();
	int getClosestReachableNode(double x, double y);
	double getDistanceToNode(int nodeId);
	void updateNode(Node& n);
	/* ----------  Misc functionality ---------------------*/
	void draw(Mat& img);
	//bool getCalibratedPos(Point2d approximatePos, Point2d relativeWallPos, Point2d& calibratedPos);
	bool getCalibratedPos(Point2d approximatePos, int dir, double dist, Point2d& calibratedPos);
	Point2d pointConversion(Point2d origin, Point2d relativePos, double angle);


};

#endif /* NAVMAP_H_ */
