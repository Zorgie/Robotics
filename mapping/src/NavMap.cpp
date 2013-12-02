/*
 * NavMap.cpp
 *
 *  Created on: Nov 15, 2013
 *      Author: robo
 */

#include "../include/NavMap.h"

Point NavMap::getVisualCoord(double x, double y){
	return Point((int)(drawOffset.x - drawScaling.x * (x)), (int) (drawOffset.y + drawScaling.y * (y)));
}


bool NavMap::validNode(int n) {
	if (n < 0)
		return false;
	if (n >= nodes.size())
		return false;
	return true;
}

void NavMap::addNode(double x, double y, int type) {
	if (type == -1) { // Object
		for (int i = 0; i < nodes.size(); i++) {
			if(nodes[i].type != type)
				continue;
			double distance = sqrt(pow(x - nodes[i].x, 2) + pow(y - nodes[i].y, 2));
			if (distance < 0.2) {
				return;
			}
		}
	}

	int newNodeId = nodes.size();
	Node n = {newNodeId,x,y,type};
	for (int i = 0; i < 4; i++) {
		double wallDist = getDistanceToWall(Point2d(x, y), i);
		if(wallDist < 0.3){
			n.walls[i] = true;
		}else{
			n.walls[i] = false;
		}
	}
	nodes.push_back(n);
	printf("Node created: %d  %d %.2f %.2f %d %d %d %d\n",n.index,n.type,n.x,n.y,n.walls[0],n.walls[1],n.walls[2],n.walls[3]);


	if(newNodeId == 0){
		lastVisitedNode = nodes[0];
		return;
	}
	addEdge(newNodeId, lastVisitedNode.index, type);
	lastVisitedNode = nodes[newNodeId];
}

void NavMap::addEdge(int from, int to, int type) {
	if(!validNode(from) || !validNode(to)){
		cerr << "Trying to add edge between invalid nodes: " << from << ", " << to;
		throw -1;
	}
	neighbours[from] = vector<Edge>();
	neighbours[to].push_back( { to, from, type });
	int invertedType = type;
	if (type == FOLLOW_LEFT_WALL)
		invertedType = FOLLOW_RIGHT_WALL;
	else if (type == FOLLOW_RIGHT_WALL)
		invertedType = FOLLOW_LEFT_WALL;
	neighbours[from].push_back( { from, to, invertedType });
}

vector<Edge> NavMap::getNeighbours(int nodeId) {
	return neighbours[nodeId];
}

vector<Edge> NavMap::getPath(int from, int to){
	vector<Edge> path;
	queue<int> q;
	map<int, Edge> visited;
	q.push(from);
	while(!q.empty()){
		int id = q.front();
		q.pop();
		visited[from] = {-1,-1,-1};
		Node n = nodes[id];
		if(id == to){
			while(id != from){
				path.insert(path.begin(), visited[id]);
				id = visited[id].from;
			}
			return path;
		}
		for(int i=0; i<neighbours[n.index].size(); i++){
			Edge e = neighbours[n.index][i];
			if(!visited.count(e.to)){// Confirms that the node is unvisited.
				q.push(e.to);
				visited[e.to] = e;
				cerr << "adding " << e.to << " to visited." << endl;
			}
		}
	}
	return path;
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
		double dist;
		if (horizontal) {
			if (x >= w.x1 && x <= w.x2) {
				dist = fabs(w.y1 - y);
				if (dist < closest) {
					index = i;
					align = 0;
					closest = dist;
					continue;
				}
			}
		} else {
			if (y >= w.y1 && y <= w.y2) { // WHY
				dist = fabs(w.x1 - x);
				if (dist < closest) {
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
	if (closest > SNAP_DISTANCE) {
		if (horizontal)
			addWall(x, y, x + 0.01, y);
		else
			addWall(x, y, x, y + 0.01);
		return;
	}
	if (align == 1) {
		if (horizontal) // Horizontal wall
			walls[index].x1 = min(x, walls[index].x1);
		else
			// Vertical wall
			walls[index].y1 = min(y, walls[index].y1);
	} else if (align == 2) {
		if (horizontal) // Horizontal wall
			walls[index].x2 = max(x, walls[index].x2);
		else
			// Vertical wall
			walls[index].y2 = max(y, walls[index].y2);
	}
	Wall w = walls[index];
}

bool NavMap::intersectsWithWall(double x1, double y1, double x2, double y2,
		Point2f &intersect) {
	Point2f o1 = Point2f(x1, y1);
	Point2f p1 = Point2f(x2, y2);
	for (int i = 0; i < walls.size(); i++) {
		Wall w = walls[i];
		Point2f o2 = Point2f(w.x1, w.y1);
		Point2f p2 = Point2f(w.x2, w.y2);
		if (intersection(o1, p1, o2, p2, intersect)) {
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
	double minX = 1000,minY = 1000, maxX = -1000, maxY = -1000;
	for(int i=0; i<walls.size(); i++){
		Wall w = walls[i];
		minX = min(w.x1, minX);
		maxX = max(w.x2, maxX);
		minY = min(w.y1, minY);
		maxY = max(w.y2, maxY);
	}

//	drawScaling.x = img.cols / (maxX - minX);
//	drawScaling.y = img.rows / (maxY - minY);
//	drawOffset.x = img.cols;//-(int)(minX * drawScaling.x);
//	drawOffset.y = -(int)(minY * drawScaling.y);
//	drawScaling.x = drawScaling.x*0.5;
//	drawScaling.y = drawScaling.y*0.5;


	img.setTo(cv::Scalar(255, 255, 255));
	Scalar black = Scalar(0, 0, 0);
	Scalar grey = Scalar(100, 100, 100);
	Scalar nodeColors[] = { Scalar(255, 0, 0), Scalar(0, 255, 0), Scalar(0, 0,
			255), Scalar(100, 0, 0), Scalar(0, 100, 0), Scalar(0, 0, 100), Scalar(100, 100, 0) };
	int colorCount = 7;

	// Drawing walls.
	// TODO Fix general case of sizing.
	for (int i = 0; i < walls.size(); i++) {
		Wall w = walls[i];
		if (norm(Point2d(w.x1, w.y1) - Point2d(w.x2, w.y2)) < WALL_MIN_LENGTH) {
			continue;
		}
		line(img, getVisualCoord(w.x1,w.y1),
				getVisualCoord(w.x2,w.y2),
				(w.horizontal ? black : grey), 3, 8);
	}
	// Drawing nodes.
	for (int i = 0; i < nodes.size(); i++) {
		Node n = nodes[i];
		if (n.type == -1) {
			putText(img, "X",
					getVisualCoord(n.x,n.y),
					FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 255));
		}else{
			circle(img, getVisualCoord(n.x,n.y),
					5, nodeColors[n.type % colorCount], 3, 8);
		}
	}
	// Drawing edges
	for(int i=0; i<neighbours.size(); i++){
		// Edges for a certain node.
		for(int j=0; j<neighbours[i].size(); j++){
			Node from = nodes[neighbours[i][j].from];
			Node to = nodes[neighbours[i][j].to];
			line(img, getVisualCoord(from.x,from.y),getVisualCoord(to.x,to.y),Scalar(100,100,100),1,8);
		}
	}
	// Drawing robot.
	circle(img, getVisualCoord(robotPos.x, robotPos.y),3,Scalar(0,0,0),3,8);

}

double NavMap::getDistanceToWall(Point2d origin, int direction){
	Point2d o;
	switch(direction){
	case 0:
		o = origin + Point2d(10,0);
		break;
	case 1:
		o = origin + Point2d(0,10);
		break;
	case 2:
		o = origin + Point2d(-10,0);
		break;
	case 3:
		o = origin + Point2d(0, -10);
		break;
	}
	double minDist = 10000;
	Point2f intersect;
	for (int i = 0; i < walls.size(); i++) {
		Wall w = walls[i];
		if (w.horizontal == (direction == 0 || direction == 1)) {
			continue;
		}
		if (intersection(Point2d(w.x1, w.y1), Point2d(w.x2, w.y2), origin, o,
				intersect)) {
			double dist = sqrt(pow(origin.x - intersect.x,2) + pow(origin.y - intersect.y,2));
			minDist = min(minDist, dist);
		}
	}
	printf("Distance to wall %d: %.2f\n", direction, minDist);
	return minDist;
}

bool NavMap::getCalibratedPos(Point2d approximatePos,
		Point2d relativeWallPos, Point2d& calibratedPos) {
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
				calibratedPos = Point2d(intersect.x, intersect.y) - relativeWallPos;
				return true;
			} else if (!horizontal && w.y1 < max(p.y, o.y)
					&& w.y1 > min(p.y, o.y)) {
				calibratedPos = Point2d(intersect.x, intersect.y) - relativeWallPos;
				return true;
			}
		}
	}
	return true;
}

Point2d NavMap::pointConversion(Point2d origin, Point2d relativePos,
		double angle) {
	Point2d world;
	world.x = origin.x + cos(angle) * relativePos.x
			- sin(angle) * relativePos.y;
	world.y = origin.y + sin(angle) * relativePos.x
			+ cos(angle) * relativePos.y;
	return world;
}
