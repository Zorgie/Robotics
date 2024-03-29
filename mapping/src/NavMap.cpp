/*
 * NavMap.cpp
 *
 *  Created on: Nov 15, 2013
 *      Author: robo
 */

#include "../include/NavMap.h"
static bool DEBUG = FALSE;

/**
 * Takes a (x,y) value in robot language and translates it into a coordinate
 * that will display on the GUI map.
 */
Point NavMap::getVisualCoord(double x, double y){
	return Point((int)(drawOffset.x - drawScaling.x * (x)), (int) (drawOffset.y + drawScaling.y * (y)));
}

/**
 * Tests a node for validity.
 * @return True if the node number exists.
 */
bool NavMap::validNode(int n) {
	if (n < 0)
		return false;
	if (n >= nodes.size())
		return false;
	return true;
}

/**
 * Adds an object to the map.
 * @param x X-position.
 * @param y Y-position.
 * @param id ID of the object.
 * @param direction The direction the robot should watch to see the
 * object at specified position.
 */
bool NavMap::addObject(double x, double y, int id, int direction){
	if(objectsFound.find(id) != objectsFound.end()){
		return false;
	}
	addNode(x, y, -id);
	objectsFound[id] = lastVisitedNode.index;
	lastVisitedNode.object = direction;
	return true;
}

/**
 * Adds a node.
 * @param x X-position of the node.
 * @param y Y-position of the node.
 * @param type The type ID of the node, specified by RobotActions.h
 */
bool NavMap::addNode(double x, double y, int type) {
	mapping::robot_pose dummy;
	return addNode(x,y,type,dummy);
}

/**
 * Adds a node.
 * @param x X-position of the node.
 * @param y Y-position of the node.
 * @param type The type ID of the node, specified by RobotActions.h
 * @calibratedPose The new position of the robot, after synchronizing with the map
 * and the sensors. This variable will be modified to the correct value.
 */
bool NavMap::addNode(double x, double y, int type, mapping::robot_pose& calibratedPose){
	int newNodeId = nodes.size();
	Node n = {newNodeId,x,y,type};

	for(int i=0; i<nodes.size(); i++)
		updateNode(nodes[i]);

	updateNode(n);

	if(newNodeId == 0){
		nodes.push_back(n);
		lastVisitedNode = nodes[0];
		return true;
	}

	// Look for a matching node.
	Node match;
	bool newCreated = true;
	if (nodeMatch(n, match)) {
		printf("Matching node found!\n");
		newNodeId = match.index;
		newCreated = false;
	} else {
		// No matching node found, adds the node.
		nodes.push_back(n);
		printf("Node created.\n"); //: %d  %d %.2f %.2f %d %d %d %d\n",n.index,n.type,n.x,n.y,n.walls[0],n.walls[1],n.walls[2],n.walls[3]);
	}

	if(lastVisitedNode.index != newNodeId){
//		addEdge(newNodeId, lastVisitedNode.index, type);
		if(!newCreated){
			Point2d calib;
			for(int i=0; i<4;i++){
				if(!nodes[newNodeId].walls[i])
					continue;
				if(!getCalibratedPos(Point2d(calibratedPose.x,calibratedPose.y),i,0.2,calib))
					continue;
				if(i%2 == 0){
					cout << "Changing Xval " << calibratedPose.x << " to " << calib.x;
					calibratedPose.x = calib.x;
					cout << ". Now: " << calibratedPose.x << endl;
				}
				else{
					cout << "Changing Yval " << calibratedPose.y << " to " << calib.y;
					calibratedPose.y = calib.y;
					cout << ". Now: " << calibratedPose.y << endl;
				}
			}
		}
	}
	lastVisitedNode = nodes[newNodeId];
	getNeighbours(newNodeId, false);
	return newCreated;
}

/**
 * Checks if a too similar node already exists.
 * @param in The node to check for similarity.
 * @param res The matching node (if matched).
 * @return True if the node matched.
 */
bool NavMap::nodeMatch(Node in, Node& res){
	if(in.type < 0)
		return false;
	double minDist = IDENTICAL_NODE_DIST+1;
	for(int i=0; i<nodes.size(); i++){
		Node n = nodes[i];
		if(n.type < 0)
			continue;
		Point2d inP = Point2d(in.x,in.y);
		Point2d nP = Point2d(n.x,n.y);
		double dist = norm(inP-nP);
		if(dist > IDENTICAL_NODE_DIST || dist > minDist){
			continue;
		}
		bool sameWalls = true;
		for(int j=0; j<4; j++){
			if(n.walls[j] != in.walls[j]){
				sameWalls = false;
			}
		}
		if(sameWalls){
			minDist = dist;
			res = n;
		}
	}
	if(minDist <= IDENTICAL_NODE_DIST){
		return true;
	}
	return false;
}

/**
 * Adds an edge between two nodes (in both directions).
 */
void NavMap::addEdge(int from, int to, int type) {
	if(!validNode(from) || !validNode(to)){
		cerr << "Trying to add edge between invalid nodes: " << from << ", " << to;
		return;
	}
	bool edgeExists = false;
	for (int i = 0; i < neighbours[to].size(); i++) {
		if (neighbours[to][i].to == to)
			edgeExists = true;
	}
	if (!edgeExists)
		neighbours[to].push_back( { to, from, type });
	edgeExists = false;
	for (int i = 0; i < neighbours[from].size(); i++) {
		if (neighbours[from][i].to == from)
			edgeExists = true;
	}
	if (!edgeExists) {
		int invertedType = type;
		if (type == FOLLOW_LEFT_WALL)
			invertedType = FOLLOW_RIGHT_WALL;
		else if (type == FOLLOW_RIGHT_WALL)
			invertedType = FOLLOW_LEFT_WALL;
		neighbours[from].push_back( { from, to, invertedType });
	}
}

/**
 * Retrieves neighbours to a specified node.
 */
vector<Edge> NavMap::getNeighbours(int nodeId) {
	return getNeighbours(nodeId, false);
}

/**
 * Retrieves neighbours to a specified node.
 */
vector<Edge> NavMap::getNeighbours(int nodeId, bool cache) {
	if(cache){
		//return neighbours[nodeId];
	}
	vector<Edge> edges;
	Node from = nodes[nodeId];
	int dir;
	for(int i=0; i<nodes.size(); i++){
		if(nodes[i].index == nodeId)
			continue;
		vector<double> dists(4, INF_ISH);
		vector<double> ids(4, -1);
		Node n = nodes[i];
		double xDiff = abs(n.x - from.x);
		double yDiff = abs(n.y - from.y);
		if(yDiff < 0.1){ // Eventual horizontal neighbours.
			Point2d cross;
			if(!intersectsWithWall(from.x,from.y,n.x,n.y,cross)){
				if (n.x > from.x && xDiff < dists[0]) { // Right
					dists[0] = xDiff;
					ids[0] = n.index;
				} else if (n.x < from.x && xDiff < dists[2]) { // Left
					dists[2] = xDiff;
					ids[2] = n.index;
				}
			}
		} else if (xDiff < 0.1) {
			Point2d cross;
			if (!intersectsWithWall(from.x, from.y, n.x, n.y, cross)) {
				if (n.y > from.y && yDiff < dists[1]) { // Up
					dists[1] = yDiff;
					ids[1] = n.index;
				} else if (n.y < from.y && yDiff < dists[3]) { // Down
					dists[3] = yDiff;
					ids[3] = n.index;
				}
			}
		}
		if (ids[0] != -1) {
			Edge e = { nodeId, ids[0] };
			if (nodes[nodeId].walls[1]) {
				e.type = FOLLOW_LEFT_WALL;
			} else if (nodes[nodeId].walls[3]) {
				e.type = FOLLOW_RIGHT_WALL;
			}
			edges.push_back(e);
		}
		if (ids[1] != -1) {
			Edge e = { nodeId, ids[1] };
			if (nodes[nodeId].walls[0]) {
				e.type = FOLLOW_RIGHT_WALL;
			} else if (nodes[nodeId].walls[2]) {
				e.type = FOLLOW_LEFT_WALL;
			}
			edges.push_back(e);
		}
		if (ids[2] != -1) {
			Edge e = { nodeId, ids[2] };
			if (nodes[nodeId].walls[1]) {
				e.type = FOLLOW_RIGHT_WALL;
			} else if (nodes[nodeId].walls[3]) {
				e.type = FOLLOW_LEFT_WALL;
			}
			edges.push_back(e);
		}
		if (ids[3] != -1) {
			Edge e = { nodeId, ids[3] };
			if (nodes[nodeId].walls[0]) {
				e.type = FOLLOW_LEFT_WALL;
			} else if (nodes[nodeId].walls[2]) {
				e.type = FOLLOW_RIGHT_WALL;
			}
			edges.push_back(e);
		}
	}
	neighbours[nodeId] = edges;
	return neighbours[nodeId];
}

/**
 * Retrieves the shortest path from the last visited node to specified node.
 */
vector<Edge> NavMap::getPath(int to){
	vector<Edge> path;
	getPath(lastVisitedNode.index, to, path);
	return path;
}

/**
 * Retrieves the shortest path from the last visited node to specified node.
 * @param to Node that shall be travelled to.
 * @param path The resulting path.
 * @return The distance in meters along the path (shortest possible).
 */
double NavMap::getPath(int to, vector<Edge>& path){
	return getPath(lastVisitedNode.index, to, path);
}

/**
 * Retrieves the shortest path from node "from" to node "to".
 */
vector<Edge> NavMap::getPath(int from, int to){
	vector<Edge> path;
	getPath(from,to,path);
	return path;
}

/**
 * Retrieves the shortest path between two nodes.
 * @param from Node that shall be travelled from.
 * @param to Node that shall be travelled to.
 * @param path The resulting path.
 * @return The distance in meters along the path (shortest possible).
 */
double NavMap::getPath(int from, int to, vector<Edge>& path){
	queue<int> q;
	priority_queue<GraphNode, vector<GraphNode>, GraphNodeCmp> pq;
	vector<double> dists = vector<double>(nodes.size(), INF_ISH);
	dists[from] = 0;
	map<int, Edge> visited;
	q.push(from);
	pq.push({nodes[from], 0});
	while(!q.empty()){
		visited[from] = {-1,-1,-1};
		GraphNode gn = pq.top();
		pq.pop();
		int id = gn.n.index;
		double dist = gn.dist;
		if(id == to){
			while(id != from){
				path.insert(path.begin(), visited[id]);
				id = visited[id].from;
			}
			if(path.size() > 0)
				lastPath = path;
			return dists[to];
		}
		vector<Edge> edges = getNeighbours(gn.n.index);
		for(int i=0; i<edges.size(); i++){
			Edge e = edges[i];
			double edgeDist = dist + max(fabs(nodes[e.to].x - nodes[id].x),fabs(nodes[e.to].y - nodes[id].y));
			if(edgeDist < dists[e.to]){
				visited[e.to] = e;
				dists[e.to] = edgeDist;
				pq.push({nodes[e.to],edgeDist});
			}
		}
	}
	return -1;
}

/**
 * Adds a wall. Between specified coordinates.
 */
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

/**
 * Attempts to extend a wall to specified position.
 */
void NavMap::extendWall(double x, double y, bool horizontal) {
	double closest = INF_ISH;
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
		Point2d &intersect) {
	Point2d o1 = Point2d(x1, y1);
	Point2d p1 = Point2d(x2, y2);
	for (int i = 0; i < walls.size(); i++) {
		Wall w = walls[i];
		Point2d o2 = Point2d(w.x1, w.y1);
		Point2d p2 = Point2d(w.x2, w.y2);
		if(norm(p2-o1) < 0.09){
			continue;
		}
		if (intersection(o1, p1, o2, p2, intersect)) {
			return true;
		}
	}
	return false;
}

void NavMap::updateNode(Node& n){
	double d = NODE_WALL_CHECK / CALIBRATE_MULTIPLIER;
	Point2d origin = Point2d(n.x,n.y);
	int wallCount = 0;
	for (int i = 0; i < 4; i++) {
		Point2d collisionPoint;
		bool collision = getCalibratedPos(origin,i,d,collisionPoint);
		n.walls[i] = collision;
		wallCount += collision;
	}
}


bool NavMap::pointOnLine(Point2d pt, Point2d line1, Point2d line2) {
	double EPS = 0.01;
	double minLX = min(line1.x, line2.x) - EPS;
	double maxLX = max(line1.x, line2.x) + EPS;
	double minLY = min(line1.y, line2.y) - EPS;
	double maxLY = max(line1.y, line2.y) + EPS;
	if(pt.x < minLX || pt.x > maxLX){
		return false;
	}
	if(pt.y < minLY || pt.y > maxLY){
		return false;
	}
	return true;
}

// Finds the intersection of two lines, or returns false.
// The lines are defined by (o1, p1) and (o2, p2).
bool NavMap::intersection(Point2d o1, Point2d p1, Point2d o2, Point2d p2,
		Point2d &r) {
	Point2d x = o2 - o1;
	Point2d d1 = p1 - o1;
	Point2d d2 = p2 - o2;

	float cross = d1.x * d2.y - d1.y * d2.x;
	if (abs(cross) < /*EPS*/1e-8)
		return false;

	double t1 = (x.x * d2.y - x.y * d2.x) / cross;
	r = o1 + d1 * t1;
	// See if the intersection point lies on the line segments we check.
	if(pointOnLine(r, o1, p1) && pointOnLine(r, o2, p2)){
		return true;
	}
	return false;
}

void NavMap::draw(Mat& img) {
	double minX = INF_ISH,minY = INF_ISH, maxX = -INF_ISH, maxY = -INF_ISH;
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
//		if (norm(Point2d(w.x1, w.y1) - Point2d(w.x2, w.y2)) < WALL_MIN_LENGTH) {
//			continue;
//		}
		line(img, getVisualCoord(w.x1,w.y1),
				getVisualCoord(w.x2,w.y2),
				(w.horizontal ? black : grey), 3, 8);
	}
	// Drawing nodes.
	for (int i = 0; i < nodes.size(); i++) {
		Node n = nodes[i];
		if (n.type < 0) {
			putText(img, "X",
					getVisualCoord(n.x,n.y),
					FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 255));
		}else{
			circle(img, getVisualCoord(n.x,n.y),
					5, nodeColors[n.type % colorCount], 3, 8);
			// Draw wall connections.
			Point2d dirs[4];
			dirs[0] = Point2d(0.2,0);
			dirs[1] = Point2d(0,-0.2);
			dirs[2] = Point2d(-0.2,0);
			dirs[3] = Point2d(0, 0.2);
			for(int i=0; i<4; i++){
				if(!n.walls[i])
					continue;
				Point2d p = Point2d(n.x,n.y) + dirs[i];
				line(img,getVisualCoord(n.x,n.y),getVisualCoord(p.x,p.y),Scalar(0,0,255),2,8);
			}
		}
	}
	for(int i=0; i<nodes.size(); i++){
			// Edges for a certain node.
		vector<Edge> edges = getNeighbours(nodes[i].index, true);
			for(int j=0; j<edges.size(); j++){
				Node to = nodes[edges[j].to];
				Node from = nodes[edges[j].from];
				line(img, getVisualCoord(from.x,from.y),getVisualCoord(to.x,to.y),Scalar(100,100,100),1,8);
			}
		}
	// Drawing path
	for(int i=0; i<lastPath.size(); i++){
		Node from = nodes[lastPath[i].from];
		Node to = nodes[lastPath[i].to];
		line(img, getVisualCoord(from.x, from.y), getVisualCoord(to.x, to.y),
				Scalar(0, 255, 0), 2, 8);
	}
	// Drawing robot.
	circle(img, getVisualCoord(robotPos.x, robotPos.y),3,Scalar(0,0,0),3,8);

}


vector<Edge> NavMap::visitAllObjects(){
	vector<Edge> bigPath;
	vector<int> toVisit;
	for(map<int,int>::iterator it = objectsFound.begin(); it != objectsFound.end(); it++){
		toVisit.push_back(it->first);
	}
	int current = 0;
	while(!toVisit.empty()){
		vector<Edge> bestPath;
		double bestDist = INF_ISH;
		int bestObject = -1;
		for(int i=0; i<toVisit.size(); i++){
			int oid = toVisit[i];
			vector<Edge> path;
			double dist = getPath(current,objectsFound[oid],path);
			if(dist < bestDist){
				bestDist = dist;
				bestObject = oid;
				bestPath = path;
			}
		}
		if(bestObject != -1){
			current = objectsFound[bestObject];
			for(int i=0; i<bestPath.size(); i++){
				bigPath.push_back(bestPath[i]);
			}
			toVisit.erase(find(toVisit.begin(), toVisit.end(), bestObject));
		}else{
		}
	}
	vector<Edge> path = getPath(current,0);
	for(int i=0; i<path.size(); i++){
		bigPath.push_back(path[i]);
	}
	lastPath = bigPath;
	return bigPath;
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
	double minDist = INF_ISH;
	Point2d intersect;
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

bool NavMap::getCalibratedPos(Point2d approximatePos, int direction, double dist, Point2d& calibratedPos){
	Point2d dir;
	switch(direction){
	case 0:
		dir = Point2d(dist, 0);
	break;
	case 1:
		dir = Point2d(0, -dist);
		break;
	case 2:
		dir = Point2d(-dist, 0);
		break;
	case 3:
		dir = Point2d(0, dist);
		break;
	}
	Point2d p = approximatePos;
	Point2d o = p + Point2d(dir.x * CALIBRATE_MULTIPLIER, dir.y * CALIBRATE_MULTIPLIER);
	bool horizontal = dir.y == 0;
	for (int i = 0; i < walls.size(); i++) {
		Wall w = walls[i];
		if (w.horizontal == horizontal)
			continue;
//		if(norm(Point2d(w.x1,w.y1) - Point2d(w.x2,w.y2)) < WALL_MIN_LENGTH){
//			continue;
//		}
		Point2d intersect;
		if (intersection(Point2d(w.x1, w.y1), Point2d(w.x2, w.y2), p, o,
				intersect)) {
			if (horizontal && w.x1 < max(p.x, o.x) && w.x1 > min(p.x, o.x)) {
				calibratedPos = Point2d(intersect.x, intersect.y) - dir;
				return true;
			} else if (!horizontal && w.y1 < max(p.y, o.y)
					&& w.y1 > min(p.y, o.y)) {
				calibratedPos = Point2d(intersect.x, intersect.y) - dir;
				return true;
			}
		}
	}
	return false;
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

void NavMap::initObjectMap(){
	string objects[] = { "fulhack", "TIGER", "ZEBRA", "ELEPHANT", "HIPPO", "GIRAFFE", "LION",
			"POTATO", "TOMATO", "ONION", "BROCOLI", "PAPRIKA", "CARROT", "CORN",
			"AVOCADO", "PEPPER", "MELON", "PEAR", "BANANA", "ORANGE", "LEMON",
			"PLATE" };
	for(int i=0; i<21; i++){
		objectStrToId[objects[i]] = i;
		objectIdToStr[i] = objects[i];
	}
}
