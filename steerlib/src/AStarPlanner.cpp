//
// Copyright (c) 2009-2015 Glen Berseth, Mubbasir Kapadia, Shawn Singh, Petros Faloutsos, Glenn Reinman, Rahul Shome
// See license.txt for complete license.
//


#include <vector>
#include <stack>
#include <set>
#include <map>
#include <iostream>
#include <algorithm> 
#include <functional>
#include <queue>
#include <math.h>
#include "planning/AStarPlanner.h"


#define COLLISION_COST  1000
#define GRID_STEP  1
#define OBSTACLE_CLEARANCE 1
#define MIN(X,Y) ((X) < (Y) ? (X) : (Y))
#define MAX(X,Y) ((X) > (Y) ? (X) : (Y))

namespace SteerLib
{
	AStarPlanner::AStarPlanner() {}

	AStarPlanner::~AStarPlanner() {}

	bool AStarPlanner::canBeTraversed(int id)
	{
		double traversal_cost = 0;
		int current_id = id;
		unsigned int x, z;
		gSpatialDatabase->getGridCoordinatesFromIndex(current_id, x, z);
		int x_range_min, x_range_max, z_range_min, z_range_max;

		x_range_min = MAX(x - OBSTACLE_CLEARANCE, 0);
		x_range_max = MIN(x + OBSTACLE_CLEARANCE, gSpatialDatabase->getNumCellsX());

		z_range_min = MAX(z - OBSTACLE_CLEARANCE, 0);
		z_range_max = MIN(z + OBSTACLE_CLEARANCE, gSpatialDatabase->getNumCellsZ());


		for (int i = x_range_min; i <= x_range_max; i += GRID_STEP)
		{
			for (int j = z_range_min; j <= z_range_max; j += GRID_STEP)
			{
				int index = gSpatialDatabase->getCellIndexFromGridCoords(i, j);
				traversal_cost += gSpatialDatabase->getTraversalCost(index);

			}
		}

		if (traversal_cost > COLLISION_COST)
			return false;
		return true;
	}

	std::vector<int> AStarPlanner::getNeighbors(int id)
	{
		std::vector<int> neighbors;
		int current_id = id;
		unsigned int x, z;
		gSpatialDatabase->getGridCoordinatesFromIndex(current_id, x, z);
		int x_range_min, x_range_max, z_range_min, z_range_max;


		x_range_min = MAX(x - GRID_STEP, 0);
		x_range_max = MIN(x + GRID_STEP, gSpatialDatabase->getNumCellsX());

		z_range_min = MAX(z - GRID_STEP, 0);
		z_range_max = MIN(z + GRID_STEP, gSpatialDatabase->getNumCellsZ());


		for (int i = x_range_min; i <= x_range_max; i += GRID_STEP)
		{
			for (int j = z_range_min; j <= z_range_max; j += GRID_STEP)
			{
				if (ifOntheGrid(i, j)) {
					int index = gSpatialDatabase->getCellIndexFromGridCoords(i, j);
					if (canBeTraversed(index)) {
						if (index != id) {
							neighbors.push_back(index);
						}
					}
				}


			}
		}

		return neighbors;
	}

	bool AStarPlanner::ifOntheGrid(unsigned int i, unsigned int j) {
		float xMin = 0;
		float zMin = 0;
		float xMax = 200;
		float zMax = 200;


		if (i<xMin || i>xMax) {
			return false;
		}
		if (j<zMin || j>zMax) {
			return false;
		}
		return true;
	}

	Util::Point AStarPlanner::getPointFromGridIndex(int id)
	{
		Util::Point p;
		gSpatialDatabase->getLocationFromIndex(id, p);
		return p;
	}

	int AStarPlanner::getGridIndexFromPoint(Util::Point p) {
		int id;
		id = gSpatialDatabase->getCellIndexFromLocation(p);
		return id;
	}
	unsigned int parents[100000];
	bool AStarPlanner::computePath(std::vector<Util::Point>& agent_path, Util::Point start, Util::Point goal, SteerLib::SpatialDataBaseInterface * _gSpatialDatabase, bool append_to_path)
	{
		start.x = start.x + 0.5;
		goal.x = goal.x + 0.5;
		start.z = start.z + 0.5;
		goal.z = goal.z + 0.5;
		std::cout << "test" << std::endl;
		gSpatialDatabase = _gSpatialDatabase;
		std::cout << "\nIn A*";
		//return AnytimeAStar(agent_path, start, goal, _gSpatialDatabase, append_to_path);
		return AStarWeighted(agent_path, start, goal, _gSpatialDatabase, append_to_path);




	}
	double AStarPlanner::fvalue(AStarPlannerNode node, double e, Util::Point goal, double weight, int heuristic) {
		return node.g + e * calculateHeuristicValue(node.point, goal, weight, heuristic);
	}

	bool AStarPlanner::AnytimeAStar(std::vector<Util::Point>& agent_path, Util::Point start, Util::Point goal, SteerLib::SpatialDataBaseInterface* _gSpatialDatabase, bool append_to_path) {
		BinaryHeap fringe = BinaryHeap();
		int weight = 1;
		int heuristic = 1;
		AStarPlannerNode startNode(start, 0, 0, NULL);
		startNode.parent = &startNode;
		double e = 2.0;
		startNode.f = fvalue(startNode, e, goal, weight, heuristic);
		AStarPlannerNode goalNode(goal, INFINITY, INFINITY, NULL);
		std::vector<Util::Point> closed;
		std::vector<AStarPlannerNode> open;
		std::vector<AStarPlannerNode> incons;
		open.push_back(startNode);
		fringe.insert(startNode);

		ImprovePath(agent_path, fringe, closed, open, incons, goalNode, e, weight, heuristic, start);
		double minNum = fvalue(open[0], 1, goal, weight, heuristic);
		for (int i = 0; i < open.size(); i++) {
			double temp = fvalue(open[i], 1, goal, weight, heuristic);
			if (temp < minNum) {
				minNum = temp;
			}
		}

		for (int i = 0; i < incons.size(); i++) {
			double temp = fvalue(incons[i], 1, goal, weight, heuristic);
			if (temp < minNum) {
				minNum = temp;
			}
		}

		minNum = goalNode.g / minNum;

		double nE = min(e, minNum);
		std::cout << "e " << e << " minNUm: " << minNum << std::endl;
		while (nE > 1.0) {

			/*
			for (int i = 0; i < 100000; i++) {
			parents[i] = 0;
			}
			*/
			e -= 0.1;

			for (int i = 0; i < incons.size(); i++) {
				bool flagIncons = false;
				for (int j = 0; j < open.size(); j++) {
					if (open[j] == incons[i]) {
						flagIncons = true;
					}
				}
				if (!flagIncons) {
					open.push_back(incons[i]);
					fringe.insert(incons[i]);
				}
			}
			//incons.clear();
			for (int i = 0; i < open.size(); i++) {
				open[i].f = fvalue(open[i], e, goal, weight, heuristic);
				fringe.update(open[i]);
			}

			closed.clear();
			ImprovePath(agent_path, fringe, closed, open, incons, goalNode, e, weight, heuristic, start);
			double minNum = fvalue(open[0], 1, goal, weight, heuristic);
			for (int i = 0; i < open.size(); i++) {
				double temp = fvalue(open[i], 1, goal, weight, heuristic);
				if (temp < minNum) {
					minNum = temp;
				}
			}

			for (int i = 0; i < incons.size(); i++) {
				double temp = fvalue(incons[i], 1, goal, weight, heuristic);
				if (temp < minNum) {
					minNum = temp;
				}
			}

			minNum = goalNode.g / minNum;

			nE = min(e, minNum);
			std::cout << "e " << e << " minNUm: " << minNum << std::endl;

		}
		return true;
	}

	void AStarPlanner::ImprovePath(std::vector<Util::Point> & agent_path, BinaryHeap & fringe, std::vector<Util::Point> & closed, std::vector<AStarPlannerNode>& open, std::vector<AStarPlannerNode>& incons, AStarPlannerNode & goal, double e, double weight, int heuristic, Util::Point start) {
		while (fvalue(goal, e, goal.point, weight, heuristic) > fvalue(fringe.peek(), e, goal.point, weight, heuristic)) {
			AStarPlannerNode node = fringe.popMin();
			closed.push_back(node.point);



			for (int i = 0; i < open.size(); i++) {
				if (open[i].point == node.point) {
					open.erase(open.begin() + i);
					break;
				}
			}

			int id = getGridIndexFromPoint(node.point);
			//std::cout << "1" << std::endl;
			std::vector<int> neighbors = getNeighbors(id);
			//std::cout << "2" << std::endl;
			if (node == goal) {
				agent_path.clear();
				goal.g = node.g;
				goal.f = fvalue(node, e, goal.point, weight, heuristic);
				goal.parent = node.parent;

				int pID = getGridIndexFromPoint(node.parent->point);
				int ppID = parents[pID];
				//std::cout << "goal's parent.parent is " << getPointFromGridIndex(ppID) << std::endl;

				//std::cout << "goal's parent.parent = " << node.parent->parent->point << std::endl;
				//return true;
				std::vector<Util::Point> tempPath;
				tempPath.push_back(node.point);
				int count = 0;
				while (getPointFromGridIndex(pID) != start) {
					tempPath.push_back(getPointFromGridIndex(pID));
					pID = parents[pID];


					//tempPath.push_back(*node.parent);
					//node = *node.parent;
					std::cout << "node is " << getPointFromGridIndex(pID) << std::endl;

				}
				tempPath.push_back(start);


				for (int i = tempPath.size() - 1; i >= tempPath.size() / 2; i--) {
					std::swap(tempPath[i], tempPath[tempPath.size() - 1 - i]);
				}

				for (int j = 0; j < tempPath.size(); j++) {
					agent_path.push_back(tempPath[j]);
					//std::cout << "test" << std::endl;
				}

				return;


			}

			for (int i = 0; i < neighbors.size(); i++) {
				bool flagOpen = false;
				bool flagClosed = false;
				Util::Point nPoint = getPointFromGridIndex(neighbors[i]);
				AStarPlannerNode nNode(nPoint, 0, 0, NULL);

				for (int j = 0; j < open.size(); j++) {
					if ((nPoint.x == open[j].point.x) && (nPoint.z == open[j].point.z)) {
						flagOpen = true;
						break;
					}
				}

				if (!flagOpen) {
					nNode.g = INFINITY;
				}
				else {
					nNode = fringe.find(nPoint);
				}
				double traversal_cost = gSpatialDatabase->getTraversalCost(neighbors[i]);
				if (nNode.g > node.g + traversal_cost) {
					nNode.g = node.g + traversal_cost;
					nNode.parent = &node;




					for (int j = 0; j < closed.size(); j++) {
						if ((nPoint.x == closed[j].x) && (nPoint.z == closed[j].z)) {
							flagClosed = true;
							break;
						}
					}

					if (!flagClosed) {
						nNode.f = fvalue(nNode, e, goal.point, weight, heuristic);
						fringe.insert(nNode);
						open.push_back(nNode);
						int index = getGridIndexFromPoint(nNode.point);
						parents[index] = getGridIndexFromPoint(node.point);
					}
					else {
						incons.push_back(nNode);
					}
				}







			}

		}
	}

	bool AStarPlanner::AStarWeighted(std::vector<Util::Point>& agent_path, Util::Point start, Util::Point goal, SteerLib::SpatialDataBaseInterface* _gSpatialDatabase, bool append_to_path) {
		BinaryHeap fringe = BinaryHeap();
		//TODO

		double weight = 1.1;
		int heuristic = 1;
		AStarPlannerNode startNode(start, 0, calculateHeuristicValue(start, goal, weight, heuristic), NULL);
		startNode.parent = &startNode;
		std::vector<Util::Point> closed;
		std::vector<AStarPlannerNode> open;
		open.push_back(startNode);
		bool flag;
		bool flag2;

		fringe.insert(startNode);

		while (fringe.size > 0) {

			AStarPlannerNode node = fringe.popMin();
			std::cout << "current node = " << node.point << std::endl;
			for (int i = 0; i < open.size(); i++) {
				if (open[i].point == node.point) {
					open.erase(open.begin() + i);
					break;
				}
			}

			if (node.point == goal) {
				std::cout << "find the goal" << std::endl;
				std::cout << "goal's parent = " << node.parent->point << std::endl;

				int pID = getGridIndexFromPoint(node.parent->point);
				int ppID = parents[pID];
				std::cout << "goal's parent.parent is " << getPointFromGridIndex(ppID) << std::endl;

				//std::cout << "goal's parent.parent = " << node.parent->parent->point << std::endl;
				//return true;
				std::vector<Util::Point> tempPath;
				tempPath.push_back(node.point);
				int count = 0;
				while (getPointFromGridIndex(pID) != start) {
					tempPath.push_back(getPointFromGridIndex(pID));
					pID = parents[pID];


					//tempPath.push_back(*node.parent);
					//node = *node.parent;
					std::cout << "node is " << getPointFromGridIndex(pID) << std::endl;

				}
				tempPath.push_back(start);


				for (int i = tempPath.size() - 1; i >= tempPath.size() / 2; i--) {
					std::swap(tempPath[i], tempPath[tempPath.size() - 1 - i]);
				}

				for (int j = 0; j < tempPath.size(); j++) {
					tempPath[j].x += 0.5;
					tempPath[j].z += 0.5;
					agent_path.push_back(tempPath[j]);
					//std::cout << "test" << std::endl;
				}
				for (int i = 0; i < agent_path.size(); i++) {
					//std::cout << "node = " << agent_path[i] << std::endl;
				}
				return true;

			}

			closed.push_back(node.point);

			int id = getGridIndexFromPoint(node.point);
			//std::cout << "1" << std::endl;
			std::vector<int> neighbors = getNeighbors(id);
			//std::cout << "2" << std::endl;

			for (int i = 0; i < neighbors.size(); i++) {
				flag = true;
				flag2 = true;
				Util::Point nPoint = getPointFromGridIndex(neighbors[i]);
				for (int j = 0; j < closed.size(); j++) {
					if ((nPoint.x == closed[j].x) && (nPoint.z == closed[j].z)) {
						flag = false;
						if (!flag) {
							std::cout << nPoint << " is in the closed" << std::endl;
						}
						break;
					}
				}
				if (flag) {
					AStarPlannerNode nNode(node.point, 0, 0, NULL);
					for (int j = 0; j < open.size(); j++) {
						if (nPoint == open[j].point) {
							nNode = open[j];
							flag2 = false;
							break;
						}
					}
					if (flag2) {
						nNode = AStarPlannerNode(nPoint, INFINITY, 0, NULL);
						open.push_back(nNode);
					}
					else {
						nNode = fringe.find(nPoint);
					}
					AStarPlannerNode neighborNode(nPoint, INFINITY, 0, NULL);
					neighborNode = UpdateNode(node, nNode, goal, flag2, fringe, open, weight, heuristic);
					nNode = neighborNode;

					AStarPlannerNode temp = AStarPlannerNode(nPoint, INFINITY, 0, NULL);
					temp = *nNode.parent;
					int pID = getGridIndexFromPoint(temp.point);
					int ppID = parents[pID];







					//std::cout << "parent.parent is " << getPointFromGridIndex(ppID) << std::endl;
					//std::cout << "fringe.size = " << fringe.size << std::endl;



					//std::cout << "parent is " << nNode.parent->point << std::endl;
					//AStarPlannerNode temp = AStarPlannerNode(nPoint, INFINITY, 0, NULL);
					//temp = *nNode.parent;
					//std::cout << "parent.parent is " << temp.parent->point << std::endl;
				}
			}


		}
		return false;
	}

	AStarPlannerNode AStarPlanner::UpdateNode(AStarPlannerNode node, AStarPlannerNode nNode, Util::Point goal, bool flag2, BinaryHeap& fringe, std::vector<AStarPlannerNode> & open, double weight, int heuristic) {

		//std::cout << "update the point" << nNode.point << std::endl;
		int index = getGridIndexFromPoint(nNode.point);
		double traversal_cost = gSpatialDatabase->getTraversalCost(index);
		//double traversal_cost = distanceBetween(node.point, nNode.point);
		if (node.g + traversal_cost < nNode.g) {
			nNode.g = node.g + traversal_cost;
			nNode.f = nNode.g + calculateHeuristicValue(nNode.point, goal, weight, heuristic);
			nNode.parent = &node;
			parents[index] = getGridIndexFromPoint(node.point);
			if (!flag2) {
				fringe.update(nNode);

				//std::cout << "update" << std::endl;
			}
			else {
				//std::cout << "insert" << std::endl;
				fringe.insert(nNode);
				//open.push_back(nNode);
				//std::cout << "insert" << std::endl;
			}

		}
		return nNode;
	}

	double AStarPlanner::calculateHeuristicValue(Util::Point current, Util::Point end, double W, int h) {
		if (h == 1) {
			return W*(abs(current.x - end.x) + abs(current.z - end.z));
		}
		else if (h == 2) {
			return W*sqrt((current.x - end.x)*(current.x - end.x) + (current.z - end.z)*(current.z - end.z));
		}
	}




}