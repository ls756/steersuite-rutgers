//
// Copyright (c) 2009-2015 Glen Berseth, Mubbasir Kapadia, Shawn Singh, Petros Faloutsos, Glenn Reinman, Rahul Shome
// See license.txt for complete license.
//


#ifndef __STEERLIB_A_STAR_PLANNER_H__
#define __STEERLIB_A_STAR_PLANNER_H__


#include <vector>
#include <stack>
#include <set>
#include <map>
#include "SteerLib.h"

namespace SteerLib
{

	/*
	@function The AStarPlannerNode class gives a suggested container to build your search tree nodes.
	@attributes
	f : the f value of the node
	g : the cost from the start, for the node
	point : the point in (x,0,z) space that corresponds to the current node
	parent : the pointer to the parent AStarPlannerNode, so that retracing the path is possible.
	@operators
	The greater than, less than and equals operator have been overloaded. This means that objects of this class can be used with these operators. Change the functionality of the operators depending upon your implementation

	*/
	class STEERLIB_API AStarPlannerNode {
	public:
		double f;
		double g;
		Util::Point point;
		AStarPlannerNode* parent;
		AStarPlannerNode(Util::Point _point, double _g, double _f, AStarPlannerNode* _parent)
		{
			f = _f;
			point = _point;
			g = _g;
			parent = _parent;
		}
		bool operator<(AStarPlannerNode other) const
		{
			return this->f < other.f;
		}
		bool operator>(AStarPlannerNode other) const
		{
			return this->f > other.f;
		}
		bool operator==(AStarPlannerNode other) const
		{
			return ((this->point.x == other.point.x) && (this->point.z == other.point.z));
		}

	};

	class STEERLIB_API BinaryHeap {
	public:
		int size;
		std::vector<AStarPlannerNode> heapList;
		BinaryHeap::BinaryHeap(void) {
			Point p(0, 0, 0);
			AStarPlannerNode start(p, 0, 0, NULL);
			heapList.push_back(start);
			size = 0;

		};

		AStarPlannerNode BinaryHeap::popMin() {
			AStarPlannerNode popped = heapList[1];
			heapList[1] = heapList[size];
			size -= 1;
			heapList.pop_back();
			heapDown(1);
			return popped;
		}

		AStarPlannerNode BinaryHeap::peek() {
			return heapList[1];

		}

		AStarPlannerNode BinaryHeap::find(Util::Point node) {
			for (int i = 1; i <= size; i++) {
				if (heapList[i].point == node) {
					return heapList[i];
				}
			}
			AStarPlannerNode temp(node, 0, 0, NULL);
			return temp;
		}

		void BinaryHeap::insert(AStarPlannerNode node) {
			heapList.push_back(node);
			//std::cout << "insert succeed, point is " << node.point << std::endl;
			size += 1;
			heapUp(size);
			//std::cout << "heapUp succeed" << std::endl;
		}

		void BinaryHeap::update(AStarPlannerNode node) {
			for (int i = 1; i <= size; i++) {
				if (heapList[i] == node) {
					heapList[i].g = node.g;
					heapList[i].f = node.f;
					//heapList[i].parent = node.parent;

					heapUp(i);
					break;
				}
			}
		}

	private:
		void BinaryHeap::heapUp(int index) {
			while (index / 2 > 0) {
				if ((heapList[index].f == heapList[index / 2].f) && (heapList[index].g <= heapList[index / 2].g)) {
					index = index / 2;
					continue;
				}
				if (heapList[index] < heapList[index / 2]) {
					AStarPlannerNode temp = heapList[index / 2];
					heapList[index / 2] = heapList[index];
					heapList[index] = temp;
					//delete &temp;
				}
				index = index / 2;
			}
		}

		int BinaryHeap::minChild(int index) {
			if (index * 2 + 1 > size) {
				return index * 2;
			}
			else if ((heapList[index * 2].f == heapList[index * 2 + 1].f) && (heapList[index * 2].g <= heapList[index * 2 + 1].g)) {
				return index * 2 + 1;
			}
			else if (heapList[index * 2] < heapList[index * 2 + 1]) {
				return index * 2;
			}
			else {
				return index * 2 + 1;
			}
		}

		void BinaryHeap::heapDown(int index) {
			while (index * 2 <= size) {
				int min = minChild(index);

				if ((heapList[index].f == heapList[min].f) && (heapList[index].g > heapList[min].g)) {
					index = min;
					continue;
				}

				if (heapList[index] > heapList[min]) {
					AStarPlannerNode temp = heapList[index];
					heapList[index] = heapList[min];
					heapList[min] = temp;
					//delete &temp;
				}
				index = min;
			}
		}
	};

	class STEERLIB_API AStarPlanner {
	public:
		AStarPlanner();
		~AStarPlanner();
		// NOTE: There are four indices that need to be disambiguated
		// -- Util::Points in 3D space(with Y=0)
		// -- (double X, double Z) Points with the X and Z coordinates of the actual points
		// -- (int X_GRID, int Z_GRID) Points with the row and column coordinates of the GridDatabase2D. The Grid database can start from any physical point(say -100,-100). So X_GRID and X need not match
		// -- int GridIndex  is the index of the GRID data structure. This is an unique id mapping to every cell.
		// When navigating the space or the Grid, do not mix the above up

		/*
		@function canBeTraversed checkes for a OBSTACLE_CLEARANCE area around the node index id for the presence of obstacles.
		The function finds the grid coordinates for the cell index  as (X_GRID, Z_GRID)
		and checks cells in bounding box area
		[[X_GRID-OBSTACLE_CLEARANCE, X_GRID+OBSTACLE_CLEARANCE],
		[Z_GRID-OBSTACLE_CLEARANCE, Z_GRID+OBSTACLE_CLEARANCE]]
		This function also contains the griddatabase call that gets traversal costs.
		*/
		bool canBeTraversed(int id);

		std::vector<int> AStarPlanner::getNeighbors(int id);
		/*
		@function getPointFromGridIndex accepts the grid index as input and returns an Util::Point corresponding to the center of that cell.
		*/
		Util::Point getPointFromGridIndex(int id);

		int getGridIndexFromPoint(Util::Point p);
		AStarPlannerNode AStarPlanner::UpdateNode(AStarPlannerNode node, AStarPlannerNode nNode, Util::Point goal, bool flag2, BinaryHeap& fringe, std::vector<AStarPlannerNode> & open, double weight, int heuristic);
		double AStarPlanner::calculateHeuristicValue(Util::Point current, Util::Point end, double W, int h);
		bool ifOntheGrid(unsigned int i, unsigned int j);
		double fvalue(AStarPlannerNode node, double e, Util::Point goal, double weight, int heuristic);
		/*
		@function computePath
		DO NOT CHANGE THE DEFINITION OF THIS FUNCTION
		This function executes an A* query
		@parameters
		agent_path : The solution path that is populated by the A* search
		start : The start point
		goal : The goal point
		_gSpatialDatabase : The pointer to the GridDatabase2D from the agent
		append_to_path : An optional argument to append to agent_path instead of overwriting it.
		*/
		void AStarPlanner::ImprovePath(std::vector<Util::Point> & agent_path, BinaryHeap & fringe, std::vector<Util::Point> & closed, std::vector<AStarPlannerNode>& open, std::vector<AStarPlannerNode>& incons, AStarPlannerNode &goal, double e, double weight, int heuristic, Util::Point start);
		bool AStarPlanner::AnytimeAStar(std::vector<Util::Point>& agent_path, Util::Point start, Util::Point goal, SteerLib::SpatialDataBaseInterface* _gSpatialDatabase, bool append_to_path);
		bool AStarPlanner::AStarWeighted(std::vector<Util::Point>& agent_path, Util::Point start, Util::Point goal, SteerLib::SpatialDataBaseInterface* _gSpatialDatabase, bool append_to_path);
		bool computePath(std::vector<Util::Point>& agent_path, Util::Point start, Util::Point goal, SteerLib::SpatialDataBaseInterface * _gSpatialDatabase, bool append_to_path = false);
	private:
		SteerLib::SpatialDataBaseInterface * gSpatialDatabase;
	};


}


#endif