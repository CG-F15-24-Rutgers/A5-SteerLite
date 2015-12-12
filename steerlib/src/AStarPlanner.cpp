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
	struct compare : public std::binary_function<AStarPlannerNode*, AStarPlannerNode*, bool>
	{
		bool operator()(const AStarPlannerNode* l, const AStarPlannerNode* r) const
		{
			//std::cout << l->point.x << " " << l->point.z << " " << l->f << " " << r->point.x << " " << r->point.z << " " << r->f << std::endl;
			if (l->f == r->f)
			{
				return l->g < r->g; //higher g values
									//return l->g > r->g; // lower g values
			}
			return l->f > r->f;
		}
	};

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



	Util::Point AStarPlanner::getPointFromGridIndex(int id)
	{
		Util::Point p;
		gSpatialDatabase->getLocationFromIndex(id, p);
		return p;
	}

	double AStarPlanner::heuristic_cost_estimate(Util::Point start, Util::Point goal, int type)
	{
		double h = 0;
		int e = 1;
		switch (type) {
		case heuristic::euclidian:
			h = sqrt((start.x - goal.x)*(start.x - goal.x) + (start.z - goal.z)*(start.z - goal.z));
			break;
		case heuristic::manhattan:
			h = abs(start.x - goal.x) + abs(start.z - goal.z);
			break;
		default:
			break;
		}
		return h*(e);
	}


	std::vector<AStarPlannerNode> AStarPlanner::neighbor_nodes(AStarPlannerNode* current, Util::Point goal, SteerLib::GridDatabase2D * _gSpatialDatabase, int type)
	{
		std::vector<AStarPlannerNode> neighbor;
		Util::Point pt;
		int id = 0;

		for (double x = current->point.x - 1; x <= current->point.x + 1; x++)
		{
			for (double z = current->point.z - 1; z <= current->point.z + 1; z++)
			{
				//int xC = abs(current->point.x - x);
				//int yC = abs(current->point.z - z);

				if ((current->point.z == z && current->point.x == x))
					continue;
				id = _gSpatialDatabase->getCellIndexFromLocation(x, z);
				gSpatialDatabase->getLocationFromIndex(id, pt);

				if (canBeTraversed(id))
				{
					int g = 1;
					if (current->point.z != z && current->point.x != x)
					{
						g = 15;
					}
					neighbor.push_back(AStarPlannerNode(pt, current->g + 1, (current->g + 1 + heuristic_cost_estimate(pt, goal, type)), current));
				}
			}
		}

		return neighbor;
	}
	bool AStarPlanner::computePath(std::vector<Util::Point>& agent_path, Util::Point start, Util::Point goal, SteerLib::GridDatabase2D * _gSpatialDatabase, bool append_to_path)
	{
		gSpatialDatabase = _gSpatialDatabase;
		//int type = heuristic::manhattan;
		int type = heuristic::manhattan;
		std::vector<AStarPlannerNode *> Close;
		std::vector<Util::Point> OpenCheck;
		std::priority_queue<AStarPlannerNode*, std::vector<AStarPlannerNode*>, compare> Open;

		Open.push(new AStarPlannerNode(start, 0, 0 + heuristic_cost_estimate(start, goal, type), NULL));

		OpenCheck.push_back(start);
		AStarPlannerNode *current = NULL;
		int nodesExpanded = 0;
		while (!Open.empty())
		{
			current = Open.top();
			//std::cout << "current" << current->point.x << " " << current->point.z  << std::endl;
			Open.pop();
			OpenCheck.erase(std::remove(OpenCheck.begin(), OpenCheck.end(), current->point));
			nodesExpanded++;
			Close.push_back(current);
			if (abs(current->point.x - goal.x) == 0.5 && abs(current->point.z - goal.z) == 0.5)
			{
				std::cout << "Number of nodes expanded" << nodesExpanded;
				reconstruct_path(current, agent_path);
				return true;
			}
			bool check = false;
			std::vector<AStarPlannerNode> neighbour = neighbor_nodes(current, goal, gSpatialDatabase, type);

			std::vector<AStarPlannerNode>::iterator itr;
			for (itr = neighbour.begin(); itr != neighbour.end(); ++itr)
			{
				std::vector<AStarPlannerNode*>::iterator itrClosedList;
				for (itrClosedList = Close.begin(); itrClosedList != Close.end(); itrClosedList++)
				{
					if (itr->point == (*itrClosedList)->point)
					{
						check = true;
						break;
					}
				}
				std::vector<Util::Point>::iterator itrOpenList;
				for (itrOpenList = OpenCheck.begin(); itrOpenList != OpenCheck.end(); itrOpenList++)
				{
					if (itr->point == *itrOpenList)
					{
						check = true;
						break;
					}
				}
				if (!check)
				{
					Open.push(new AStarPlannerNode(itr->point, itr->g, itr->f, itr->parent));
					OpenCheck.push_back(itr->point);
				}
				if (check)
				{
					check = false;
				}

			}
		}
		return false;
	}
	void AStarPlanner::reconstruct_path(AStarPlannerNode *current, std::vector<Util::Point>& agent_path)
	{
		if (current == NULL)
			return;
		reconstruct_path(current->parent, agent_path);
		agent_path.push_back(current->point);
	}
}