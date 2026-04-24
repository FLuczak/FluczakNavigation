#pragma once
#include <unordered_map>
#include <vector>

#include "EuclideanGraph.hpp"
#include "SimpleVector2D.hpp"

namespace fluczak::Ai
{

	class AStar
	{
    public:
        //Heuristic for traversing graphs without euclidean distances between nodes
        static float ZeroHeuristic(const Ai::Node& a, const Ai::Node& b) { return 0.0f; };
        //Heuristic for traversing graphs with euclidean distances between nodes
        static float DistanceHeuristic(const Ai::Node& a, const Ai::Node& b) { return VectorMath::SimpleVector2D<float>::Length(a.position - b.position); };

        /**
		 \brief A method that while provided a start node from the graph, end node from the graph and a heuristic function (To chose from ZeroHeuristic- mostly used for non euclidean space graphs and DistanceHeuristic- mostly used for graphs representing euclidean spaces)
		 * \param graph - A graph the nodes are in
		 * \param start - A Start node from the graph
		 * \param goal - An end node from the graph
		 * \param heuristicFunction - A provided heuristic function (ZeroHeuristic or DistanceHeuristic)
		 * \return - A list of node pointers. If the path doesn't exist- the list is empty
		 */
        static std::vector<std::reference_wrapper<const fluczak::Ai::Node>> FindPath(const EuclideanGraph& graph, const Ai::Node& start, const Ai::Node& goal, float (*heuristicFunction)(const Ai::Node&, const Ai::Node&));
        static void DebugDrawPath(const std::vector<const Ai::Node*>& path);
        static void DebugDrawPath(const std::vector<VectorMath::SimpleVector2D<float>>& path);
    private:
        /**
         * \brief A struct that is created during A* search. It contains
         * methods and variables that make A* search possible.
         */
        struct AStarNode
        {
            AStarNode(Ai::Node* node) : graphNode(node) {}
            AStarNode(const Ai::Node* node) : graphNode(const_cast<Ai::Node*>(node)) {}

            float GetF() const { return g + h; }
            void SetG(float newG) { g = newG; }
            void SetHeuristic(const float newH) { h = newH; }

        	Ai::Node* graphNode = nullptr;
            float g = 0;  // Cost from start node to this node
            float h = 0;  // Heuristic (estimated cost from this node to goal node)
        };

        /**
         * \brief A struct used in priority queue for comparison methods
         */
        struct CompareNodes
        {
            bool operator()(const AStarNode& one, const AStarNode& two) const { return one.GetF() > two.GetF(); }
        };

        /**
		 * \brief Used for the FindPath method. Returns an actual path from Astar search result
		 * \param end - goal of the astar search
		 * \param predecessors - map of predecessors created by the Astar search
		 * \return - A valid path of nodes ready for use
		 */
        static std::vector<std::reference_wrapper<const fluczak::Ai::Node>> RetracePath(const AStarNode& end,
	        const std::unordered_map<const Ai::Node*, const Ai::Node*>&
	        predecessors);
    };
}
