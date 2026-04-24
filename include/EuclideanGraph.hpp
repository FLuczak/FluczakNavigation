#pragma once
#include <unordered_map>

#include "SimpleVector2D.hpp"
#include "vector"

namespace fluczak::Ai
{
    class Node
    {
    public:
        Node(const Node& node);
        Node(const VectorMath::SimpleVector2D<float>& pos);
        VectorMath::SimpleVector2D<float> position = VectorMath::SimpleVector2D<float>(0.0f, 0.0f);
    };

    class Edge
    {
    public:
        Edge(const Edge& edge);
        Edge(Node& nodeA, Node& nodeB, float newCost = 1.0f);
        float GetCost() const { return cost; }
        void SetCost(float newCost) { cost = newCost; }

        Node& a;
        Node& b;
    private:
        float cost = 1.0f;
    };

    /**
     * \brief A struct for creating edges from node indices from the graph
     */
    struct EdgeData
    {
        EdgeData(int nodeIndOne, int nodeIndTwo, float newCost) :nodeIndexOne(nodeIndOne), nodeIndexTwo(nodeIndTwo), cost(newCost) {}
        int nodeIndexOne = 0;
        int nodeIndexTwo = 0;
        float cost = 1.0f;
    };

    class EuclideanGraph
    {
    public:
	    /**
	     * \brief Construct empty graph. Used for dynamic adding of nodes to the graph.
	     */
	    EuclideanGraph();

        /**
		 * \brief Constructor for creating a graph out of only values- takes in a vector of node positions
		 *  and a tuple of node indices in the nodeData array + float
		 * \param nodeData - vector of node positions- each separate entry will be a separate node of the graph
		 * \param edgeData - a tuple of 3 entries- index of A node index of the edge, B node index of the edge and a weight
		 */
        EuclideanGraph(const std::vector<VectorMath::SimpleVector2D<float>>& nodeData, const std::vector<EdgeData>& edgeData);

        /**
		 * \brief Constructor for preconstructed nodes and edges
		 * \param nodesToSet - a vector of nodes that is going to be assinged to graph nodes
		 * \param edgesToSet - a vector of pre-constructed edges that is going to be assigned to graph edges
		 */
        EuclideanGraph(const std::vector<Node>& nodesToSet, const std::vector<Edge>& edgesToSet);

        /**
		 * \brief A graph constructor where a vector of preconstructed nodes is provided and and a tuple of node indices in the
		 * \nodeData array + float for edge weight
		 * \param nodesToSet - a vector of nodes that is going to be assinged to graph nodes
		 * \param edgeData -  A vector of edge data structures with 2 indices and a cost
		 */
        EuclideanGraph(const std::vector<Node>& nodesToSet, const std::vector<EdgeData>& edgeData);

        /**
		 * \brief A method for initializing weights in accordance to the distance between node vector2 positions
		 * It can be used during initialization of euclidean space graphs without provided edge weights
		 */
        void InitializeEuclideanWeights();

        /**
		 * \brief A method for drawing the graph inside the game world. Debug Drawer needs to be initialized first with a line drawing function pointer in     order to draw the graph.
		 */
        void DebugDrawGraph() const;

        /**
		 * \brief A function that pushes nodes to the graph after it was constructed
		 * \param nodesToAdd - a vector of nodes that are going to be added to the graph
		 */
        void AddNodes(const std::vector<Node>& nodesToAdd);

        /**
		 * \brief A function that adds nodes and assigns edges to them
		 * \param nodesToAdd - a vector of nodes to add to the graph
		 * \param edgesToSet - Edges that are going to be assigned to the nodes
		 */
        void AddNodesAndEdges(const std::vector<Node>& nodesToAdd, const std::vector<Edge>& edgesToSet);

        /**
		 * \brief A function that adds nodes and assigns edges to them
		 * \param nodesToAdd - A vector of nodes to add
		 * \param edgesToSet - A vector of edge data that will be later turned into edges and assigned to nodes
		 */
    	void AddNodesAndEdges(const std::vector<Node>& nodesToAdd, const std::vector<EdgeData>& edgesToSet);

        std::vector<Edge> GetAssociatedEdges(const Node& node) const;

    	const std::vector<Node>& GetNodes()const { return nodes; }
    private:
        std::vector<Node> nodes{};
        std::vector<Edge> edges{};

        std::unordered_map<const Node*, std::vector<int>> nodeEdgeAssociations;
    };

}  // namespace Ai
