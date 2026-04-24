#include "EuclideanGraph.hpp"

fluczak::Ai::Node::Node(const Node& node) : position(node.position)
{

}

fluczak::Ai::Node::Node(const VectorMath::SimpleVector2D<float>& pos) : position(pos)
{
}

fluczak::Ai::Edge::Edge(const Edge& edge) : a(edge.a), b(edge.b), cost(edge.cost)
{
}

fluczak::Ai::Edge::Edge(Node& nodeA, Node& nodeB, float newCost) : a(nodeA), b(nodeB), cost(newCost)
{
}

fluczak::Ai::EuclideanGraph::EuclideanGraph()= default;

fluczak::Ai::EuclideanGraph::EuclideanGraph(const std::vector<VectorMath::SimpleVector2D<float>>& nodeData,const std::vector<EdgeData>& edgeData)
{
    for (const auto& node : nodeData)
    {
        nodes.emplace_back(node);
    }

    for (const auto& potentialEdge : edgeData)
    {
        edges.emplace_back(nodes[potentialEdge.nodeIndexOne], nodes[potentialEdge.nodeIndexTwo], potentialEdge.cost);
        nodeEdgeAssociations[&nodes[potentialEdge.nodeIndexOne]].push_back(edges.size() - 1);
    }

}

fluczak::Ai::EuclideanGraph::EuclideanGraph(const std::vector<Node>& nodesToSet, const std::vector<Edge>& edgesToSet)
{
    int index = 0;
    for (const Node& node : nodesToSet)
    {
        nodes.emplace_back(node);
        for (Edge const& edge : edgesToSet)
        {
            if (&edge.a != &node) continue;
            edges.emplace_back(edge);
            nodeEdgeAssociations[&nodes[index]].push_back(edges.size() - 1);
        }
        index++;
    }
}

fluczak::Ai::EuclideanGraph::EuclideanGraph(const std::vector<Node>& nodesToSet, const std::vector<EdgeData>& edgeData)
{
    for (const Node& node : nodesToSet)
    {
        nodes.push_back(node);
    }

    if (nodesToSet.empty()) return;
    for (const EdgeData& potentialEdge : edgeData)
    {
        edges.emplace_back(nodes[potentialEdge.nodeIndexOne], nodes[potentialEdge.nodeIndexTwo], potentialEdge.cost);
        nodeEdgeAssociations[&nodes[potentialEdge.nodeIndexOne]].push_back(edges.size() - 1);
    }
}

void fluczak::Ai::EuclideanGraph::InitializeEuclideanWeights()
{
    if (nodes.empty()) return;

    for (Edge& edge : edges)
    {
        edge.SetCost(VectorMath::SimpleVector2D<float>::Length(edge.a.position - edge.b.position));
    }
}

void fluczak::Ai::EuclideanGraph::DebugDrawGraph() const
{

}

void fluczak::Ai::EuclideanGraph::AddNodes(const std::vector<Node>& nodesToAdd)
{
    for (const Node& node : nodesToAdd)
    {
        nodes.push_back(node);
    }
}

void fluczak::Ai::EuclideanGraph::AddNodesAndEdges(const   std::vector<Node>&nodesToAdd,
	const std::vector<Edge>& edgesToSet)
{
    int index = 0;
    for (const Node& node : nodesToAdd)
    {
        nodes.emplace_back(node);
        for (Edge const& edge : edgesToSet)
        {
            if (&edge.a != &node) continue;
        	edges.emplace_back(edge);
            nodeEdgeAssociations[&nodes[index]].push_back(edges.size() - 1);
        }
        index++;
    }
}

void fluczak::Ai::EuclideanGraph::AddNodesAndEdges(const std::vector<Node>& nodesToAdd,
	const std::vector<EdgeData>& edgesToSet)
{
    for (const Node& node : nodesToAdd)
    {
        nodes.push_back(node);
    }

    if (nodesToAdd.empty()) return;
    for (const EdgeData& potentialEdge : edgesToSet)
    {
        edges.emplace_back(nodes[potentialEdge.nodeIndexOne], nodes[potentialEdge.nodeIndexTwo], potentialEdge.cost);
        nodeEdgeAssociations[&nodes[potentialEdge.nodeIndexOne]].push_back(edges.size() - 1);
    }
}

std::vector<fluczak::Ai::Edge> fluczak::Ai::EuclideanGraph::GetAssociatedEdges(const Node& node) const
{
    std::vector<fluczak::Ai::Edge> toReturn{};

    for (const auto index : nodeEdgeAssociations.at(&node))
    {
        toReturn.push_back(edges[index]);
    }

    return toReturn;
}

