#pragma once
#include <vector>

#include "EuclideanGraph.hpp"
#include "SimpleVector2D.hpp"

namespace fluczak::Ai
{
    class NavMeshPath
    {
    public:
        NavMeshPath() = default;
        NavMeshPath(const std::vector<std::reference_wrapper<const Ai::Node>>& nodesToSet);
        NavMeshPath(const std::vector<VectorMath::SimpleVector2D<float>>& pointsToSet);

        //Given a point return a t value of how far along the path the given point is
        float GetPercentageAlongPath(const VectorMath::SimpleVector2D<float>& point) const;
        //get an arbitrary closest point on the path to a given point 
        VectorMath::SimpleVector2D<float> GetClosestPointOnPath(const VectorMath::SimpleVector2D<float>& point) const;
        //Given a t value- return a point in this percentage along the path
        VectorMath::SimpleVector2D<float> FindPointOnPath(float t) const;

        std::vector<VectorMath::SimpleVector2D<float>>& GetPoints() { return points; }
        std::vector<const Ai::Node*>& GetGraphNodes() { return aiNodes; }
        bool IsEmpty() const { return points.empty(); }

    private:
        std::vector<VectorMath::SimpleVector2D<float>> points;
        std::vector<const Ai::Node*> aiNodes;
    };
}
