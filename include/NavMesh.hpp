#pragma once

#pragma warning(push)
#pragma warning(disable : 4267)
#include "clipper.h"
#pragma warning(pop)

#include <array>
#include <Triangulation.h>

#include "EuclideanGraph.hpp"
#include "NavMeshPath.hpp"
#include "SimplePolygon2D.hpp"
#include "CDT/CDTUtils.h"
#include "clipper2/clipper.core.h"

namespace fluczak::Ai
{
    /**
     * \brief A class for handling navmesh sampling- it contains information if a sample was successful and
     * a potential node if it was. IMPORTANT: node is a nullptr if the sample has not hit a place on a navmesh
     */
    class NavMeshSample
    {
    public:
        NavMeshSample(bool didHit, const Ai::Node* hitNode) : hit(didHit), node(hitNode) {};
        bool DidHit() const { return hit; }
        const Ai::Node* GetHitNode() const { return node; }
    private:
        bool hit = false;
        const Ai::Node* node{};
    };

    /**
     * \brief Main nav mesh class. After it is initialized from given geometry and baked
     * path finding and navigation.
     */
    class NavMesh
    {
    public:
        NavMesh(float agentRadius);
        NavMesh(float agentRadiusToSet, const std::vector<fluczak::VectorMath::SimplePolygon2D>& floor, const std::vector<fluczak::VectorMath::SimplePolygon2D>& obstacles);

        //Getter function for the underlying graph
        const Ai::EuclideanGraph& GetGraph() const { return graph; }
        //Getter function for if the navmesh was baked properly.
        bool IsBaked() const { return isBaked; }

        //A function that takes a path to the graph and creates paths for obstacles and walkable floors
        //Main initialization function- triangulates the mesh and generates the graph
        void Bake();
        //Draw the navmesh as well as the underlying graph
        void DebugDraw() const;
        //Check if a given position is on navmesh- returned object is a navmeshsample
        NavMeshSample SamplePosition(const fluczak::VectorMath::SimpleVector2D<float>& position) const;

        //Find a path from a node to another node
        NavMeshPath FindPath(const Ai::Node& start, const Ai::Node& goal)const;
        //Find a path from a world position to another position
        NavMeshPath FindPath(fluczak::VectorMath::SimpleVector2D<float> startPos, fluczak::VectorMath::SimpleVector2D<float> endPos) const;
        //Returns a path after applying a funnel algorithm to it
        std::vector<fluczak::VectorMath::SimpleVector2D<float>> FunnelPath(NavMeshPath path) const;
        // Given a point returns the closest navmesh node point
        fluczak::VectorMath::SimpleVector2D<float> FindClosestPointOnNavMesh(fluczak::VectorMath::SimpleVector2D<float> point) const;
    private:
        /**
         * \brief a class representing a triangle in the navmesh: it has some extra data
         * for handling the graph and creates a bridge between CDT triangulation and Navigation api
         */
        struct Triangle
        {
            Triangle(const CDT::Triangle& triangle) : cdtTriangle(triangle) {};
            std::array<fluczak::VectorMath::SimpleVector2D<float>, 3> vertices{};
            CDT::Triangle cdtTriangle;
            int nodeIndex = 0;
        };

        /**
         * \brief A struct representing a portal used for the funnel algorithm. It consists of the
         * left and right points of the portal.
         */
        struct Portal
        {
            Portal(fluczak::VectorMath::SimpleVector2D<float> right, fluczak::VectorMath::SimpleVector2D<float> left) : rightPoint(right), leftPoint(left) {};
            fluczak::VectorMath::SimpleVector2D<float> rightPoint;
            fluczak::VectorMath::SimpleVector2D<float> leftPoint;
        };

        // BAKING:

        void RemoveHolesFromPaths(Clipper2Lib::PathsD& levelPaths);
        void BakeFloorIntoSummarizedPath(Clipper2Lib::PathsD& levelPaths);
        // Given a summarized level path- differentiate obstacles path from the floors
        void BakeObstaclesIntoSummarizedPath(std::vector<Clipper2Lib::PathsD>& levelPaths, float offset) const;
        // Separate floors divided by obstacles
        void HandleSeparatedFloorsByObstacles(Clipper2Lib::PathsD& path, std::vector<Clipper2Lib::PathsD>& levelPaths) const;

        // Find all portals along a path- used for he funnel algorithm
        std::vector<Portal> FindPortalsAlongPath(NavMeshPath& path) const;
        // Check if a point is on the left from a line
        static bool IsLeft(fluczak::VectorMath::SimpleVector2D<float> startPoint,
                           fluczak::VectorMath::SimpleVector2D<float> endPoint,
                           fluczak::VectorMath::SimpleVector2D<float> point);
        // TRIANGULATION:
        // collective triangulation method that calls all the methods below
        void TriangulateMesh(std::vector<Clipper2Lib::PathsD>& paths);
        // Add vertices to the triangulation from previously created paths for both obstacles and floors
        static void AddTriangulationVertices(CDT::Triangulation<double>& cdt, const Clipper2Lib::PathsD& levelPath);
        // Add edges to the triangulation from previously created paths for both obstacles and floors
        static void AddTriangulationEdges(CDT::Triangulation<double>& cdt, const Clipper2Lib::PathsD& levelPath);
        // Initialize the triangles vector from the mesh triangulation result
        void InitializeTriangles(const CDT::Triangulation<double>& cdt);
        // Initialize graph from the triangles vector
        void InitializeGraph();
        // Make a mesh from previously created triangles- used in debugging
        void CreateTrianglesMesh();

        //Get a centroid of a triangle
        static fluczak::VectorMath::SimpleVector2D<float> GetTriangleCentroid(const Triangle& triangle);
        //Method that returns a glm vector2 from a Cdt vector2
        static fluczak::VectorMath::SimpleVector2D<float> VectorFromCDT(const CDT::V2d<double>& cdtVec);

        static float SignedArea(const fluczak::VectorMath::SimpleVector2D<float>& p1, const fluczak::VectorMath::SimpleVector2D<float>& p2, const fluczak::VectorMath::SimpleVector2D<float>& p3);
        static bool PointInTriangle(const fluczak::VectorMath::SimpleVector2D<float>& pt, const fluczak::VectorMath::SimpleVector2D<float>& v1, const fluczak::VectorMath::SimpleVector2D<float>& v2, const fluczak::VectorMath::SimpleVector2D<float>& v3);
        // Final mesh representing triangulated navmesh- final effect of baking
        Clipper2Lib::PathsD trianglesPath;
        // An underlying navmesh graph used for finding paths and navigation
        Ai::EuclideanGraph graph = Ai::EuclideanGraph();
        // All floors in the navmesh
        Clipper2Lib::PathsD floorPaths;
        // All obstacles in the navmesh
        Clipper2Lib::PathsD obstaclesPath;
        // Triangles
        std::vector<Triangle> triangles;
        // Is navmesh baked? If not, the pathfinding will not be performed
        bool isBaked = false;
        //How big is the agent- used for offseting the navmesh obstacles
        float agentRadius = 0.0f;
    };
}