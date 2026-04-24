# fluczak-navigation

A C++ navigation mesh library for game AI and pathfinding. Give it floor polygons and obstacles, get back a fully triangulated navmesh, an A* pathfinder, and a funnel algorithm that produces clean, straight-line paths.

---

## What it does

The library takes raw geometry (floor polygons, obstacle polygons) and builds a traversal graph from it using Constrained Delaunay Triangulation. From there you get:

- **Navmesh baking** from arbitrary 2D floor and obstacle geometry
- **Agent radius inflation** so obstacles are padded by the agent's physical size before triangulation
- **A* pathfinding** between any two navmesh nodes or world positions
- **Funnel algorithm** (string pulling) to turn raw triangle-hop paths into smooth waypoint lists
- **Position sampling** to test whether a world point lands on the navmesh
- **Closest point queries** to snap off-mesh positions back onto the mesh
- **Path interpolation** to find a position at any percentage along a path

---

## Architecture

```
fluczak::Ai             Core navigation classes
fluczak::VectorMath     2D vector math, template-based
fluczak::VectorMath::Utils  Geometric predicates and segment utilities
```

Three external libraries are bundled under `/external/`:

| Library | Purpose |
|---|---|
| CDT | Constrained Delaunay Triangulation for mesh generation |
| Clipper2 | Polygon boolean operations (union, difference) for obstacle inflation |
| Predicates | Robust geometric predicates for accurate point tests |

---

## Quick start

### 1. Define your geometry

```cpp
using namespace fluczak::Ai;
using fluczak::VectorMath::SimpleVector2D;
using SimplePolygon2D = std::vector<SimpleVector2D<float>>;

// A rectangular floor
SimplePolygon2D floor = {
    {0, 0}, {100, 0}, {100, 100}, {0, 100}
};

// A box obstacle in the middle
SimplePolygon2D obstacle = {
    {40, 40}, {60, 40}, {60, 60}, {40, 60}
};
```

### 2. Build and bake the navmesh

```cpp
float agentRadius = 2.0f;

NavMesh navMesh(agentRadius, {floor}, {obstacle});
navMesh.Bake();  // Triangulates the geometry and builds the graph
```

### 3. Find a path

```cpp
SimpleVector2D<float> start = {10, 10};
SimpleVector2D<float> goal  = {90, 90};

NavMeshPath path = navMesh.FindPath(start, goal);
```

### 4. Smooth it with the funnel algorithm

The raw path hops through triangle centroids. The funnel algorithm string-pulls it into straight-line waypoints:

```cpp
std::vector<SimpleVector2D<float>> waypoints = navMesh.FunnelPath(path);
```

### 5. Sample a position

```cpp
NavMeshSample sample = navMesh.SamplePosition({55, 55});

if (sample.DidHit()) {
    const Node* node = sample.GetHitNode();
    // point is on the navmesh, node is the triangle it landed in
}
```

---

## API reference

### `NavMesh`

The main class. Owns the geometry, the graph, and all query methods.

```cpp
// Construct empty, add geometry later
NavMesh(float agentRadius);

// Construct with geometry upfront
NavMesh(float agentRadius,
        const std::vector<SimplePolygon2D>& floors,
        const std::vector<SimplePolygon2D>& obstacles);

void Bake();       // Must be called before any queries
bool IsBaked() const;

const EuclideanGraph& GetGraph();

// Pathfinding
NavMeshPath FindPath(const Node& start, const Node& goal) const;
NavMeshPath FindPath(SimpleVector2D<float> start, SimpleVector2D<float> end) const;

// Path smoothing
std::vector<SimpleVector2D<float>> FunnelPath(NavMeshPath path) const;

// Position queries
NavMeshSample SamplePosition(const SimpleVector2D<float>& position) const;
SimpleVector2D<float> FindClosestPointOnNavMesh(SimpleVector2D<float> point) const;

void DebugDraw() const;
```

### `NavMeshPath`

Returned by `FindPath`. Holds both the raw graph nodes and the final world-space waypoints.

```cpp
bool IsEmpty() const;

std::vector<SimpleVector2D<float>>& GetPoints();
std::vector<const Node*>& GetGraphNodes();

// Path traversal utilities
float GetPercentageAlongPath(const SimpleVector2D<float>& point) const;
SimpleVector2D<float> GetClosestPointOnPath(SimpleVector2D<float> point) const;
SimpleVector2D<float> FindPointOnPath(float t) const;  // t in [0, 1]
```

### `NavMeshSample`

Result of `SamplePosition`.

```cpp
bool DidHit() const;
const Node* GetHitNode() const;
```

### `AStar`

The pathfinder. Used internally by `NavMesh::FindPath`, but available directly if you want to pathfind on your own graph.

```cpp
static std::vector<std::reference_wrapper<const Node>> FindPath(
    const EuclideanGraph& graph,
    const Node& start,
    const Node& goal,
    float (*heuristic)(const Node&, const Node&)
);

// Heuristics
static float ZeroHeuristic(const Node& a, const Node& b);      // Dijkstra-style
static float DistanceHeuristic(const Node& a, const Node& b);  // Standard A* Euclidean
```

### `EuclideanGraph`

The navigation graph. Nodes hold 2D positions, edges hold traversal costs.

```cpp
EuclideanGraph();
EuclideanGraph(const std::vector<SimpleVector2D<float>>& positions,
               const std::vector<EdgeData>& edges);
EuclideanGraph(const std::vector<Node>& nodes, const std::vector<Edge>& edges);

void InitializeEuclideanWeights();  // Set edge costs from node distances
void AddNodes(const std::vector<Node>& nodes);
void AddNodesAndEdges(const std::vector<Node>& nodes, const std::vector<Edge>& edges);

std::vector<Edge> GetAssociatedEdges(Node& node) const;
const std::vector<Node>& GetNodes() const;

void DebugDrawGraph() const;
```

### `SimpleVector2D<T>`

A lightweight 2D vector template. Works for any numeric type.

```cpp
SimpleVector2D<float> v(3.0f, 4.0f);

T Length() const;
static T Length(const SimpleVector2D& v);
static T Length2(const SimpleVector2D& v);        // Squared length
static T Distance2(const SimpleVector2D& a, const SimpleVector2D& b);

SimpleVector2D Normalized();
void Normalize();
float Dot(const SimpleVector2D& other);
static float StaticDot(const SimpleVector2D& a, const SimpleVector2D& b);

static SimpleVector2D<float> Lerp(const SimpleVector2D<float>& a,
                                   const SimpleVector2D<float>& b,
                                   float t);

// Arithmetic operators: +, -, *, /
// Comparison operators: ==, !=
```

### Geometry utilities

```cpp
namespace fluczak::VectorMath::Utils

bool DoSegmentsIntersect(SimpleVector2D<float> a1, SimpleVector2D<float> a2,
                          SimpleVector2D<float> b1, SimpleVector2D<float> b2);

bool IsPointLeftOfLine(const SimpleVector2D<float>& point,
                        const SimpleVector2D<float>& lineA,
                        const SimpleVector2D<float>& lineB);

bool IsPointInsidePolygon(const SimpleVector2D<float>& point, SimplePolygon2D polygon);

SimpleVector2D<float> GetNearestPointOnLineSegment(const SimpleVector2D<float>& p,
                                                    const SimpleVector2D<float>& segA,
                                                    const SimpleVector2D<float>& segB);
```

---

## How the navmesh baking works

1. Floor polygons are unioned together using Clipper2
2. Obstacles are inflated by `agentRadius` (Minkowski sum via Clipper2 offsetting)
3. The inflated obstacles are subtracted from the floor union
4. The resulting polygon is fed to CDT to produce a Constrained Delaunay Triangulation
5. Triangle centroids become graph nodes, adjacent triangles get edges
6. Edge weights are initialized as Euclidean distances between centroids

This means the navmesh respects the agent's physical footprint. Paths will never route the agent closer to a wall or obstacle than `agentRadius`.

---

## Path smoothing

`FunnelPath` runs the funnel (string-pulling) algorithm over the raw sequence of triangle nodes returned by A*. It traces a funnel through the shared edges (portals) between consecutive triangles and collapses the path into the shortest set of waypoints that stays within the navigable corridor.

The result is a straight-line path that visually looks like an agent taking natural routes rather than weaving through triangle centers.

---

## Using A* standalone

If you have your own graph and just want the pathfinder:

```cpp
EuclideanGraph myGraph;
// ... populate nodes and edges ...

auto nodePath = AStar::FindPath(
    myGraph,
    startNode,
    goalNode,
    AStar::DistanceHeuristic
);
```

Use `ZeroHeuristic` for graphs where Euclidean distance is not a valid lower bound (non-spatial graphs).

---

## Dependencies

All dependencies are bundled in `/external/` and require no separate installation.

- **CDT** (header-only) by Artur Bac
- **Clipper2** by Angus Johnson
- **Predicates** by Jonathan Shewchuk (exact arithmetic geometric predicates)

---

## Namespace summary

```
fluczak::Ai::NavMesh
fluczak::Ai::NavMeshPath
fluczak::Ai::NavMeshSample
fluczak::Ai::AStar
fluczak::Ai::EuclideanGraph
fluczak::Ai::Node
fluczak::Ai::Edge

fluczak::VectorMath::SimpleVector2D<T>
fluczak::VectorMath::Utils::DoSegmentsIntersect
fluczak::VectorMath::Utils::IsPointLeftOfLine
fluczak::VectorMath::Utils::IsPointInsidePolygon
fluczak::VectorMath::Utils::GetNearestPointOnLineSegment
```
