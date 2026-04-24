#include "NavMeshPath.hpp"

#include "MathUtilities.hpp"

fluczak::Ai::NavMeshPath::NavMeshPath(const std::vector<std::reference_wrapper<const Ai::Node>>& nodesToSet)
{
    for (auto node : nodesToSet)
    {
        points.push_back(node.get().position);
        aiNodes.push_back(&node.get());
    }
}

fluczak::Ai::NavMeshPath::NavMeshPath(const std::vector<VectorMath::SimpleVector2D<float>>& pointsToSet)
{
    points = pointsToSet;
}

float fluczak::Ai::NavMeshPath::GetPercentageAlongPath(const VectorMath::SimpleVector2D<float>& point) const
{
    if (points.size() < 2) return 0.0f;

    float totalLength = 0.0f;
    std::vector<float> accumulatedLengths(points.size());

    for (size_t i = 0; i < points.size() - 1; i++)
    {
	    VectorMath::SimpleVector2D<float> delta = points[i + 1] - points[i];
        const float segmentLength = VectorMath::SimpleVector2D<float>::Length(delta);
        totalLength += segmentLength;
        accumulatedLengths[i + 1] = accumulatedLengths[i] + segmentLength;
    }

    size_t segmentIndex = 0;
    float closestDistance = std::numeric_limits<float>::max();

    //find the closest point to the given point
    for (size_t i = 0; i < points.size() - 1; i++)
    {
        auto closestPoint = VectorMath::Utils::GetNearestPointOnLineSegment(point, points[i + 1], points[i]);
        const float distance = VectorMath::SimpleVector2D<float>::Length(point- closestPoint);

        if (distance < closestDistance)
        {
            closestDistance = distance;
            segmentIndex = i;
        }
    }

    // Calculate the percentage along the closest segment.
    const float t = VectorMath::SimpleVector2D<float>::StaticDot(point - points[segmentIndex], points[segmentIndex + 1] - points[segmentIndex]) / powf(VectorMath::SimpleVector2D<float>::Length(points[segmentIndex + 1] - points[segmentIndex]),2);
    // Calculate the accumulated length up to the closest segment.
    const float accumulatedLengthUpToSegment = accumulatedLengths[segmentIndex] + t * (accumulatedLengths[segmentIndex + 1] - accumulatedLengths[segmentIndex]);
    // Calculate the total percentage along the entire path.
    const float totalT = accumulatedLengthUpToSegment / totalLength;
    return totalT;
}

fluczak::VectorMath::SimpleVector2D<float> fluczak::Ai::NavMeshPath::GetClosestPointOnPath(const VectorMath::SimpleVector2D<float>& point) const
{
    if (points.size() <= 1) return {};
    std::vector<VectorMath::SimpleVector2D<float>> closestPoints = {};

    for (size_t i = 0; i < points.size() - 1; i++)
    {
        closestPoints.push_back(VectorMath::Utils::GetNearestPointOnLineSegment(point, points[i], points[i + 1]));
    }

    std::sort(closestPoints.begin(), closestPoints.end(), [point](const VectorMath::SimpleVector2D<float>& a, const VectorMath::SimpleVector2D<float>& b)
        {
            return VectorMath::SimpleVector2D<float>::Distance2(a, point) < VectorMath::SimpleVector2D<float>::Distance2(b, point);
        });

    return closestPoints[0];
}

fluczak::VectorMath::SimpleVector2D<float> fluczak::Ai::NavMeshPath::FindPointOnPath(float t) const
{
    if (points.empty()) return { 0, 0 };
    if (t <= 0.0f)
    {
        return points.front();  // Return the first point for position 0 or less
    }
    if (t >= 1.0f)
    {
        return points.back();  // Return the last point for position 1 or more
    }

    float totalLength = 0.0f;
    std::vector<float> segmentLengths;

    for (size_t i = 0; i < points.size() - 1; i++)
    {
        float segmentLength = VectorMath::SimpleVector2D<float>::Length(points[i]- points[i + 1]);
        totalLength += segmentLength;
        segmentLengths.push_back(segmentLength);
    }

    const float targetLength = t * totalLength;

    // Find the segment containing the target length
    size_t segmentIndex = 0;
    float accumulatedLength = 0.0f;
    for (size_t i = 0; i < segmentLengths.size(); i++)
    {
        accumulatedLength += segmentLengths[segmentIndex];
        if (accumulatedLength >= targetLength)
        {
            break;
        }
        segmentIndex++;
    }

    if (segmentIndex >= points.size() - 1) return points.back();
    const float segmentT = (targetLength - accumulatedLength + segmentLengths[segmentIndex]) / segmentLengths[segmentIndex];
    return VectorMath::SimpleVector2D<float>::Lerp(points[segmentIndex], points[segmentIndex + 1], segmentT);
}
