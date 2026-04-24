#pragma once
#include "SimplePolygon2D.hpp"
#include "Predicates/predicates.h"

namespace fluczak::VectorMath::Utils
{
    static inline bool DoSegmentsIntersect(const SimpleVector2D<float>& one, const SimpleVector2D<float>& two, const SimpleVector2D<float>& three, const SimpleVector2D<float>& four)
    {
        const double denominator = ((two.x - one.x) * (four.y - three.y)) - ((two.y - one.y) * (four.x - three.x));
        // Check for parallel lines
        if (denominator == 0) return false;

        const double numerator1 = ((one.y - three.y) * (four.x - three.x)) - ((one.x - three.x) * (four.y - three.y));
        const double numerator2 = ((one.y - three.y) * (two.x - one.x)) - ((one.x - three.x) * (two.y - one.y));

        const double reverse_denominator = 1.0f / static_cast<float>(denominator);

        const double r = static_cast<float>(numerator1) * reverse_denominator;
        const double s = static_cast<float>(numerator2) * reverse_denominator;
        return (r >= 0 && r <= 1) && (s >= 0 && s <= 1);
    }

    static inline bool IsPointLeftOfLine(const SimpleVector2D<float>& point, const SimpleVector2D<float>& line1, const SimpleVector2D<float>& line2)
    {
	    const double p[2] = { point.x, point.y };
	    const double la[2] = { line1.x, line1.y };
	    const double lb[2] = { line2.x, line2.y };
        return predicates::adaptive::orient2d(p, la, lb) > 0;
    }

    static inline bool IsPointInsidePolygon(const SimpleVector2D<float>& point, const SimplePolygon2D& polygon)
    {
        // Adapted from: https://wrfranklin.org/Research/Short_Notes/pnpoly.html

        size_t i, j;
        size_t n = polygon.size();
        bool inside = false;

        for (i = 0, j = n - 1; i < n; j = i++)
        {
            if ((polygon[i].y > point.y != polygon[j].y > point.y) &&
                (point.x < (polygon[j].x - polygon[i].x) * (point.y - polygon[i].y) / (polygon[j].y - polygon[i].y) + polygon[i].x))
                inside = !inside;
        }

        return inside;
    }

    static SimpleVector2D<float> GetNearestPointOnLineSegment( const SimpleVector2D<float>& p, const SimpleVector2D<float>& segmentA, const SimpleVector2D<float>& segmentB)
    {
        float t = SimpleVector2D<float>::StaticDot(p - segmentA, segmentB - segmentA) / SimpleVector2D<float>::Distance2(segmentA, segmentB);
        if (t <= 0) return segmentA;
        if (t >= 1) return segmentB;
        return   (segmentA* (1 - t) ) +   (segmentB*t);
    }
}
