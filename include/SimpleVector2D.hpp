#pragma once
#include <vector>

namespace fluczak::VectorMath
{
	template<class T>
	class SimpleVector2D
	{
	public:
		SimpleVector2D();
		SimpleVector2D(const T& xToSet, const T& yToSet);

		SimpleVector2D(const SimpleVector2D& other);

		T Length() const;

		SimpleVector2D operator*(T other)const;

		SimpleVector2D operator-(const SimpleVector2D& other) const;

		SimpleVector2D operator+(const SimpleVector2D& other) const
		{
			return { x + other.x,y + other.y };
		}

		SimpleVector2D operator/(T other) const;
		bool operator!=(const SimpleVector2D& other) const;

		static SimpleVector2D<float> Lerp(const SimpleVector2D<float>& a, const SimpleVector2D<float>& b, float t)
		{
			return a*(1 - t)  + b*t;
		}

		static T Distance2(const SimpleVector2D<T>& segmentA, const SimpleVector2D<T>& segmentB);

		static T Length(const SimpleVector2D& vector);
		static T Length2(const SimpleVector2D& vector);

		bool operator==(const SimpleVector2D& other) const;
		float Dot(const SimpleVector2D& other) const;
		static float StaticDot(const SimpleVector2D& a, const SimpleVector2D& b);

		SimpleVector2D Normalized();

		void Normalize();

		T x;
		T y;
	};

	template <class T>
	SimpleVector2D<T>::SimpleVector2D()
	{
		
	}

	template <class T>
	SimpleVector2D<T>::SimpleVector2D(const T& xToSet, const T& yToSet): x(xToSet), y(yToSet)
	{
			
	}

	template <class T>
	SimpleVector2D<T>::SimpleVector2D(const SimpleVector2D& other): x(other.x), y(other.y)
	{
	}

	template <class T>
	T SimpleVector2D<T>::Length() const
	{
		return static_cast<T>(sqrt(pow(static_cast<double>(x), 2) + pow(static_cast<double>(y), 2)));
	}

	template <class T>
	SimpleVector2D<T> SimpleVector2D<T>::operator*(T other)const
	{
		SimpleVector2D result;

		result.x = x * other;
		result.y = y * other;

		return result;
	}

	template <class T>
	SimpleVector2D<T> SimpleVector2D<T>::operator-(const SimpleVector2D& other) const
	{
		SimpleVector2D result{};

		result.x = x - other.x;
		result.y = y - other.y;

		return  result;
	}

	template <class T>
	SimpleVector2D<T> SimpleVector2D<T>::operator/(T other) const
	{
		SimpleVector2D result{};

		result.x = x / other;
		result.y = y / other;

		return result;
	}

	template <class T>
	bool SimpleVector2D<T>::operator!=(const SimpleVector2D& other) const
	{
		return other.x != x || other.y != y;
	}

	template <class T>
	T SimpleVector2D<T>::Distance2(const SimpleVector2D<T>& segmentA, const SimpleVector2D<T>& segmentB)
	{
		return Length2(segmentA - segmentB);
	}

	template <class T>
	T SimpleVector2D<T>::Length(const SimpleVector2D& vector)
	{
		return vector.Length();
	}

	template <class T>
	T SimpleVector2D<T>::Length2(const SimpleVector2D& vector)
	{
		return pow(vector.x, 2) + pow(vector.y, 2);
	}

	template <class T>
	bool SimpleVector2D<T>::operator==(const SimpleVector2D& other) const
	{
		return other.x == x && other.y == y;
	}

	template <class T>
	float SimpleVector2D<T>::Dot(const SimpleVector2D& other) const
	{
		return x * other.x + y * other.y;
	}

	template <class T>
	float SimpleVector2D<T>::StaticDot(const SimpleVector2D& a, const SimpleVector2D& b)
	{
		return a.x * b.x + a.y * b.y;
	}

	template <class T>
	SimpleVector2D<T> SimpleVector2D<T>::Normalized()
	{
		SimpleVector2D result{};

		result.x = x;
		result.y = y;

		result /= Length();

		return result;
	}

	template <class T>
	void SimpleVector2D<T>::Normalize()
	{
		SimpleVector2D normalized = Normalized();

		x = normalized.x;
		y = normalized.y;
	}
}
