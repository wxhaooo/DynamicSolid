#pragma once

#include<algorithm>
#include<iostream>
#include<vector>


namespace utility
{
	namespace geometry
	{
	
	}
}

namespace utility
{
	namespace geometry
	{
		template<typename real>
		void Add(const real& x, std::vector<real>& xs) {
			if (std::find(xs.begin(), xs.end(), x) == xs.end())
				xs.push_back(x);
		}

		template<typename real>
		void Add(const real& x, TArray<real>& xs)
		{
			if (!xs.Find(x)) xs.Add(x);
		}
	}
}

