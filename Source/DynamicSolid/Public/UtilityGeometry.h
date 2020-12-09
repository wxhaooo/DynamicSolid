#pragma once

#include<algorithm>
#include<iostream>
#include<vector>
#include "FTetDynamicPoint.h"
#include "FDynamicTetrahedron.h"

class FTetrahedronMesh;

namespace utility
{
	namespace geometry
	{
		void ParseTetFile(const FString& TetFilePath, FTetrahedronMesh* TetrahedronMeshPt);

		Vector3<real> TriangleFaceNormal(const Vector3<real> P0, const Vector3<real> P1, const Vector3<real> P2);
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

