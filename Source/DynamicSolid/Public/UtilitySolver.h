// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

// #include "CoreMinimal.h"

#include "UtilityMath.h"

DECLARE_LOG_CATEGORY_EXTERN(LogSolver, Log, All);

namespace utility
{
	namespace solver
	{
		VectorX<real> MPCG(const SpMat<real>& A, const VectorX<real>& b,
			const VectorX<real>& z, const TArray<Matrix3x3<real>>& SArray,
			const real Epsilon, const int MaxEpoch);

		VectorX<real> MPCGFilter(const VectorX<real>& V, 
			const TArray<Matrix3x3<real>>& SArray);

		VectorX<real> PPCG(const SpMat<real>& A, const VectorX<real>& b,
			const VectorX<real>& z, const TArray<Matrix3x3<real>>& SArray);

		SpMat<real> ComputeGlobalS(const TArray<Matrix3x3<real>>& SArray);
	}
}
