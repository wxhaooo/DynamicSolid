// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

// #include "CoreMinimal.h"

#include "UtilityMath.h"

namespace utility
{
	namespace solver
	{
		VectorX<real> MPCG(const MatrixX<real>& A, const VectorX<real>& b);
	}
}
