// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "UtilityMath.h"

namespace utility
{
	namespace numeric
	{
		UVSGroup RotationVariantSVD(const Matrix3x3<real>& F);
	}
}