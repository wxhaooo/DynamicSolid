// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "UtilityMath.h"

namespace utility
{
	namespace debug
	{
		void AddOnScreen(int Key, float TimeToDisplay, const FColor& DisplayColor, const Matrix3x3<real>& Mat);

		void AddOnScreen(int Key, float TimeToDisplay, const FColor& DisplayColor, const Vector3<real>& Vec3);

		void AddOnScreen(int Key, float TimeToDisplay, const FColor& DisplayColor, const VectorX<real>& VecX, int Start, int End);

		FString ConvertLogStr(const VectorX<real>& VecX, int Start, int End);
	}
}