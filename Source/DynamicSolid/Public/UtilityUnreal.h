// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "UtilityMath.h"

namespace utility
{
	namespace unreal
	{
		FVector Vector3ToFVector(const Vector3<real>& EigenVector);

		Vector3<real> FVectorToVector3(const FVector& UeVector);

		FVector2D Vector2ToFVector2D(const Vector2<real>& EigenVector);

		Vector2<real> FVector2DToVector2(const FVector2D& UeVector);

		
	}
}
