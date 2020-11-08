// Fill out your copyright notice in the Description page of Project Settings.


#include "UtilityUnreal.h"

namespace utility
{
	namespace unreal
	{
		FVector Vector3ToFVector(const Vector3<real>& EigenVector)
		{
			return FVector(EigenVector(0), EigenVector(1), EigenVector(2));
		}

		Vector3<real> FVectorToVector3(const FVector& UeVector)
		{
			return Vector3<real>(UeVector.X, UeVector.Y, UeVector.Z);
		}

		FVector2D Vector2ToFVector2D(const Vector2<real>& EigenVector)
		{
			return FVector2D(EigenVector(0), EigenVector(1));
		}

		Vector2<real> FVector2DToVector2(const FVector2D& UeVector)
		{
			return Vector2<real>(UeVector.X, UeVector.Y);
		}
	}
}

