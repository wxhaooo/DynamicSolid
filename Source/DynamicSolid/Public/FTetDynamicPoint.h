// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "UtilityMath.h"

class FTetDynamicEdge;
class FTetDynamicFace;

class DYNAMICSOLID_API FTetDynamicPoint
{
public:
	FTetDynamicPoint();
	~FTetDynamicPoint();

	Vector3<real> Position;
	Vector3<real> RestPosition;
	Vector3<real> InitialPosition;

	Vector3<real> Velocity;
	Vector3<real> InitialVelocity;

	Vector3<real> Accleration;

	TArray<FTetDynamicEdge*> AdjacentEdges;
	TArray<FTetDynamicFace*> AdjacentFaces;
};
