// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "FTetDynamicPoint.h"

class FTetDynamicPoint;
class FTetDynamicFace;

class DYNAMICSOLID_API FTetDynamicEdge
{
public:
	FTetDynamicEdge();
	~FTetDynamicEdge();

	FTetDynamicPoint* Point[2];
	TArray<FTetDynamicFace*> AdjacentFaces;
};
