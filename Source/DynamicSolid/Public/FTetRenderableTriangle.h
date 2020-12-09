// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "UtilityMath.h"
#include "FTetDynamicPoint.h"
#include "UtilityGeometry.h"

class DYNAMICSOLID_API FTetRenderableTriangle
{
public:
	FTetRenderableTriangle();
	~FTetRenderableTriangle();

	FTetRenderableTriangle(DynamicPointSPtr P0, DynamicPointSPtr P1, DynamicPointSPtr P2)
	{
		this->P0 = P0; this->P1 = P1; this->P2 = P2;
		UpdateNormal();
	}

	DynamicPointSPtr P0, P1, P2;
	
	Vector3<real> Normal;

	void UpdateNormal();
};

using TetRenderableTriangleUPtr = TUniquePtr<FTetRenderableTriangle>;
using TetRenderableTriangleSPtr = TSharedPtr<FTetRenderableTriangle>;
using TetRenderableTrianglePtr = FTetRenderableTriangle*;