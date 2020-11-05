// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "FTetDynamicPoint.h"
#include "FDynamicTetrahedron.h"

class DYNAMICSOLID_API FTetrahedronMesh
{
public:
	FTetrahedronMesh();
	~FTetrahedronMesh();

	TArray<FTetDynamicPoint*> DynamicPoints;
	TArray<FDynamicTetrahedron*> DynamicTetrahedrons;
};
