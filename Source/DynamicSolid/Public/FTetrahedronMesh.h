// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "FTetDynamicPoint.h"
#include "FDynamicTetrahedron.h"

class DYNAMICSOLID_API FTetrahedronMesh
{
public:
	FTetrahedronMesh();
	FTetrahedronMesh(const FString& TetrahedronMeshPath);
	~FTetrahedronMesh();

	//dynamic data
	TArray<TSharedPtr<FTetDynamicPoint>> DynamicPointArray;
	TArray<TSharedPtr<FDynamicTetrahedron>> DynamicTetrahedronArray;

	//renderable data
	TArray<int> RenderablePointIndexArray;
	TArray<int> RenderableTriangleIndexArray;
	TArray<Vector2<real>> RenderableUvArray;
	TArray<Vector3<real>> RenderableNormalArray;
	
	bool Initialize(const FString& TetrahedronMeshPath);
	void ParseTetrahedronMesh(const FString& TetrahedronMeshPath);
	bool ApplyRootActorTransform(const FTransform& RootActorTransform = FTransform::Identity);
};