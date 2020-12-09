// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "FTetDynamicPoint.h"
#include "FDynamicTetrahedron.h"
#include "FTetRenderableTriangle.h"

struct FDoubleVertices
{
public:
	TArray<int> doubles;
};

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
	TArray<TSharedPtr<FTetRenderableTriangle>> RenderableTetSurfaceTriangleArray;
	TArray<int> RenderableTriangleIndexArray;
	TArray<Vector2<real>> RenderableUvArray;
	TArray<Vector3<real>> RenderableNormalArray;
	TArray<Vector3<real>> RenderableBiTangentArray;
	TArray<Vector3<real>> RenderableTangentArray;

	real Mass;

	int DynamicPointNumber() { return DynamicPointArray.Num(); }
	int SurfacePointNumber() { return RenderablePointIndexArray.Num(); }
	int DynamicTetraherdronNumber() { return DynamicTetrahedronArray.Num(); }

	void ExtractTetSurfaceTriangle();
	bool ComputeMassPerPoint(real Density);
	bool Initialize(const FString& TetrahedronMeshPath);
	void ParseTetrahedronMesh(const FString& TetrahedronMeshPath);
	bool ApplyRootActorTransform(const FTransform& RootActorTransform = FTransform::Identity);

	void ComputeNormals();

private:
	TArray<FDoubleVertices> FindDoubleVertices();
	void ComputeNormals(const TArray<FDoubleVertices>& doubles);
};