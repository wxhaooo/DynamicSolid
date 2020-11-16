// Fill out your copyright notice in the Description page of Project Settings.


#include "FTetrahedronMesh.h"
#include "UtilityGeometry.h"
#include "UtilityUnreal.h"

FTetrahedronMesh::FTetrahedronMesh()
{
	
}

FTetrahedronMesh::FTetrahedronMesh(const FString& TetrahedronMeshPath)
{
	Initialize(TetrahedronMeshPath);
}

FTetrahedronMesh::~FTetrahedronMesh()
{
	
}

bool FTetrahedronMesh::Initialize(const FString& TetrahedronMeshPath)
{
	ParseTetrahedronMesh(TetrahedronMeshPath);
	return true;
}

void FTetrahedronMesh::ParseTetrahedronMesh(const FString& TetrahedronMeshPath)
{
    FString TetMeshName, TetMeshFormat;
    TetrahedronMeshPath.Split(".",&TetMeshName,&TetMeshFormat);

    if (TetMeshFormat != "tet") {
	    UE_LOG(LogTemp, Display, TEXT("Loaded file is not .tet format! It is %s!"), *TetMeshFormat);
	    return;
    }

    // UE_LOG(LogTemp, Display, TEXT("TetMeshName: %s"), *TetMeshName);
    // UE_LOG(LogTemp, Display, TEXT("TetMeshFormat: %s"), *TetMeshFormat);

    //parse tet file
    utility::geometry::ParseTetFile(TetrahedronMeshPath,this);
}

bool FTetrahedronMesh::ApplyRootActorTransform(const FTransform& RootActorTransform)
{
    for (int i = 0; i < DynamicPointArray.Num(); i++)
    {
	TSharedPtr<FTetDynamicPoint> CurDynamicPoint = DynamicPointArray[i];
	FVector TmpPosition = utility::unreal::Vector3ToFVector(CurDynamicPoint->Position);

	TmpPosition = RootActorTransform.TransformPosition(TmpPosition);
	
	CurDynamicPoint->Position = utility::unreal::FVectorToVector3(TmpPosition);
	CurDynamicPoint->InitialPosition = CurDynamicPoint->PostPosition =
	CurDynamicPoint->RestPosition = CurDynamicPoint->Position;
    }
    return true;
}

