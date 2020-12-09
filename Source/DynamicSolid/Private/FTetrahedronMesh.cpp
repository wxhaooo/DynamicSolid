// Fill out your copyright notice in the Description page of Project Settings.


#include "FTetrahedronMesh.h"
#include "UtilityGeometry.h"
#include "UtilityUnreal.h"
#include "MagicNumber.h"

FTetrahedronMesh::FTetrahedronMesh()
{
	
}

FTetrahedronMesh::FTetrahedronMesh(const FString& TetrahedronMeshPath)
{
	Mass = 0.f;
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

	TmpPosition *= UnitTransferMToCM;
	TmpPosition = RootActorTransform.TransformPosition(TmpPosition);
	TmpPosition *= UnitTransferCmToM;
    	
	CurDynamicPoint->Position = utility::unreal::FVectorToVector3(TmpPosition);
	CurDynamicPoint->InitialPosition = CurDynamicPoint->PostPosition =
	CurDynamicPoint->RestPosition = CurDynamicPoint->Position;
    }
    return true;
}

bool FTetrahedronMesh::ComputeMassPerPoint(real Density)
{
	for (int i = 0; i < DynamicTetrahedronArray.Num(); i++)
	{
		TSharedPtr<FDynamicTetrahedron> CurTet = DynamicTetrahedronArray[i];
		real TetMass = CurTet->ComputeMass(Density);
		Mass += TetMass;
		CurTet->p0->AddMass(TetMass / 4.f);
		CurTet->p1->AddMass(TetMass / 4.f);
		CurTet->p2->AddMass(TetMass / 4.f);
		CurTet->p3->AddMass(TetMass / 4.f);
	}

	
	return true;
}