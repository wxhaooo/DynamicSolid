// Fill out your copyright notice in the Description page of Project Settings.


#include "FTetrahedronMesh.h"
#include "UtilityGeometry.h"

FTetrahedronMesh::FTetrahedronMesh()
{
	
}

FTetrahedronMesh::FTetrahedronMesh(const FString& TetrahedronMeshPath)
{
	ParseTetrahedronMesh(TetrahedronMeshPath);
}

FTetrahedronMesh::~FTetrahedronMesh()
{
	
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

