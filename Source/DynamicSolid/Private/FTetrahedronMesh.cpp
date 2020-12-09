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

void FTetrahedronMesh::ExtractTetSurfaceTriangle()
{
	int TriangleNum = RenderableTriangleIndexArray.Num() / 3;

	for (int i = 0; i < TriangleNum; i += 3)
	{
		int P0Index = RenderableTriangleIndexArray[i];
		int P1Index = RenderableTriangleIndexArray[i + 1];
		int P2Index = RenderableTriangleIndexArray[i + 2];

		RenderableTetSurfaceTriangleArray.Add(MakeShared<FTetRenderableTriangle>(
			DynamicPointArray[P0Index],
			DynamicPointArray[P1Index],
			DynamicPointArray[P2Index]));

		DynamicPointArray[P0Index]->AdjRenderableTriangleArray.Add(RenderableTetSurfaceTriangleArray.Last());
		DynamicPointArray[P1Index]->AdjRenderableTriangleArray.Add(RenderableTetSurfaceTriangleArray.Last());
		DynamicPointArray[P2Index]->AdjRenderableTriangleArray.Add(RenderableTetSurfaceTriangleArray.Last());

	}
}

bool FTetrahedronMesh::Initialize(const FString& TetrahedronMeshPath)
{
	ParseTetrahedronMesh(TetrahedronMeshPath);
	ExtractTetSurfaceTriangle();
	
	return true;
}

void FTetrahedronMesh::ComputeNormals()
{
    ComputeNormals(FindDoubleVertices());
}

void FTetrahedronMesh::ComputeNormals(const TArray<FDoubleVertices>& doubles)
{
    // FStaticLODModel& Model = Mesh->GetResourceForRendering()->LODModels[0];
    //
    // TArray<FSoftSkinVertex*> vi = GetAllVertices(Model);

    TArray<DynamicPointSPtr> vi;
    for (int i = 0; i < RenderablePointIndexArray.Num(); i++)
    {
        vi.Add(DynamicPointArray[RenderablePointIndexArray[i]]);
    }

    const int count = vi.Num();
    TArray<Vector3<real>> normal, tangent, bitangent;
    for(int i=0;i<count;i++)
    {
        normal.Add(Vector3<real>::Zero());
        tangent.Add(Vector3<real>::Zero());
        bitangent.Add(Vector3<real>::Zero());
    }

    // FRawStaticIndexBuffer16or32Interface& ind = *Model.MultiSizeIndexContainer.GetIndexBuffer();
    
	Vector3<real> n;
    uint32 ti[3];

    int i, j, k;

    uint16 offset = 0;

    for (i = 0; i < RenderableTriangleIndexArray.Num(); i += 3)
    {
        ti[0] = RenderableTriangleIndexArray[i];
        ti[1] = RenderableTriangleIndexArray[i + 1];
        ti[2] = RenderableTriangleIndexArray[i + 2];

        n = (vi[ti[0]]->Position - vi[ti[1]]->Position).cross(vi[ti[2]]->Position - vi[ti[0]]->Position);

       Vector3<real> deltaPos1 = vi[ti[1]]->Position - vi[ti[0]]->Position;
       Vector3<real> deltaPos2 = vi[ti[2]]->Position - vi[ti[0]]->Position;

    	
      Vector2<real> deltaUV1 = RenderableUvArray[ti[1]] - RenderableUvArray[ti[0]];
      Vector2<real> deltaUV2 = RenderableUvArray[ti[2]] - RenderableUvArray[ti[0]];

        float r = 1.0f / (deltaUV1.x() * deltaUV2.y() - deltaUV1.y() * deltaUV2.x());
        Vector3<real> t = (deltaPos1 * deltaUV2.y() - deltaPos2 * deltaUV1.y()) * r;
        Vector3<real> b = (deltaPos2 * deltaUV1.x() - deltaPos1 * deltaUV2.x()) * r;

        for (j = 0; j < 3; j++)
        {
            normal[ti[j]] += n;

            tangent[ti[j]] += t;
            bitangent[ti[j]] += b;

            for (k = 0; k < doubles[ti[j] - offset].doubles.Num(); k++)
            {
                normal[doubles[ti[j] - offset].doubles[k] + offset] += n;
            }
        }
    }
    FVector TanX, TanY;
    for (i = 0; i < count; i++)
    {
        normal[i].normalize();
        tangent[i].normalize();
        bitangent[i].normalize();

        // vi[i]->TangentZ = normal[i];
        // vi[i]->TangentX = tangent[i];
        // vi[i]->TangentY = bitangent[i];
    }

    RenderableNormalArray = normal;
    RenderableTangentArray = tangent;
    RenderableBiTangentArray = bitangent;
}


TArray<FDoubleVertices> FTetrahedronMesh::FindDoubleVertices()
{
    const float DoublePositionTolerance = .001f;

    // FStaticLODModel& Model = Mesh->GetResourceForRendering()->LODModels[0];
    // TArray<FSoftSkinVertex*> vi = GetAllVertices(Model);
	//Get Renderable Point Array

    TArray<DynamicPointSPtr> vi;
    for (int i = 0; i < RenderablePointIndexArray.Num(); i++)
	{
        vi.Add(DynamicPointArray[RenderablePointIndexArray[i]]);
	}
	
    const int count = vi.Num();

    TArray<FDoubleVertices> doubles;
    doubles.SetNum(count);

    int i, j, k;

    for (i = 0; i < count; i++)
    {
        if (doubles[i].doubles.Num() != 0)
            continue;

        FDoubleVertices matches;

        for (j = i + 1; j < count; j++)
        {
            if (doubles[j].doubles.Num() != 0)
                continue;

            if ((vi[i]->Position - vi[j]->Position).isMuchSmallerThan(DoublePositionTolerance))
                matches.doubles.Add(j);
            // if (vi[i]->Position.Equals(vi[j]->Position, DoublePositionTolerance))
            //     matches.doubles.Add(j);
        }

        if (matches.doubles.Num())
        {
            doubles[i] = matches;
            for (j = 0; j < matches.doubles.Num(); j++)
            {
                doubles[matches.doubles[j]].doubles.Add(i);
                for (k = 0; k < matches.doubles.Num(); k++)
                {
                    if (k != j)
                        doubles[matches.doubles[j]].doubles.Add(matches.doubles[k]);
                }
            }
        }
    }

    return doubles;
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