// Fill out your copyright notice in the Description page of Project Settings.


#include "DynamicSolidComponent.h"
#include "RuntimeMeshComponent.h"
#include "FTetrahedronMesh.h"
#include "Providers/RuntimeMeshProviderStatic.h"
#include "UtilityUnreal.h"

// Sets default values for this component's properties
UDynamicSolidComponent::UDynamicSolidComponent()
{
	// Set this component to be initialized when the game starts, and to be ticked every frame.  You can turn these features
	// off to improve performance if you don't need them.
	PrimaryComponentTick.bCanEverTick = true;

	RuntimeMeshComp = CreateDefaultSubobject<URuntimeMeshComponent>("RuntimeMeshComp");
}


// Called when the game starts
void UDynamicSolidComponent::BeginPlay()
{
	Super::BeginPlay();

	// ...
	
}


// Called every frame
void UDynamicSolidComponent::TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction)
{
	Super::TickComponent(DeltaTime, TickType, ThisTickFunction);

	// ...
}

URuntimeMeshComponent* UDynamicSolidComponent::GetRuntimeMeshComp()
{
	return RuntimeMeshComp;
}

bool UDynamicSolidComponent::InitializeRuntimeMeshComp()
{
	RuntimeMeshProviderStatic
		= NewObject<URuntimeMeshProviderStatic>(this,
			"RuntimeMeshProviderStatic");
	
	if (RuntimeMeshProviderStatic == nullptr)
	{
		return false;
	}
	RuntimeMeshComp->Initialize(RuntimeMeshProviderStatic);
	
	return true;
}

bool UDynamicSolidComponent::UpdateRenderableData()
{
	if (TetrahedronMeshSPtr == nullptr) return false;
	
	FRuntimeMeshSectionProperties MeshSectionProp;
	FRuntimeMeshRenderableMeshData RenderableMeshData;

	FRuntimeMeshTriangleStream TriangleStream;
	FRuntimeMeshVertexPositionStream PositionStream;
	FRuntimeMeshVertexTexCoordStream TexCoordsStream;
	FRuntimeMeshVertexColorStream ColorsStream;
	FRuntimeMeshVertexTangentStream TangentsStream;
	FRuntimeMeshTriangleStream AdjacencyTrianglesStream;

	// UE_LOG(LogTemp, Display, TEXT("%d"), TetrahedronMeshSPtr->RenderableTriangleIndexArray.Num());
	for (int i = 0; i < TetrahedronMeshSPtr->RenderableTriangleIndexArray.Num(); i += 3)
	{
		TriangleStream.AddTriangle(
			TetrahedronMeshSPtr->RenderableTriangleIndexArray[i],
			TetrahedronMeshSPtr->RenderableTriangleIndexArray[i + 1],
			TetrahedronMeshSPtr->RenderableTriangleIndexArray[i + 2]);
	}

	check(TetrahedronMeshSPtr->RenderablePointIndexArray.Num()
		== TetrahedronMeshSPtr->RenderableUvArray.Num());

	UE_LOG(LogTemp, Display, TEXT("PointIndexArraySize: %d\n"), TetrahedronMeshSPtr->RenderablePointIndexArray.Num());
	UE_LOG(LogTemp, Display, TEXT("PointUvArraySize: %d\n"), TetrahedronMeshSPtr->RenderableUvArray.Num());
	UE_LOG(LogTemp, Display, TEXT("PointNormalArraySize: %d\n"), TetrahedronMeshSPtr->RenderableNormalArray.Num());

	check(TetrahedronMeshSPtr->RenderablePointIndexArray.Num()
		== TetrahedronMeshSPtr->RenderableNormalArray.Num());
	
	//vertex attributes
	for (int i = 0; i <TetrahedronMeshSPtr->RenderablePointIndexArray.Num(); i++)
	{
		int CurPointIndex = TetrahedronMeshSPtr->RenderablePointIndexArray[i];
		PositionStream.Add(utility::unreal::Vector3ToFVector(TetrahedronMeshSPtr->DynamicPointArray[CurPointIndex]->Position));
		ColorsStream.Add(FColor::White);
		TexCoordsStream.Add(utility::unreal::Vector2ToFVector2D(TetrahedronMeshSPtr->RenderableUvArray[CurPointIndex]));
		TangentsStream.Add(utility::unreal::Vector3ToFVector(TetrahedronMeshSPtr->RenderableNormalArray[CurPointIndex]), FVector(0., 0., 0.));
	}
	
	RenderableMeshData.Triangles = TriangleStream;
	RenderableMeshData.Positions = PositionStream;
	RenderableMeshData.TexCoords = TexCoordsStream;
	RenderableMeshData.Colors = ColorsStream;
	RenderableMeshData.Tangents = TangentsStream;
	RenderableMeshData.AdjacencyTriangles = AdjacencyTrianglesStream;
	
	MeshSectionProp.bIsVisible = true;
	MeshSectionProp.bCastsShadow = true;
	MeshSectionProp.MaterialSlot = 1;
	MeshSectionProp.NumTexCoords = TexCoordsStream.Num();
	MeshSectionProp.UpdateFrequency =
		ERuntimeMeshUpdateFrequency::Frequent;
	MeshSectionProp.bWants32BitIndices = true;
	
	RuntimeMeshProviderStatic->SetupMaterialSlot
	(0, TEXT("TriMat"), MeshMaterial);
	
	RuntimeMeshProviderStatic->CreateSection(0, 0,
		MeshSectionProp, RenderableMeshData);
	
	return true;
}

TSharedPtr<FTetrahedronMesh> UDynamicSolidComponent::GetTetrahedronMeshSPtr()
{
	return TetrahedronMeshSPtr;
}

void UDynamicSolidComponent::PostEditChangeProperty(FPropertyChangedEvent& PropertyChangedEvent)
{
	FName PropName = PropertyChangedEvent.Property->GetFName();

	UNameProperty* prop = static_cast<UNameProperty*>(PropertyChangedEvent.Property);
	FString MeshLoc;
	prop->GetPropertyValue(&MeshLoc);

	// UE_LOG(LogTemp, Display, TEXT("PropPos: %s"), *MeshLoc);

	if (PropName != FName("InitialSolidPath")) return;

	//to resolve tet solid

	TetrahedronMeshSPtr = MakeShared<FTetrahedronMesh>(InitialSolidPath);
	
	// UE_LOG(LogTemp, Display, TEXT("PropName: %s"), *PropName.ToString());
}

