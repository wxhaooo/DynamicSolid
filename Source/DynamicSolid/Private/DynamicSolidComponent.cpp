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
			TEXT("RuntimeMeshProviderStatic"));
	
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
	
	// FRuntimeMeshSectionProperties MeshSectionProp;
	// FRuntimeMeshRenderableMeshData RenderableMeshData;
	//
	// FRuntimeMeshTriangleStream TriangleStream;
	// FRuntimeMeshVertexPositionStream PositionStream;
	// FRuntimeMeshVertexTexCoordStream TexCoordsStream;
	// FRuntimeMeshVertexColorStream ColorsStream;
	// FRuntimeMeshVertexTangentStream TangentsStream;
	// FRuntimeMeshTriangleStream AdjacencyTrianglesStream;
	//
	// // UE_LOG(LogTemp, Display, TEXT("%d"), TetrahedronMeshSPtr->RenderableTriangleIndexArray.Num());
	// for (int i = 0; i < TetrahedronMeshSPtr->RenderableTriangleIndexArray.Num(); i += 3)
	// {
	// 	// UE_LOG(LogTemp, Display, TEXT("%d %d %d\n"), TetrahedronMeshSPtr->RenderableTriangleIndexArray[i],
	// 	// 	TetrahedronMeshSPtr->RenderableTriangleIndexArray[i + 1],
	// 	// 	TetrahedronMeshSPtr->RenderableTriangleIndexArray[i + 2]);
	// 	
	// 	TriangleStream.AddTriangle(
	// 		TetrahedronMeshSPtr->RenderableTriangleIndexArray[i],
	// 		TetrahedronMeshSPtr->RenderableTriangleIndexArray[i + 1],
	// 		TetrahedronMeshSPtr->RenderableTriangleIndexArray[i + 2]);
	// }
	//
	// check(TetrahedronMeshSPtr->RenderablePointIndexArray.Num()
	// 	== TetrahedronMeshSPtr->RenderableUvArray.Num());
	//
	// UE_LOG(LogTemp, Display, TEXT("PointIndexArraySize: %d\n"), TetrahedronMeshSPtr->RenderablePointIndexArray.Num());
	// UE_LOG(LogTemp, Display, TEXT("PointUvArraySize: %d\n"), TetrahedronMeshSPtr->RenderableUvArray.Num());
	// UE_LOG(LogTemp, Display, TEXT("PointNormalArraySize: %d\n"), TetrahedronMeshSPtr->RenderableNormalArray.Num());
	//
	// check(TetrahedronMeshSPtr->RenderablePointIndexArray.Num()
	// 	== TetrahedronMeshSPtr->RenderableNormalArray.Num());
	//
	// //vertex attributes
	// for (int i = 0; i < TetrahedronMeshSPtr->RenderablePointIndexArray.Num(); i++)
	// {
	// 	int CurPointIndex = TetrahedronMeshSPtr->RenderablePointIndexArray[i];
	// 	// FVector tmp = utility::unreal::Vector3ToFVector(TetrahedronMeshSPtr->DynamicPointArray[CurPointIndex]->Position);
	// 	// UE_LOG(LogTemp, Display, TEXT("%s\n"), *tmp.ToString());
	// 	PositionStream.Add(utility::unreal::Vector3ToFVector(TetrahedronMeshSPtr->DynamicPointArray[CurPointIndex]->Position));
	// 	ColorsStream.Add(FColor::White);
	// 	TexCoordsStream.Add(utility::unreal::Vector2ToFVector2D(TetrahedronMeshSPtr->RenderableUvArray[CurPointIndex]));
	// 	TangentsStream.Add(FVector(1., 0., 0.),
	// 		FVector(0., 1., 0.),
	// 		FVector(0., 0., 1.));
	// 	// TangentsStream.Add(utility::unreal::Vector3ToFVector(TetrahedronMeshSPtr->RenderableNormalArray[CurPointIndex]),
	// 	// 	utility::unreal::Vector3ToFVector(TetrahedronMeshSPtr->RenderableNormalArray[CurPointIndex]));
	// }
	//
	// RenderableMeshData.Triangles = TriangleStream;
	// RenderableMeshData.Positions = PositionStream;
	// RenderableMeshData.TexCoords = TexCoordsStream;
	// RenderableMeshData.Colors = ColorsStream;
	// RenderableMeshData.Tangents = TangentsStream;
	// RenderableMeshData.AdjacencyTriangles = AdjacencyTrianglesStream;
	//
	// MeshSectionProp.bIsVisible = true;
	// MeshSectionProp.bCastsShadow = true;
	// MeshSectionProp.MaterialSlot = 1;
	// MeshSectionProp.NumTexCoords = TexCoordsStream.Num();
	// MeshSectionProp.UpdateFrequency =
	// 	ERuntimeMeshUpdateFrequency::Frequent;
	// MeshSectionProp.bWants32BitIndices = true;
	// // RuntimeMeshProviderStatic->CreateSection(0, 0,
	// 	MeshSectionProp, RenderableMeshData);

	if (MeshMaterial != nullptr)
		RuntimeMeshProviderStatic->SetupMaterialSlot
		(0, TEXT("TriMat"), MeshMaterial);

	TArray<FVector> PositionArray;
	TArray<FVector> NormalArray;
	TArray<FVector2D> TexCoordArray;
	TArray<FRuntimeMeshTangent> TangentArray;
	TArray<FColor> ColorArray;

	UE_LOG(LogTemp, Display, TEXT("PointIndexArraySize: %d\n"), TetrahedronMeshSPtr->RenderablePointIndexArray.Num());
	UE_LOG(LogTemp, Display, TEXT("PointUvArraySize: %d\n"), TetrahedronMeshSPtr->RenderableUvArray.Num());
	UE_LOG(LogTemp, Display, TEXT("PointNormalArraySize: %d\n"), TetrahedronMeshSPtr->RenderableNormalArray.Num());

	check(TetrahedronMeshSPtr->RenderablePointIndexArray.Num()
		== TetrahedronMeshSPtr->RenderableUvArray.Num());
	
	check(TetrahedronMeshSPtr->RenderablePointIndexArray.Num()
		== TetrahedronMeshSPtr->RenderableNormalArray.Num());

	for(int i=0;i<TetrahedronMeshSPtr->RenderablePointIndexArray.Num();i++)
	{
		int CurIndex = TetrahedronMeshSPtr->RenderablePointIndexArray[i];
		PositionArray.Add(utility::unreal::Vector3ToFVector(TetrahedronMeshSPtr->DynamicPointArray[CurIndex]->Position));
		NormalArray.Add(utility::unreal::Vector3ToFVector(TetrahedronMeshSPtr->RenderableNormalArray[CurIndex]));
		TexCoordArray.Add(utility::unreal::Vector2ToFVector2D(TetrahedronMeshSPtr->RenderableUvArray[CurIndex]));
	}
	
	ColorArray.Init(FColor::Red,
		TetrahedronMeshSPtr->RenderablePointIndexArray.Num());

	RuntimeMeshProviderStatic->CreateSectionFromComponents(0, 0, 0,
		PositionArray,
		TetrahedronMeshSPtr->RenderableTriangleIndexArray,
		NormalArray,
		TexCoordArray,ColorArray,TangentArray,
		ERuntimeMeshUpdateFrequency::Frequent, true);
	
	return true;
}

TSharedPtr<FTetrahedronMesh> UDynamicSolidComponent::GetTetrahedronMeshSPtr()
{
	return TetrahedronMeshSPtr;
}

void UDynamicSolidComponent::SetTetrahedronMeshSPtr(const TSharedPtr<FTetrahedronMesh>& TetMeshSPtr)
{
	TetrahedronMeshSPtr = TetMeshSPtr;
}

FString UDynamicSolidComponent::GetInitDynamicSolidPath()
{
	return InitDynamicSolidPath;
}

bool UDynamicSolidComponent::Initialize()
{
	if (InitDynamicSolidPath == "") return false;

	TetrahedronMeshSPtr = MakeShared<FTetrahedronMesh>(InitDynamicSolidPath);

	if (TetrahedronMeshSPtr == nullptr) return false;

	if (!InitializeRuntimeMeshComp()) return false;
	if (!UpdateRenderableData()) return false;

	return true;
}

// void UDynamicSolidComponent::PostEditChangeProperty(FPropertyChangedEvent& PropertyChangedEvent)
// {
	// FName PropName = PropertyChangedEvent.Property->GetFName();
	//
	// UNameProperty* prop = static_cast<UNameProperty*>(PropertyChangedEvent.Property);
	// FString MeshLoc;
	// prop->GetPropertyValue(&MeshLoc);
	//
	// if (PropName != FName("InitialSolidPath")) return;
	// if (InitialSolidPath == "") return;
	//
	// //to resolve tet solid
	// TetrahedronMeshSPtr = MakeShared<FTetrahedronMesh>(InitialSolidPath);
// }