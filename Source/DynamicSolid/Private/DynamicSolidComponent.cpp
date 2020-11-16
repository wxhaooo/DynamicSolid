// Fill out your copyright notice in the Description page of Project Settings.


#include "DynamicSolidComponent.h"
#include "RuntimeMeshComponent.h"
#include "FTetrahedronMesh.h"
#include "Providers/RuntimeMeshProviderStatic.h"
#include "UtilityUnreal.h"
#include "DrawDebugHelpers.h"

// Sets default values for this component's properties
UDynamicSolidComponent::UDynamicSolidComponent()
{
    // Set this component to be initialized when the game starts, and to be ticked every frame.  You can turn these features
    // off to improve performance if you don't need them.
    PrimaryComponentTick.bCanEverTick = true;

    Gravity = FVector(0., 0., -9.8f);
    bFixedTimeStep = false;

    RuntimeMeshComp = CreateDefaultSubobject<URuntimeMeshComponent>("RuntimeMeshComp");
    RuntimeMeshComp->SetupAttachment(this);

    bVisualDebugger = false;
}

// Called when the game starts
void UDynamicSolidComponent::BeginPlay()
{
    Super::BeginPlay();

    UE_LOG(LogTemp, Display, TEXT("Dynamic Solid Component Begin Play"));
    // ...
	
}

// Called every frame
void UDynamicSolidComponent::TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction)
{
    Super::TickComponent(DeltaTime, TickType, ThisTickFunction);

    float SimulatedTime
	    = (bFixedTimeStep & (!FMath::IsNearlyZero(TimeStep)))
	    ? TimeStep : DeltaTime;

    ApplyGravity(SimulatedTime);

    CollisionResolve();

    UpdateRenderableData();

    if(bVisualDebugger)
      VisualMotionDebugger();
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

bool UDynamicSolidComponent::CreateRenderableData()
{
    AActor* OwnerActor = GetOwner();
    if (TetrahedronMeshSPtr == nullptr) return false;
    if (OwnerActor == nullptr) return false;

    if (MeshMaterial != nullptr)
	    RuntimeMeshProviderStatic->SetupMaterialSlot
	    (0, TEXT("TriMat"), MeshMaterial);

    TArray<FVector> PositionArray;
    TArray<FVector> NormalArray;
    TArray<FVector2D> TexCoordArray;
    TArray<FRuntimeMeshTangent> TangentArray;
    TArray<FColor> ColorArray;

    FTransform OwnerActorTransform = OwnerActor->GetActorTransform();

    UE_LOG(LogTemp, Display, TEXT("PointIndexArraySize: %d\n"), TetrahedronMeshSPtr->RenderablePointIndexArray.Num());
    UE_LOG(LogTemp, Display, TEXT("PointUvArraySize: %d\n"), TetrahedronMeshSPtr->RenderableUvArray.Num());
    UE_LOG(LogTemp, Display, TEXT("PointNormalArraySize: %d\n"), TetrahedronMeshSPtr->RenderableNormalArray.Num());

    for (int i = 0; i < TetrahedronMeshSPtr->RenderablePointIndexArray.Num(); i++)
    {
	int CurIndex = TetrahedronMeshSPtr->RenderablePointIndexArray[i];
	//RuntimeMeshComponent会自动应用节点的Transform，所以还需要把Transform变回去
	//每轮都多很多次矩阵向量乘法
	FVector Tmp = utility::unreal::Vector3ToFVector(TetrahedronMeshSPtr->DynamicPointArray[CurIndex]->Position);
	Tmp = OwnerActorTransform.InverseTransformPosition(Tmp);
	PositionArray.Add(Tmp);
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

void UDynamicSolidComponent::SetTetrahedronMeshSPtr(TSharedPtr<FTetrahedronMesh> TetMeshPtr)
{
    TetrahedronMeshSPtr = TetMeshPtr;
}

FString UDynamicSolidComponent::GetInitDynamicSolidPath()
{
    return InitDynamicSolidPath;
}

bool UDynamicSolidComponent::Initialize()
{	
    if (InitDynamicSolidPath == "") return false;
	
    AActor* OwnerActor = GetOwner();
    if (OwnerActor == nullptr) return false;

    //实际是获取RootComponent的Transform，Actor的RootTransform又是DynamicSolidComponent
    //DynamicSolidComponent的RootComponent又是RuntimeMeshComp，所以它们获取的Transform是
    //一致的
    FTransform RootActorTransform = OwnerActor->GetActorTransform();
    
    TetrahedronMeshSPtr = MakeShared<FTetrahedronMesh>(InitDynamicSolidPath);
    //将载入的Mesh的顶点位置修正到移动后的位置
    TetrahedronMeshSPtr->ApplyRootActorTransform(RootActorTransform);
	
    if (TetrahedronMeshSPtr == nullptr) return false;

    if (!InitializeRuntimeMeshComp()) return false;
    if (!CreateRenderableData()) return false;

    return true;
}

bool UDynamicSolidComponent::ApplyGravity(float DeltaTime)
{
    if (TetrahedronMeshSPtr == nullptr) return false;
    TArray<TSharedPtr<FTetDynamicPoint>> DynamicPointArray = TetrahedronMeshSPtr->DynamicPointArray;
    for (int i = 0; i < DynamicPointArray.Num(); i++)
    {
	DynamicPointArray[i]->Velocity =
		DynamicPointArray[i]->Velocity + 
		utility::unreal::FVectorToVector3(Gravity) * DeltaTime;

	DynamicPointArray[i]->PostPosition =
		DynamicPointArray[i]->Position +
		DynamicPointArray[i]->Velocity * DeltaTime;
    }
    return true;
}

void UDynamicSolidComponent::CollisionResolve()
{
    if (TetrahedronMeshSPtr == nullptr) return;
    TArray<TSharedPtr<FTetDynamicPoint>> DynamicPointArray
        = TetrahedronMeshSPtr->DynamicPointArray;

    TArray<int> RenderablePointIndexArray = TetrahedronMeshSPtr->RenderablePointIndexArray;

    for (int i = 0; i < RenderablePointIndexArray.Num(); i++)
	{
        int CurPointIndex = RenderablePointIndexArray[i];
        TSharedPtr<FTetDynamicPoint> CurRenderablePoint = DynamicPointArray[CurPointIndex];

        FVector TraceStart = utility::unreal::Vector3ToFVector(CurRenderablePoint->Position);
        FVector TraceEnd = utility::unreal::Vector3ToFVector(CurRenderablePoint->PostPosition);
        FHitResult HitResult;

        FCollisionQueryParams CollisionQueryParams;
        CollisionQueryParams.AddIgnoredActor(GetOwner());
        CollisionQueryParams.bTraceComplex = true;
    	
    	if (GetWorld()->LineTraceSingleByChannel(HitResult,TraceStart,TraceEnd,ECollisionChannel::ECC_Visibility,CollisionQueryParams))
    	{
            // GEngine->AddOnScreenDebugMessage(-1, 3.f, FColor::Red, "23333");
            FVector ImpactPoint = HitResult.ImpactPoint;
            // DrawDebugLine(GetWorld(), TraceStart, ImpactPoint, FColor::Blue, true,10.f,0,5.);
    	}
	}

    for (int i = 0; i < DynamicPointArray.Num(); i++)
    {
        TSharedPtr<FTetDynamicPoint> CurDynamicPoint = DynamicPointArray[i];
    	CurDynamicPoint->Position = CurDynamicPoint->PostPosition;
    }
}  

bool UDynamicSolidComponent::UpdateRenderableData()
{
    AActor* OwnerActor = GetOwner();
    if (RuntimeMeshComp == nullptr) return false;
    if (TetrahedronMeshSPtr == nullptr) return false;
    if (OwnerActor == nullptr) return false;

    TArray<FVector> PositionArray;
    TArray<FVector> NormalArray;
    TArray<FVector2D> TexCoordArray;
    TArray<FRuntimeMeshTangent> TangentArray;
    TArray<FColor> ColorArray;

    FTransform OwnerActorTransform = OwnerActor->GetActorTransform();

    for (int i = 0; i < TetrahedronMeshSPtr->RenderablePointIndexArray.Num(); i++)
    {
	int CurIndex = TetrahedronMeshSPtr->RenderablePointIndexArray[i];
	//RuntimeMeshComponent会自动应用节点的Transform，所以还需要把Transform变回去
	//每轮都多很多次矩阵向量乘法
	FVector Tmp = utility::unreal::Vector3ToFVector(TetrahedronMeshSPtr->DynamicPointArray[CurIndex]->Position);
    // DrawDebugPoint(GetWorld(), Tmp, 5.f, FColor::Cyan, false, 0.1f);
        Tmp = OwnerActorTransform.InverseTransformPosition(Tmp);
	PositionArray.Add(Tmp);
	NormalArray.Add(utility::unreal::Vector3ToFVector(TetrahedronMeshSPtr->RenderableNormalArray[CurIndex]));
	TexCoordArray.Add(utility::unreal::Vector2ToFVector2D(TetrahedronMeshSPtr->RenderableUvArray[CurIndex]));
    }

    ColorArray.Init(FColor::Red,
	    TetrahedronMeshSPtr->RenderablePointIndexArray.Num());

    RuntimeMeshProviderStatic->UpdateSectionFromComponents(0, 0,
	    PositionArray,
	    TetrahedronMeshSPtr->RenderableTriangleIndexArray,
	    NormalArray,
	    TexCoordArray, ColorArray, TangentArray);

    return true;
}

bool UDynamicSolidComponent::ApplyInternalForce(float DeltaTime)
{
    if (TetrahedronMeshSPtr == nullptr) return false;
    //TArray<TSharedPtr<FTetDynamicPoint>> DynamicPointArray = TetrahedronMeshSPtr->DynamicPointArray;

    return true;
}

void UDynamicSolidComponent::VisualMotionDebugger()
{
    TArray<TSharedPtr<FTetDynamicPoint>> DynamicPointArray = TetrahedronMeshSPtr->DynamicPointArray;
    TArray<int> RenderablePointIndexArray = TetrahedronMeshSPtr->RenderablePointIndexArray;

    TSet<int> IsRenderablePoint;
    for(int i=0;i<RenderablePointIndexArray.Num();i++)
    {
	if (!IsRenderablePoint.Contains(RenderablePointIndexArray[i]))
	    IsRenderablePoint.Add(RenderablePointIndexArray[i]);
    }
    
    for (int i = 0; i < DynamicPointArray.Num(); i++)
    {
	TSharedPtr<FTetDynamicPoint> CurDynamicPoint = DynamicPointArray[i];
	FVector CurDynamicPos = utility::unreal::Vector3ToFVector(CurDynamicPoint->Position);
	// CurDynamicPos = GetOwner()->GetActorTransform().InverseTransformPosition(CurDynamicPos);
	if (IsRenderablePoint.Contains(i))
	    DrawDebugPoint(GetWorld(), CurDynamicPos, 5.f, FColor::Cyan, false, 0.1f);
	else
	    DrawDebugPoint(GetWorld(), CurDynamicPos, 5.f, FColor::Red, false, 0.1f);
    }
}

// #ifdef WITH_EDITOR
// void UDynamicSolidComponent::PostEditChangeProperty(FPropertyChangedEvent& PropertyChangedEvent)
// {
// 	
//     FName PropName = PropertyChangedEvent.Property->GetFName();
//     
//     UNameProperty* prop = static_cast<UNameProperty*>(PropertyChangedEvent.Property);
//     FString MeshLoc;
//     prop->GetPropertyValue(&MeshLoc);
//     
//     if (PropName != FName("InitDynamicSolidPath")) return;
//     if (!FPaths::FileExists(InitDynamicSolidPath)) return;
//
//     //to resolve tet solid
// 	// this->TetrahedronMeshSPtr = MakeShared<FTetrahedronMesh>(InitDynamicSolidPath);
//     Initialize();
//
// 	// this->TestValue = 1234;
// 	//
// 	// this->TestStrSPtr = MakeShared<FString>("23333");
// 	// this->TestStrPtr = new FString("455555");
//
// 	Super::PostEditChangeProperty(PropertyChangedEvent);
//
// }
// #endif