// Fill out your copyright notice in the Description page of Project Settings.

#include "DynamicSolidComponent.h"
#include "RuntimeMeshComponent.h"
#include "FTetrahedronMesh.h"
#include "Providers/RuntimeMeshProviderStatic.h"
#include "UtilityUnreal.h"
#include "DrawDebugHelpers.h"
#include "Kismet/KismetMathLibrary.h"
#include "UtilitySolver.h"

DEFINE_LOG_CATEGORY(LogDynamicSolidCompInit);

// Sets default values for this component's properties
UDynamicSolidComponent::UDynamicSolidComponent()
{
    // Set this component to be initialized when the game starts, and to be ticked every frame.  You can turn these features
    // off to improve performance if you don't need them.
    PrimaryComponentTick.bCanEverTick = true;

    GravityAccleration = FVector(0., 0., -9.8f);
    bFixedTimeStep = false;

    RuntimeMeshComp = CreateDefaultSubobject<URuntimeMeshComponent>("RuntimeMeshComp");
    RuntimeMeshComp->SetupAttachment(this);

    bVisualDebugger = false;

    YoungModulus = 1000.f;
    PoissonRatio = 0.4f;

    TimeStep = 0.0005f;
    MaxTimeStep = 0.05f;

    Mu = ComputeMu();
    Lambda = ComputeLambda();
}

// Called when the game starts
void UDynamicSolidComponent::BeginPlay()
{
    Super::BeginPlay();

    UE_LOG(LogTemp, Display, TEXT("Dynamic Solid Component Begin Play"));
}

// Called every frame
void UDynamicSolidComponent::TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction)
{
    Super::TickComponent(DeltaTime, TickType, ThisTickFunction);

    if (IntegrationMethod == EIntegrationMethod::IM_NONE) return;

    float SimulatedTime
	    = (bFixedTimeStep & (!FMath::IsNearlyZero(TimeStep)))
	    ? TimeStep : DeltaTime;

	if(IntegrationMethod == EIntegrationMethod::IM_EXP_FD)
	{
		//TimeStep should be enough small (<=0.0001),too slow
        EXP_ForwardDifference(SimulatedTime);
	}
    else if(IntegrationMethod == EIntegrationMethod::IM_EXP_VERLET)
    {
        EXP_Verlet(SimulatedTime);
    }
    else if(IntegrationMethod == EIntegrationMethod::IM_IMP_MPCG)
    {
        IMP_MPCG(SimulatedTime);
    }
    else if(IntegrationMethod == EIntegrationMethod::IM_IMP_PPCG)
    {
        IMP_PPCG(SimulatedTime);
    }
	
    CollisionResolve();

    UpdateRenderableData();

    if(bVisualDebugger)
      VisualMotionDebugger();

    // GEngine->AddOnScreenDebugMessage(-1, 3.f, FColor::Red, FString::SanitizeFloat(Mu));
    // GEngine->AddOnScreenDebugMessage(-1, 3.f, FColor::Cyan, FString::SanitizeFloat(Lambda));

	// UE_LOG(LogTemp, Display, TEXT("Lame's First Parameter: %f"), Mu);
    // UE_LOG(LogTemp, Display, TEXT("Lame's Second Parameter: %f"), Lambda);
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
    
    if (RuntimeMeshProviderStatic == nullptr)   return false;
    
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

    // UE_LOG(LogTemp, Display, TEXT("PointIndexArraySize: %d\n"), TetrahedronMeshSPtr->RenderablePointIndexArray.Num());
    // UE_LOG(LogTemp, Display, TEXT("PointUvArraySize: %d\n"), TetrahedronMeshSPtr->RenderableUvArray.Num());
    // UE_LOG(LogTemp, Display, TEXT("PointNormalArraySize: %d\n"), TetrahedronMeshSPtr->RenderableNormalArray.Num());

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

TSharedPtr<FTetrahedronMesh> UDynamicSolidComponent::ConvertTriangleToTetrahedronMesh()
{
    TSharedPtr<FTetrahedronMesh> TetrahedronMeshSPtrOut;

    return TetrahedronMeshSPtrOut;
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
    // if (!CreateRenderableData()) return false;

    return true;
}

VectorX<real> UDynamicSolidComponent::ComputeWindForce(float SimulatedTime)
{
    VectorX<real> WindForce;
    if (TetrahedronMeshSPtr == nullptr) return WindForce;

    return WindForce;
}

VectorX<real> UDynamicSolidComponent::ComputeGravity(float SimulatedTime)
{
    VectorX<real> Gravity;
    if (TetrahedronMeshSPtr == nullptr) return Gravity;
	
    TArray<TSharedPtr<FTetDynamicPoint>> DynamicPointArray = TetrahedronMeshSPtr->DynamicPointArray;
    Gravity.setZero(DynamicPointArray.Num() * 3);
	
	for (int i = 0; i < DynamicPointArray.Num(); i++)
    {
		Gravity.block<3, 1>(i * 3, 0)
                = DynamicPointArray[i]->Mass * utility::unreal::FVectorToVector3(GravityAccleration);
    }

    return Gravity;
}

VectorX<real> UDynamicSolidComponent::ComputeInternalForce(float SimulatedTime)
{
    VectorX<real> InternalForceOut;

    if (TetrahedronMeshSPtr == nullptr) return InternalForceOut;
    TArray<TSharedPtr<FTetDynamicPoint>> DynamicPointArray = TetrahedronMeshSPtr->DynamicPointArray;
    TArray<TSharedPtr<FDynamicTetrahedron>> DynamicTetArray = TetrahedronMeshSPtr->DynamicTetrahedronArray;
    InternalForceOut.setZero(DynamicPointArray.Num() * 3);

    for (int i = 0; i < DynamicTetArray.Num(); i++)
	{
        TSharedPtr<FDynamicTetrahedron> CurTet = DynamicTetArray[i];
        Vector12<real> InternalForce = CurTet->ComputeForce(Mu, Lambda);
        // Vector12<real> InternalForce = CurTet->ComputeInternalForceAsSifakisWay(Mu, Lambda);

    	// if(i == 3)
     //    GEngine->AddOnScreenDebugMessage(-1, 3.f,
     //        FColor::Red, FString::SanitizeFloat(InternalForce.norm()));
		TTuple<int, int, int, int> PointIndices = CurTet->PointIndices();
        int p0Index = PointIndices.Get<0>(), p1Index = PointIndices.Get<1>();
        int p2Index = PointIndices.Get<2>(), p3Index = PointIndices.Get<3>();

        InternalForceOut.block<3, 1>(p0Index * 3, 0) += InternalForce.block<3, 1>(0, 0);
        InternalForceOut.block<3, 1>(p1Index * 3, 0) += InternalForce.block<3, 1>(3, 0);
        InternalForceOut.block<3, 1>(p2Index * 3, 0) += InternalForce.block<3, 1>(6, 0);
        InternalForceOut.block<3, 1>(p3Index * 3, 0) += InternalForce.block<3, 1>(9, 0);

	}
	
    return InternalForceOut;
}

VectorX<real> UDynamicSolidComponent::ComputeConstraintForce(float SimulatedTime)
{
    VectorX<real> ConstraintForce;

    if (TetrahedronMeshSPtr == nullptr) return ConstraintForce;
    //TArray<TSharedPtr<FTetDynamicPoint>> DynamicPointArray = TetrahedronMeshSPtr->DynamicPointArray;

    return ConstraintForce;
}

bool UDynamicSolidComponent::EXP_ForwardDifference(float SimulatedTime)
{
    if (TetrahedronMeshSPtr == nullptr) return false;
	
    VectorX<real> Gravity = ComputeGravity(SimulatedTime);
    VectorX<real> InternalForce = ComputeInternalForce(SimulatedTime);
    // 
    // InternalForce.setZero();
    TArray<TSharedPtr<FTetDynamicPoint>> DynamicPointArray = TetrahedronMeshSPtr->DynamicPointArray;

	//Forward Difference
    for (int i = 0; i < DynamicPointArray.Num(); i++)
	{
        TSharedPtr<FTetDynamicPoint> CurPoint = DynamicPointArray[i];
        Vector3<real> GravityInPoint = Gravity.block<3, 1>(i * 3, 0);
        Vector3<real> InternalForceInPoint = InternalForce.block<3, 1>(i * 3, 0);

        Vector3<real> TotalForceInPoint;
        TotalForceInPoint.setZero();
    	//for test internal
        if (CurPoint->PointIndex > 3)
        TotalForceInPoint = CurPoint->MassInv * (GravityInPoint + InternalForceInPoint);

        CurPoint->Velocity = CurPoint->Velocity + TotalForceInPoint * SimulatedTime;
        CurPoint->PostPosition = CurPoint->Position + CurPoint->Velocity * SimulatedTime;
	}

    return true;
}

bool UDynamicSolidComponent::EXP_Verlet(float SimulatedTime)
{
    VectorX<real> Gravity = ComputeGravity(SimulatedTime);
    VectorX<real> InternalForce = ComputeInternalForce(SimulatedTime);
    // InternalForce.setZero();

    TArray<TSharedPtr<FTetDynamicPoint>> DynamicPointArray = TetrahedronMeshSPtr->DynamicPointArray;

    //Verlet Integration
    for (int i = 0; i < DynamicPointArray.Num(); i++)
    {
        TSharedPtr<FTetDynamicPoint> CurPoint = DynamicPointArray[i];
        Vector3<real> GravityInPoint = Gravity.block<3, 1>(i * 3, 0);
        Vector3<real> InternalForceInPoint = InternalForce.block<3, 1>(i * 3, 0);

        Vector3<real> TotalAccInPoint;
        TotalAccInPoint.setZero();

        if (CurPoint->PointIndex >= 49)
        TotalAccInPoint = CurPoint->MassInv * (GravityInPoint + InternalForceInPoint);

    	//Verlet Integration
        CurPoint->Velocity = CurPoint->Velocity + CurPoint->Accleration * SimulatedTime * 0.5f;
        CurPoint->PostPosition = CurPoint->Position + CurPoint->Velocity * SimulatedTime;
        CurPoint->Velocity = CurPoint->Velocity + TotalAccInPoint * SimulatedTime * 0.5f;

        CurPoint->Accleration = TotalAccInPoint;
    }
    return true;
}

bool UDynamicSolidComponent::IMP_MPCG(float SimulatedTime)
{
	//MPCG in Baraff98
    VectorX<real> Gravity = ComputeGravity(SimulatedTime);
    VectorX<real> ExternalForce = Gravity;
    VectorX<real> VelocityConstraint_Collsion = GetCollisionConstraints(SimulatedTime);
    TTuple<SpMat<real>, VectorX<real>> ImplicitEquation = GetImplicitEquation(ExternalForce);
    TArray<Matrix3x3<real>> PositionConstraints = GetPositionConstraints();
	
	SpMat<real> A = ImplicitEquation.Get<0>();
    VectorX<real> b = ImplicitEquation.Get<1>();

    utility::solver::MPCG(A, b, VelocityConstraint_Collsion, PositionConstraints, Tolerance, MaxEpoch);
	
    return true;
}

bool UDynamicSolidComponent::IMP_PPCG(float SimulatedTime)
{
    return true;
}

TTuple<SpMat<real>, VectorX<real>> UDynamicSolidComponent::GetImplicitEquation(const VectorX<real>& ExternalForce)
{
    SpMat<real> A;
    VectorX<real> b;


    return TTuple<SpMat<real>, VectorX<real>>(A, b);
}

TArray<Matrix3x3<real>> UDynamicSolidComponent::GetPositionConstraints()
{
    TArray<Matrix3x3<real>> PositionConstraintsOut;
    if (TetrahedronMeshSPtr == nullptr) return PositionConstraintsOut;

    TArray<TSharedPtr<FTetDynamicPoint>> DynamicPointArray = TetrahedronMeshSPtr->DynamicPointArray;
    PositionConstraintsOut.SetNum(DynamicPointArray.Num());

	//other position constraints
    return PositionConstraintsOut;
}

VectorX<real> UDynamicSolidComponent::GetCollisionConstraints(float SimulatedTime)
{
    VectorX<real> VelocityConstraint_CollsionOut;

    if (TetrahedronMeshSPtr == nullptr) return VelocityConstraint_CollsionOut;
	
    TArray<TSharedPtr<FTetDynamicPoint>> DynamicPointArray
        = TetrahedronMeshSPtr->DynamicPointArray;

    VelocityConstraint_CollsionOut.setZero(DynamicPointArray.Num() * 3);

    TArray<int> RenderablePointIndexArray = TetrahedronMeshSPtr->RenderablePointIndexArray;

    for (int i = 0; i < RenderablePointIndexArray.Num(); i++)
    {
        //Move Position Forward 
        int CurPointIndex = RenderablePointIndexArray[i];
        TSharedPtr<FTetDynamicPoint> CurRenderablePoint = DynamicPointArray[CurPointIndex];

        CurRenderablePoint->PostPosition = CurRenderablePoint->Position +
            CurRenderablePoint->Velocity * SimulatedTime;

        FVector TraceStart = utility::unreal::Vector3ToFVector(CurRenderablePoint->Position);
        FVector TraceEnd = utility::unreal::Vector3ToFVector(CurRenderablePoint->PostPosition);
        FHitResult HitResult;

        FCollisionQueryParams CollisionQueryParams;
        CollisionQueryParams.AddIgnoredActor(GetOwner());
        CollisionQueryParams.bTraceComplex = true;

        FVector ExpectedPosition = TraceEnd;
        if (GetWorld()->LineTraceSingleByChannel(HitResult, TraceStart,
            TraceEnd, ECollisionChannel::ECC_Visibility, CollisionQueryParams))
            ExpectedPosition = HitResult.ImpactPoint;
    	
        Vector3<real> ExpectedVelocity = utility::unreal::FVectorToVector3((ExpectedPosition - TraceStart) / SimulatedTime);
        VelocityConstraint_CollsionOut.block<3, 1>(CurPointIndex * 3, 0) = ExpectedVelocity;
    }
	
    return VelocityConstraint_CollsionOut;
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
            // CurRenderablePoint->PostPosition = utility::unreal::FVectorToVector3(ImpactPoint);
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

float UDynamicSolidComponent::ComputeMu()
{
    return YoungModulus / (2.f * (1.f + PoissonRatio));
}

float UDynamicSolidComponent::ComputeLambda()
{
    return (YoungModulus * PoissonRatio) /
        ((1.f + PoissonRatio) * (1.f - 2.f * PoissonRatio));
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

bool UDynamicSolidComponent::EditorStateSync()
{
    InitializeRuntimeMeshComp();

    RuntimeMeshProviderStatic->CreateSectionFromComponents(0, 0, 0,
        SMPositionArray,
        SMTriangleIndexArray,
        SMNormalArray,
        SMUvArray, SMColorArray, SMTangentArray,
        ERuntimeMeshUpdateFrequency::Frequent, true);

    return true;
}

bool UDynamicSolidComponent::LoadInitDynamicSolidMesh()
{
    SMPositionArray.Reset();    SMUvArray.Reset();
    SMNormalArray.Reset();  SMColorArray.Reset();
    SMTriangleIndexArray.Reset();   SMTangentArray.Reset();

    if (InitDynamicSolidMesh != nullptr) {

        FPositionVertexBuffer* PositionVertexBuffer = &(InitDynamicSolidMesh->RenderData->LODResources[0].VertexBuffers.PositionVertexBuffer);
        FRawStaticIndexBuffer* IndexBuffer = &(InitDynamicSolidMesh->RenderData->LODResources[0].IndexBuffer);
        FStaticMeshVertexBuffers* VertexBuffer = &(InitDynamicSolidMesh->RenderData->LODResources[0].VertexBuffers);

        int PositionVertexNum = PositionVertexBuffer->GetNumVertices();
        for (int i = 0; i < PositionVertexNum; i++)
        {
            FVector VertexPos = PositionVertexBuffer->VertexPosition(i);
            FVector VertexPosInWorld = GetOwner()->GetActorLocation() + GetComponentTransform().TransformVector(VertexPos);
            SMPositionArray.Add(VertexPosInWorld);
            //add other information
        }

        for (int i = 0; i < IndexBuffer->GetNumIndices(); i++)
        {
            SMTriangleIndexArray.Add(IndexBuffer->GetArrayView()[i]);
        }

        UE_LOG(LogDynamicSolidCompInit, Display, TEXT("StaticMesh Position Number:%d"), SMPositionArray.Num());
        UE_LOG(LogDynamicSolidCompInit, Display, TEXT("StaticMesh Triangle Number:%d"), SMTriangleIndexArray.Num() / 3);
    }
	
    return true;
}

#ifdef WITH_EDITOR
void UDynamicSolidComponent::PostEditChangeProperty(FPropertyChangedEvent& PropertyChangedEvent)
{
    Super::PostEditChangeProperty(PropertyChangedEvent);

    FName PropName = PropertyChangedEvent.Property->GetFName();
    
    UNameProperty* Prop = static_cast<UNameProperty*>(PropertyChangedEvent.Property);
    FString MeshLoc;
    Prop->GetPropertyValue(&MeshLoc);
    
    if (PropName == FName("YoungModulus") ||
        PropName == FName("PoissonRatio"))
    {
        if (PropName == FName("PoissonRatio"))
            PoissonRatio = FMath::Clamp(PoissonRatio, -0.99f, 0.49f);
    	
        Mu = ComputeMu();
        Lambda = ComputeLambda();
    }

	if(PropName == FName("InternalForceDamping"))
        InternalForceDamping = FMath::Clamp(InternalForceDamping, 0.0f, 1.0f);

    if (PropName == FName("TimeStep") || PropName == FName("MaxTimeStep"))
    {
        if (PropName == FName("MaxTimeStep"))
            MaxTimeStep = FMath::Clamp(MaxTimeStep, 0.f, 1.f);
    	
        TimeStep = FMath::Clamp(TimeStep, 0.f, MaxTimeStep);
    }

    if (PropName == FName("InitDynamicSolidMesh"))
        LoadInitDynamicSolidMesh();
}
#endif