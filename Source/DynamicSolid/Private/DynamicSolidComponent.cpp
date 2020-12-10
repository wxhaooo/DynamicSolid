// Fill out your copyright notice in the Description page of Project Settings.

#include "DynamicSolidComponent.h"
#include "RuntimeMeshComponent.h"
#include "FTetrahedronMesh.h"
#include "Providers/RuntimeMeshProviderStatic.h"
#include "UtilityUnreal.h"
#include "DrawDebugHelpers.h"
#include "ImageUtils.h"
#include "Kismet/KismetMathLibrary.h"
#include "UtilitySolver.h"
#include "Async/ParallelFor.h"
#include "UtilityDebug.h"
#include "Misc/FileHelper.h"
// #include "Camera/PlayerCameraManager.h"

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

    YoungModulus = 1e6f;
    PoissonRatio = 0.4f;

    TimeStep = 0.005f;
    MaxTimeStep = 0.05f;

    Mu = ComputeMu();
    Lambda = ComputeLambda();

    bFixedTimeStep = true;
    bForcePD = true;
	
    SolverTolerance = 1e-5;
    CollisionOffset = 0.2f;
    InternalForceDamping = 0.005f;
    MaxEpoch = 1000;
    Density = 600.f;

    bIsVisualPointCloud = false;
    bIsVisualNormal = false;
    bSaveSimulatedFrame = false;
    VisualNormalLength = 15.f;
}

// Called when the game starts
void UDynamicSolidComponent::BeginPlay()
{
    Super::BeginPlay();

    TotalSimulatorTime = 0.f;
    FrameNumber = 0;

    UE_LOG(LogTemp, Display, TEXT("Dynamic Solid Component Begin Play"));
}

// Called every frame
void UDynamicSolidComponent::TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction)
{
    Super::TickComponent(DeltaTime, TickType, ThisTickFunction);

    if (IntegrationMethod == EIntegrationMethod::IM_NONE) return;

    FrameNumber++;
	
    float SimulatedTime
	    = (bFixedTimeStep & (!FMath::IsNearlyZero(TimeStep)))
	    ? TimeStep : DeltaTime;

    TotalSimulatorTime += SimulatedTime;
    // GEngine->AddOnScreenDebugMessage(-1, 3.f,
    //     FColor::Blue, FString::SanitizeFloat(TotalSimulatorTime));

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
	
    UpdateRenderableData();

    if(bIsVisualPointCloud)
      VisualMotionDebugger();

    if (bSaveSimulatedFrame)
        SaveSimulateFrame();
    // GEngine->AddOnScreenDebugMessage(-1, 3.f, FColor::Red, FString::SanitizeFloat(Mu));
    // GEngine->AddOnScreenDebugMessage(-1, 3.f, FColor::Cyan, FString::SanitizeFloat(Lambda));

	// UE_LOG(LogTemp, Display, TEXT("Lame's First Parameter: %f"), Mu);
    // UE_LOG(LogTemp, Display, TEXT("Lame's Second Parameter: %f"), Lambda);
}

bool UDynamicSolidComponent::SaveSimulateFrame()
{
    UGameViewportClient* gameViewport = GEngine->GameViewport;
    FViewport* InViewport = gameViewport->Viewport;
    TArray<FColor> Bitmap;
    FIntRect Rect(0, 0, InViewport->GetSizeXY().X, InViewport->GetSizeXY().Y);
    // GEngine->AddOnScreenDebugMessage(-1, 3.f, FColor::Red, FString::FromInt(InViewport->GetSizeXY().X));
    // GEngine->AddOnScreenDebugMessage(-1, 3.f, FColor::Red, FString::FromInt(InViewport->GetSizeXY().Y));
    bool bScreenshotSuccessful = GetViewportScreenShot(InViewport, Bitmap, Rect);

    if (!bScreenshotSuccessful) return false;
  
    FIntVector Size(InViewport->GetSizeXY().X, InViewport->GetSizeXY().Y, 0);
    TArray<uint8> CompressedBitmap;
    FString ScreenShotName = TEXT("E:/Code/Unreal/DynamicSolid/out.png");
    FImageUtils::CompressImageArray(Size.X, Size.Y, Bitmap, CompressedBitmap);
    if(FrameNumber == 1)
		FFileHelper::SaveArrayToFile(CompressedBitmap, *ScreenShotName);
    return true;
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
    if (bIsVisualPointCloud) return false;
	
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

	//compute normals runtime
    TetrahedronMeshSPtr->ComputeNormals();

    // UE_LOG(LogTemp, Display, TEXT("PointIndexArraySize: %d\n"), TetrahedronMeshSPtr->RenderablePointIndexArray.Num());
    // UE_LOG(LogTemp, Display, TEXT("PointUvArraySize: %d\n"), TetrahedronMeshSPtr->RenderableUvArray.Num());
    // UE_LOG(LogTemp, Display, TEXT("PointNormalArraySize: %d\n"), TetrahedronMeshSPtr->RenderableNormalArray.Num());

    for (int i = 0; i < TetrahedronMeshSPtr->RenderablePointIndexArray.Num(); i++)
    {
	int CurIndex = TetrahedronMeshSPtr->RenderablePointIndexArray[i];
	//RuntimeMeshComponent会自动应用节点的Transform，所以还需要把Transform变回去
	//每轮都多很多次矩阵向量乘法
	FVector Tmp = utility::unreal::Vector3ToFVector(TetrahedronMeshSPtr->DynamicPointArray[CurIndex]->Position);
    Tmp *= UnitTransferMToCM;
    FVector CurNormal = utility::unreal::Vector3ToFVector(TetrahedronMeshSPtr->RenderableNormalArray[CurIndex]);
    // FVector CurNormal = utility::unreal::Vector3ToFVector(TetrahedronMeshSPtr->DynamicPointArray[CurIndex]->ComputeNormal());
    if (bIsVisualNormal) {
        if (IntegrationMethod == EIntegrationMethod::IM_NONE)
            DrawDebugLine(GetWorld(), Tmp, Tmp + CurNormal * VisualNormalLength, FColor::Cyan, true, -1);
        else
    		DrawDebugLine(GetWorld(), Tmp, Tmp + CurNormal * VisualNormalLength, FColor::Cyan, false, 0.1f);
    }
    //因为RuntimeComponent会自动加一个Transform上去，所以先乘以一个InverseTransform
    Tmp = OwnerActorTransform.InverseTransformPosition(Tmp);
   
    // Tmp = OwnerActorTransform.TransformPosition(Tmp);
	PositionArray.Add(Tmp);
    NormalArray.Add(CurNormal);
	// NormalArray.Add(utility::unreal::Vector3ToFVector(TetrahedronMeshSPtr->RenderableNormalArray[CurIndex]));
	TexCoordArray.Add(utility::unreal::Vector2ToFVector2D(TetrahedronMeshSPtr->RenderableUvArray[CurIndex]));
    FRuntimeMeshTangent CurTangent;
    CurTangent.TangentX = utility::unreal::Vector3ToFVector(TetrahedronMeshSPtr->RenderableTangentArray[CurIndex]);
    TangentArray.Add(CurTangent);
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
    //计算每个点的质量
    TetrahedronMeshSPtr->ComputeMassPerPoint(Density);
    if (TetrahedronMeshSPtr == nullptr) return false;

    if (!InitializeRuntimeMeshComp()) return false;
    if (!CreateRenderableData()) return false;

    return true;
}

bool UDynamicSolidComponent::ComputeMorphForce(float SimulatedTime, SpMat<real>& A, VectorX<real>& b)
{
 //    TArray<TSharedPtr<FTetDynamicPoint>> DynamicPointArray = TetrahedronMeshSPtr->DynamicPointArray;
 //    TArray<TSharedPtr<FDynamicTetrahedron>> DynamicTetrahedronArray = TetrahedronMeshSPtr->DynamicTetrahedronArray;
 //
	// //here to add morph force
    //b +=(dt * f)
    //A -= (dt2 * K) (This need it)
	
    return true;
}

VectorX<real> UDynamicSolidComponent::ComputeWindForce(float SimulatedTime)
{
    //Here to add wind force
    VectorX<real> WindForce;
    if (TetrahedronMeshSPtr == nullptr) return WindForce;

    return WindForce;
}

bool UDynamicSolidComponent::ComputeWindForce(float SimulatedTime, SpMat<real>& A, VectorX<real>& b)
{
    //Here to add wind force
	//b +=(dt * f)
	//A -= (dt2 * K) (This doesn't need it)
    return true;
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

bool UDynamicSolidComponent::ComputeGravity(float SimulatedTime, SpMat<real>& A, VectorX<real>& b)
{
    //b +=(dt * f)
    //A -= (dt2 * K) (This doesn't need it)
	
    TArray<TSharedPtr<FTetDynamicPoint>> DynamicPointArray = TetrahedronMeshSPtr->DynamicPointArray;

    Vector3<real> g = utility::unreal::FVectorToVector3(GravityAccleration);
	
    for (int i = 0; i < DynamicPointArray.Num(); i++)
    {
        b.block<3, 1>(i * 3, 0)+= SimulatedTime * DynamicPointArray[i]->Mass * g;
    }

    // ParallelFor(DynamicPointArray.Num(), [&](int Idx)
    //     {
    // b.block<3, 1>(Idx * 3, 0) += SimulatedTime * DynamicPointArray[Idx]->Mass * g;
    //     });

    return true;
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
        Vector12<real> InternalForce = CurTet->ComputeForceAsTensorWay(Mu, Lambda, InternalEnergyModel);
        // Vector12<real> InternalForce = CurTet->ComputeInternalForceAsSifakisWay(Mu, Lambda);

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

SpMat<real> UDynamicSolidComponent::ComputeInternalForce(float SimulatedTime, SpMat<real>& A, VectorX<real>& b)
{
    TArray<TSharedPtr<FTetDynamicPoint>> DynamicPointArray = TetrahedronMeshSPtr->DynamicPointArray;
    TArray<TSharedPtr<FDynamicTetrahedron>> DynamicTetArray = TetrahedronMeshSPtr->DynamicTetrahedronArray;

    int n = DynamicPointArray.Num() * 3;
    SpMat<real> K(n, n);
    K.setZero();
    // MatrixX<real> K;
    // K.setZero(n, n);
    for (int nn = 0; nn < DynamicTetArray.Num(); nn++)
	{
        TSharedPtr<FDynamicTetrahedron> CurTet = DynamicTetArray[nn];
        Matrix<real, 12, 12> Ki = CurTet->K(Mu, Lambda, InternalEnergyModel, bForcePD);
        TTuple<int, int, int, int> TetPointIndices = CurTet->PointIndices();
    	
        int p0Index = TetPointIndices.Get<0>(), p1Index = TetPointIndices.Get<1>();
        int p2Index = TetPointIndices.Get<2>(), p3Index = TetPointIndices.Get<3>();
    	
        TArray<int> AuxArray{ p0Index,p1Index,p2Index,p3Index };
 
    	// // for dense matrix K
     //     for (int i = 0; i < AuxArray.Num(); i++)
     //     {
     //         int fiIdx = AuxArray[i];
     //         for (int j = 0; j < AuxArray.Num(); j++)
	    //      {
     //             int xjIdx = AuxArray[j];
     //             Matrix3x3<real> fixj = Ki.block<3, 3>(3 * i, 3 * j);
     //             K.block<3, 3>(fiIdx * 3, xjIdx * 3) += fixj;
	    //      }
     //     }
 
    	//for sparse matrix K
        for (int i = 0; i < AuxArray.Num(); i++)
        {
            int fiIdx = AuxArray[i];
            for (int j = 0; j < AuxArray.Num(); j++)
            {
                int xjIdx = AuxArray[j];
                Matrix3x3<real> fixj = Ki.block<3, 3>(3 * i, 3 * j);
        
                for (int p = 0; p < 3; p++)
            	{
                    for (int q = 0; q < 3; q++)
            		{
                        K.coeffRef(fiIdx * 3 + p, xjIdx * 3 + q) += fixj(p,q);
            		}
            	}
            }
        }
	}
	
    A = (A - SimulatedTime * (SimulatedTime + InternalForceDamping) * K).eval();
    VectorX<real> v;
    v.setZero(n);
 
    for (int i = 0; i < DynamicPointArray.Num(); i++)
	{
        v.block<3, 1>(3 * i, 0) += DynamicPointArray[i]->Velocity;
	}
 
    // ParallelFor(DynamicPointArray.Num(), [&](int Idx)
    //     {
    //         v.block<3, 1>(3 * Idx, 0) += DynamicPointArray[Idx]->Velocity;
    //     });
 
    VectorX<real> f = ComputeInternalForce(SimulatedTime);
		// GEngine->AddOnScreenDebugMessage(-1, 3.f, FColor::Red, FString::SanitizeFloat(f.norm()));
    b += (SimulatedTime * (f + (SimulatedTime + InternalForceDamping) * K * v));

    return K;
    // return K.sparseView();
}

VectorX<real> UDynamicSolidComponent::ComputeConstraintForce(float SimulatedTime)
{
    VectorX<real> ConstraintForce;

    if (TetrahedronMeshSPtr == nullptr) return ConstraintForce;
    //TArray<TSharedPtr<FTetDynamicPoint>> DynamicPointArray = TetrahedronMeshSPtr->DynamicPointArray;

    return ConstraintForce;
}

bool UDynamicSolidComponent::ComputeOffsetCorrection(float SimulatedTime, SpMat<real>& K, SpMat<real>& A, VectorX<real>& b,const VectorX<real>& OffsetMask)
{
    VectorX<real> y;
    y = OffsetMask * CollisionOffset;

    b += SimulatedTime * K * y;
	
    return true;
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

    CollisionResolve();

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

    CollisionResolve();

    return true;
}

bool UDynamicSolidComponent::IMP_MPCG(float SimulatedTime)
{
    if (TetrahedronMeshSPtr == nullptr)
    {
        GEngine->AddOnScreenDebugMessage(-1, 3.f,
            FColor::Black, TEXT("TetrahedronMeshSPtr is nullptr"));
        return false;
    }
    //MPCG in Baraff98
    TTuple<SpMat<real>,VectorX<real>, SpMat<real>> ImplicitEquation = GetImplicitEquation(SimulatedTime);

    SpMat<real> A = ImplicitEquation.Get<0>();
    VectorX<real> b = ImplicitEquation.Get<1>();
    SpMat<real> K = ImplicitEquation.Get<2>();
	
    TArray<Matrix3x3<real>> PositionConstraints = GetPositionConstraints();
    TTuple<VectorX<real>,VectorX<real>> CollisionConstraints = GetCollisionConstraints(SimulatedTime, PositionConstraints);

    VectorX<real> VelocityConstraint_Collision = CollisionConstraints.Get<0>();
    VectorX<real> OffsetMask_Collision = CollisionConstraints.Get<1>();
    // UE_LOG(LogSolver, Display, TEXT("%s"), *utility::debug::ConvertLogStr(VelocityConstraint_Collision, 0, VelocityConstraint_Collision.size()));
    //offset correction
    ComputeOffsetCorrection(SimulatedTime, K, A, b, OffsetMask_Collision);
    // GEngine->AddOnScreenDebugMessage(-1, 3.f, FColor::Black, FString::SanitizeFloat(Tolerance));
    VectorX<real> DeltaV = utility::solver::MPCG(A, b, VelocityConstraint_Collision, PositionConstraints, SolverTolerance, MaxEpoch);
    // UE_LOG(LogSolver, Display, TEXT("%s"),*utility::debug::ConvertLogText(DeltaV, 0, DeltaV.size()));
    AdvanceStep(SimulatedTime,DeltaV);
    return true;
}

bool UDynamicSolidComponent::IMP_PPCG(float SimulatedTime)
{
    if (TetrahedronMeshSPtr == nullptr)
    {
        GEngine->AddOnScreenDebugMessage(-1, 3.f,
            FColor::Black, TEXT("TetrahedronMeshSPtr is nullptr"));
        return false;
    }
    //PPCG in Tamstorf15
    TTuple<SpMat<real>, VectorX<real>, SpMat<real>> ImplicitEquation = GetImplicitEquation(SimulatedTime);

    SpMat<real> A = ImplicitEquation.Get<0>();
    VectorX<real> b = ImplicitEquation.Get<1>();
    SpMat<real> K = ImplicitEquation.Get<2>();

    TArray<Matrix3x3<real>> PositionConstraints = GetPositionConstraints();
    TTuple<VectorX<real>, VectorX<real>> CollisionConstraints = GetCollisionConstraints(SimulatedTime, PositionConstraints);

    VectorX<real> VelocityConstraint_Collision = CollisionConstraints.Get<0>();
    VectorX<real> OffsetMask_Collision = CollisionConstraints.Get<1>();
    // UE_LOG(LogSolver, Display, TEXT("%s"), *utility::debug::ConvertLogStr(VelocityConstraint_Collision, 0, VelocityConstraint_Collision.size()));
    //offset correction
    ComputeOffsetCorrection(SimulatedTime, K, A, b, OffsetMask_Collision);

    // MatrixX<real> DenseMat(A);
    // Eigen::EigenSolver<MatrixX<real>> EigenSolver(DenseMat);
    //
    // for (int i = 0; i < EigenSolver.eigenvalues().size(); i++)
    // {
    //     if (EigenSolver.eigenvalues()(i).real() < 0)
    //         GEngine->AddOnScreenDebugMessage(-1, 3.f, FColor::Red, "Eigenvalue is negative");
    // }
	
    // GEngine->AddOnScreenDebugMessage(-1, 3.f, FColor::Black, FString::SanitizeFloat(Tolerance));
    VectorX<real> DeltaV = utility::solver::PPCG(A, b, VelocityConstraint_Collision, PositionConstraints);
    // UE_LOG(LogSolver, Display, TEXT("%s"),*utility::debug::ConvertLogText(DeltaV, 0, DeltaV.size()));
    AdvanceStep(SimulatedTime, DeltaV);
    return true;
}

bool UDynamicSolidComponent::AdvanceStep(float SimulatedTime, const VectorX<real>& DeltaV)
{
    if (TetrahedronMeshSPtr == nullptr) return false;

    TArray<TSharedPtr<FTetDynamicPoint>> DynamicPointArray = TetrahedronMeshSPtr->DynamicPointArray;

    for (int i = 0; i < DynamicPointArray.Num(); i++)
	{
        TSharedPtr<FTetDynamicPoint> CurDynamicPoint = DynamicPointArray[i];
        Vector3<real> CurPointDeltaV = DeltaV.block<3, 1>(3 * i, 0);
        // utility::debug::AddOnScreen(-1, 3.f, FColor::Red, CurPointDeltaV);
        CurDynamicPoint->Velocity += CurPointDeltaV;
        CurDynamicPoint->Position += CurDynamicPoint->Velocity * SimulatedTime;
	}
	
	return true;
}

TTuple<SpMat<real>,VectorX<real>, SpMat<real>> UDynamicSolidComponent::GetImplicitEquation(real SimulatedTime)
{
    TArray<TSharedPtr<FTetDynamicPoint>> DynamicPointArray = TetrahedronMeshSPtr->DynamicPointArray;
    TArray<TSharedPtr<FDynamicTetrahedron>> DynamicTetrahedronArray = TetrahedronMeshSPtr->DynamicTetrahedronArray;

    int n = DynamicPointArray.Num() * 3;
    real dt = SimulatedTime;
    real dt2 = dt * dt;
	
    SpMat<real> A(n, n);
    VectorX<real> b;

    A.setZero();
    b.setZero(n);

    // Mass Matrix
     for (int i = 0; i < DynamicPointArray.Num(); i++)
     {
         A.coeffRef(3 * i, 3 * i) = DynamicPointArray[i]->Mass;
         A.coeffRef(3 * i + 1, 3 * i + 1) = DynamicPointArray[i]->Mass;
         A.coeffRef(3 * i + 2, 3 * i + 2) = DynamicPointArray[i]->Mass;
     }

	// //parallel set mass matrix
 //    ParallelFor(DynamicPointArray.Num(), [&](int Idx)
 //        {
	// 	    A.coeffRef(3 * Idx, 3 * Idx) = DynamicPointArray[Idx]->Mass;
	// 	    A.coeffRef(3 * Idx + 1, 3 * Idx + 1) = DynamicPointArray[Idx]->Mass;
	// 	    A.coeffRef(3 * Idx + 2, 3 * Idx + 2) = DynamicPointArray[Idx]->Mass;
 //        });

	//external force
    ComputeGravity(SimulatedTime, A, b);
    // ComputeWindForce(SimulatedTime, A, b);
    // ComputeMorphForce(SimulatedTime, A, b);

	//internal force
    SpMat<real> K = ComputeInternalForce(SimulatedTime, A, b);

    return TTuple<SpMat<real>, VectorX<real>, SpMat<real>>(A, b, K);
}

TArray<Matrix3x3<real>> UDynamicSolidComponent::GetPositionConstraints()
{
    TArray<Matrix3x3<real>> PositionConstraintsOut;
    if (TetrahedronMeshSPtr == nullptr) return PositionConstraintsOut;

    TArray<TSharedPtr<FTetDynamicPoint>> DynamicPointArray = TetrahedronMeshSPtr->DynamicPointArray;
    Matrix3x3<real> IdMat = Matrix3x3<real>::Identity();
    PositionConstraintsOut.SetNum(DynamicPointArray.Num());
	
    ParallelFor(DynamicPointArray.Num(), [&](int Idx)
        {
            PositionConstraintsOut[Idx] = IdMat;
        });

    for (int i = int(DebugPositionConstraintsRange.X); i < int(DebugPositionConstraintsRange.Y); i++)
	{
        PositionConstraintsOut[i].setZero();
	}
	//Manual test case
    // PositionConstraintsOut[2].setZero();
    // PositionConstraintsOut[3].setZero();
    // PositionConstraintsOut[6].setZero();
    // PositionConstraintsOut[7].setZero();

    // utility::debug::AddOnScreen(-1, 3.f, FColor::Cyan, PositionConstraintsOut[2]);
	//other position constraints
    return PositionConstraintsOut;
}

TTuple<VectorX<real>, VectorX<real>> UDynamicSolidComponent::GetCollisionConstraints(float SimulatedTime, TArray<Matrix3x3<real>>& PositionConstraints)
{
    VectorX<real> VelocityConstraint_CollsionOut;
    VectorX<real> OffsetMask_CollisionOut;

    if (TetrahedronMeshSPtr == nullptr) 
        return TTuple<VectorX<real>, VectorX<real>>(VelocityConstraint_CollsionOut, OffsetMask_CollisionOut);
	
    TArray<TSharedPtr<FTetDynamicPoint>> DynamicPointArray
        = TetrahedronMeshSPtr->DynamicPointArray;

    VelocityConstraint_CollsionOut.setZero(DynamicPointArray.Num() * 3);
    OffsetMask_CollisionOut.setZero(DynamicPointArray.Num() * 3);

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

        //convert UE4 space to simulation space(cm -> m)
        TraceStart *= UnitTransferMToCM;
        TraceEnd *= UnitTransferMToCM;

        // TraceStart = GetOwner()->GetActorTransform().InverseTransformPosition(TraceStart);
        // TraceEnd = GetOwner()->GetActorTransform().InverseTransformPosition(TraceEnd);

    	//convert simulation space to UE4 space
        // TraceStart = GetOwner()->GetActorTransform().TransformPosition(TraceStart);
        // TraceEnd = GetOwner()->GetActorTransform().TransformPosition(TraceEnd);

        FHitResult HitResult;

        FCollisionQueryParams CollisionQueryParams;
        CollisionQueryParams.AddIgnoredActor(GetOwner());
        CollisionQueryParams.bTraceComplex = true;

        // DrawDebugPoint(GetWorld(), TraceStart, 10.f, FColor::Red, true);
        // DrawDebugLine(GetWorld(), TraceStart, TraceEnd, FColor::Red, true);

        if (GetWorld()->LineTraceSingleByChannel(HitResult, TraceStart,
            TraceEnd, ECollisionChannel::ECC_Visibility, CollisionQueryParams))
        {
            FVector ExpectedPosition = HitResult.ImpactPoint;
            // DrawDebugLine(GetWorld(), TraceStart, ExpectedPosition, FColor::Red, true, -1, 0, 5);
            Vector3<real> HitNormal = utility::unreal::FVectorToVector3(HitResult.ImpactNormal);
            // real dist = FVector::Dist(ExpectedPosition, TraceStart);
            // ExpectedPosition = TraceStart + (ExpectedPosition - TraceStart).GetSafeNormal() * (dist);
        	// Full constraint to the point
            PositionConstraints[CurPointIndex].setZero();
            OffsetMask_CollisionOut.block<3, 1>(CurPointIndex * 3, 0) = HitNormal;
        	// DrawDebugLine(GetWorld(), TraceStart, ExpectedPosition, FColor::Cyan, true, -1, 0, 10);
        	//cm/s -> m/s
            Vector3<real> ExpectedVelocity = utility::unreal::FVectorToVector3((ExpectedPosition - TraceStart) / SimulatedTime) * UnitTransferCmToM;
            // real NormalVelocity = ExpectedVelocity.dot(HitNormal);
        	// NormalVelocity * 
        	//Expected Delta Velocity
            VelocityConstraint_CollsionOut.block<3, 1>(CurPointIndex * 3, 0) = (ExpectedVelocity - CurRenderablePoint->Velocity);
        }
    }
	
    return TTuple<VectorX<real>, VectorX<real>>(VelocityConstraint_CollsionOut, OffsetMask_CollisionOut);
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

	//update surface normal runtime
    TetrahedronMeshSPtr->ComputeNormals();

    for (int i = 0; i < TetrahedronMeshSPtr->RenderablePointIndexArray.Num(); i++)
    {
		int CurIndex = TetrahedronMeshSPtr->RenderablePointIndexArray[i];
		//RuntimeMeshComponent会自动应用节点的Transform，所以还需要把Transform变回去
		//每轮都多很多次矩阵向量乘法
		FVector Tmp = utility::unreal::Vector3ToFVector(TetrahedronMeshSPtr->DynamicPointArray[CurIndex]->Position);
	    FVector CurNormal = utility::unreal::Vector3ToFVector(TetrahedronMeshSPtr->RenderableNormalArray[CurIndex]);
        // FVector CurNormal = utility::unreal::Vector3ToFVector(TetrahedronMeshSPtr->DynamicPointArray[CurIndex]->ComputeNormal());
    	FVector2D CurUv = utility::unreal::Vector2ToFVector2D(TetrahedronMeshSPtr->RenderableUvArray[CurIndex]);
	    Tmp = Tmp * UnitTransferMToCM;

	    if (bIsVisualNormal)
	        DrawDebugLine(GetWorld(), Tmp, Tmp + CurNormal * VisualNormalLength, FColor::Cyan, false, 0.1f);

	    //思路同CreateRenderableData()
	    Tmp = OwnerActorTransform.InverseTransformPosition(Tmp);
        FRuntimeMeshTangent CurTangent;
        CurTangent.TangentX = utility::unreal::Vector3ToFVector(TetrahedronMeshSPtr->RenderableTangentArray[CurIndex]);
        TangentArray.Add(CurTangent);
        // CurNormal = OwnerActorTransform.TransformPosition(CurNormal);
		PositionArray.Add(Tmp);
		NormalArray.Add(CurNormal);
	    TexCoordArray.Add(CurUv);
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
    for (int i = 0; i < RenderablePointIndexArray.Num(); i++)
    {
        if (!IsRenderablePoint.Contains(RenderablePointIndexArray[i]))
            IsRenderablePoint.Add(RenderablePointIndexArray[i]);
    }
    
    for (int i = 0; i < DynamicPointArray.Num(); i++)
    {
	TSharedPtr<FTetDynamicPoint> CurDynamicPoint = DynamicPointArray[i];
	FVector CurDynamicPos = utility::unreal::Vector3ToFVector(CurDynamicPoint->Position);
    CurDynamicPos = CurDynamicPos * UnitTransferMToCM;
	// CurDynamicPos = GetOwner()->GetActorTransform().InverseTransformPosition(CurDynamicPos);
	if (IsRenderablePoint.Contains(i))
	    DrawDebugPoint(GetWorld(), CurDynamicPos, 5.f, FColor::Cyan, false, 0.1f);
	else
	    DrawDebugPoint(GetWorld(), CurDynamicPos, 5.f, FColor::Red, false, 0.1f);
    }
}

bool UDynamicSolidComponent::EditorStateSync()
{
    AActor* OwnerActor = GetOwner();
    if (OwnerActor == nullptr) return false;
	   
    InitializeRuntimeMeshComp();

    // if (MeshMaterial != nullptr)
    RuntimeMeshProviderStatic->SetupMaterialSlot
        (0, TEXT("TriMat"), MeshMaterial);

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
            FVector VertexPosInWorld =GetComponentTransform().TransformVector(VertexPos);
            // FVector VertexPosInWorld = GetOwner()->GetActorLocation() + GetComponentTransform().TransformVector(VertexPos);
            //position
        	SMPositionArray.Add(VertexPosInWorld);
            //add other information
        	//uv
            SMUvArray.Add(VertexBuffer->StaticMeshVertexBuffer.GetVertexUV(i, 0));
        	//normal
            SMNormalArray.Add(VertexBuffer->StaticMeshVertexBuffer.VertexTangentZ(i));
			//tangent
            FRuntimeMeshTangent CurMeshTangent;
            CurMeshTangent.TangentX = VertexBuffer->StaticMeshVertexBuffer.VertexTangentX(i);
            SMTangentArray.Add(CurMeshTangent);

        }

        for (int i = 0; i < IndexBuffer->GetNumIndices(); i++)
        {
            SMTriangleIndexArray.Add(IndexBuffer->GetArrayView()[i]);
        }
    	
        UE_LOG(LogDynamicSolidCompInit, Display, TEXT("StaticMesh Position Number:%d"), SMPositionArray.Num());
        UE_LOG(LogDynamicSolidCompInit, Display, TEXT("StaticMesh Triangle Number:%d"), SMTriangleIndexArray.Num() / 3);
        UE_LOG(LogTemp, Display, TEXT("StaticMesh Normal Number:%d"), SMNormalArray.Num());
        UE_LOG(LogTemp, Display, TEXT("StaticMesh Tangent Number:%d"), SMTangentArray.Num());
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
            PoissonRatio = FMath::Clamp(PoissonRatio, 0.f, 0.49999f);
    	
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

    if (PropName == FName("MaxEpoch"))
        MaxEpoch = FMath::Clamp(MaxEpoch, 1, MaxEpoch);

    if (PropName == FName("InternalForceDamping"))
        InternalForceDamping = FMath::Clamp(InternalForceDamping, 0.f, 1.f);

    if (PropName == FName("SolverTolerance"))
        SolverTolerance = FMath::Clamp(SolverTolerance,0.f, 0.1f);

    if (PropName == FName("Density"))
        Density = FMath::Clamp(Density, 0.f, Density);
}
#endif