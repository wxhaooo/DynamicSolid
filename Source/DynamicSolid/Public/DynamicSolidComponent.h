// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "Components/SceneComponent.h"
// #include "FTetrahedronMesh.h"
#include "InternalEnergyModel.h"
#include "RuntimeMeshComponent.h"
#include "UtilityMath.h"
#include "MagicNumber.h"
#include "DynamicSolidComponent.generated.h"

DECLARE_LOG_CATEGORY_EXTERN(LogDynamicSolidCompInit, Log, All);

class URuntimeMeshComponent;
class URuntimeMeshProviderStatic;
class FTetrahedronMesh;

UENUM()
enum EIntegrationMethod {
    IM_NONE             UMETA(DisplayName = "None"),
    IM_EXP_FD           UMETA(DisplayName = "Explicit Method: FowardDifference"),
    IM_EXP_VERLET       UMETA(DisplayName = "Explicit Method: Verlet"),
    IM_IMP_MPCG         UMETA(DisplayName = "Implicit Method: MPCG"),
    IM_IMP_PPCG         UMETA(DisplayName = "Implicit Method: PPCG"),
};

UCLASS( ClassGroup=(Custom), meta=(BlueprintSpawnableComponent) )
class DYNAMICSOLID_API UDynamicSolidComponent : public USceneComponent
{
    GENERATED_BODY()

public:	
    // Sets default values for this component's properties
    UDynamicSolidComponent();

protected:
    // Called when the game starts
    virtual void BeginPlay() override;

protected:
    UPROPERTY(VisibleAnywhere, Category = "Components")
	URuntimeMeshComponent* RuntimeMeshComp;

    UPROPERTY(EditDefaultsOnly, BlueprintReadWrite, Category = "Dynamic Solid Configurations")
        FString InitDynamicSolidPath;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Dynamic Solid Configurations")
        UStaticMesh* InitDynamicSolidMesh;

    //Simulator Configuration
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Simulator Configurations")
	float TimeStep;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Simulator Configurations")
        float MaxTimeStep;

	//m/s^{2}
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Simulator Configurations")
        FVector GravityAccleration;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Simulator Configurations")
        float YoungModulus;

	//Poisson rate should not equal to 0.5f
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Simulator Configurations")
        float PoissonRatio;

    //Lame's first parameter
    UPROPERTY(VisibleDefaultsOnly, BlueprintReadWrite, Category = "Simulator Configurations")
        float Lambda;
	
	//Lame's second parameter
    UPROPERTY(VisibleDefaultsOnly, BlueprintReadWrite, Category = "Simulator Configurations")
        float Mu;

	//kg/m^3
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Simulator Configurations")
        float Density;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Simulator Configurations")
        float InternalForceDamping;
	
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Simulator Configurations")
	bool bFixedTimeStep;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Simulator Configurations")
        bool bVisualDebugger;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Simulator Configurations")
        float SolverTolerance;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Simulator Configurations")
        float CollisionOffset;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Simulator Configurations")
        int MaxEpoch;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Simulator Configurations")
        bool bForcePD;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Simulator Configurations")
        TEnumAsByte<EIntegrationMethod> IntegrationMethod;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Simulator Configurations")
        TEnumAsByte<EInternalEnergyModel> InternalEnergyModel;

    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Simulator Configurations")
        FVector2D DebugPositionConstraintsRange;

    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "MeshMaterial")
	UMaterialInterface* MeshMaterial;

    UPROPERTY()
        URuntimeMeshProviderStatic* RuntimeMeshProviderStatic;
	
public:
    // Called every frame
    virtual void TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction) override;

    virtual void PostEditChangeProperty(FPropertyChangedEvent& PropertyChangedEvent) override;
	
    TSharedPtr<FTetrahedronMesh> GetTetrahedronMeshSPtr();

    void SetTetrahedronMeshSPtr(TSharedPtr<FTetrahedronMesh> TetMeshPtr);

    URuntimeMeshComponent* GetRuntimeMeshComp();

    bool EditorStateSync();

    bool InitializeRuntimeMeshComp();

    bool CreateRenderableData();

    bool UpdateRenderableData();

    FString GetInitDynamicSolidPath();

    bool Initialize();

	//wait to code
    VectorX<real> ComputeWindForce(float SimulatedTime);

    bool ComputeWindForce(float SimulatedTime, SpMat<real>& A, VectorX<real>& b);

    VectorX<real> ComputeGravity(float SimulatedTime);

    bool ComputeGravity(float SimulatedTime, SpMat<real>& A, VectorX<real>& b);
	
    VectorX<real> ComputeInternalForce(float SimulatedTime);

    SpMat<real> ComputeInternalForce(float SimulatedTime, SpMat<real>& A, VectorX<real>& b);

    VectorX<real> ComputeConstraintForce(float SimulatedTime);

    bool ComputeMorphForce(float SimulatedTime, SpMat<real>& A, VectorX<real>& b);

    bool ComputeOffsetCorrection(float SimulatedTime, SpMat<real>& K, SpMat<real>& A, VectorX<real>& b, const VectorX<real>& OffsetMask);

    void CollisionResolve();

    void VisualMotionDebugger();

    float ComputeMu();

    float ComputeLambda();

private:

    bool EXP_ForwardDifference(float SimulatedTime);

    bool EXP_Verlet(float SimulatedTime);

    bool IMP_MPCG(float SimulatedTime);

    bool IMP_PPCG(float SimulatedTime);

    bool AdvanceStep(float SimulatedTime, const VectorX<real>& DeltaV);
	
    TSharedPtr<FTetrahedronMesh> TetrahedronMeshSPtr;

    TTuple<VectorX<real>,VectorX<real>> GetCollisionConstraints(float SimulatedTime, TArray<Matrix3x3<real>>& PositionConstraints);

    TTuple<SpMat<real>, VectorX<real>, SpMat<real>> GetImplicitEquation(real SimulatedTime);

    TArray<Matrix3x3<real>> GetPositionConstraints();

    TSharedPtr<FTetrahedronMesh> ConvertTriangleToTetrahedronMesh();

    bool LoadInitDynamicSolidMesh();

    UPROPERTY()
        TArray<FVector> SMPositionArray;
    UPROPERTY()
        TArray<FVector2D> SMUvArray;
    UPROPERTY()
        TArray<FVector> SMNormalArray;
    UPROPERTY()
        TArray<FColor> SMColorArray;
    UPROPERTY()
        TArray<int> SMTriangleIndexArray;
    UPROPERTY()
        TArray<FRuntimeMeshTangent> SMTangentArray;

    float TotalSimulatorTime;
};
