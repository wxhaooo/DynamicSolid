// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "Components/SceneComponent.h"
// #include "FTetrahedronMesh.h"
#include "UtilityMath.h"
#include "DynamicSolidComponent.generated.h"

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

    //Simulator Configuration
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Simulator Configurations")
		float TimeStep;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Simulator Configurations")
		float MaxTimeStep;

    UPROPERTY(EditAnywhere,BlueprintReadWrite,Category="Simulator Configurations")
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

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Simulator Configurations")
        float Density;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Simulator Configurations")
        float InternalForceDamping;
	
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Simulator Configurations")
		bool bFixedTimeStep;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Simulator Configurations")
        bool bVisualDebugger;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Simulator Configurations")
        TEnumAsByte<EIntegrationMethod> IntegrationMethod;

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

    bool InitializeRuntimeMeshComp();

    bool CreateRenderableData();

    bool UpdateRenderableData();

    FString GetInitDynamicSolidPath();

    bool Initialize();

    VectorX<real> ComputeGravity(float SimulatedTime);

    VectorX<real> ComputeInternalForce(float SimulatedTime);

    VectorX<real> ComputeConstraintForce(float SimulatedTime);

    void CollisionResolve();

    void VisualMotionDebugger();

    float ComputeMu();

    float ComputeLambda();

private:

    bool EXP_ForwardDifference(float SimulatedTime);

    bool EXP_Verlet(float SimulatedTime);

    bool IMP_MPCG(float SimulatedTime);

    bool IMP_PPCG(float SimulatedTime);
	
    TSharedPtr<FTetrahedronMesh> TetrahedronMeshSPtr;

	
	
};
