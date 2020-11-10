// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "Components/SceneComponent.h"
#include "FTetrahedronMesh.h"
#include "DynamicSolidComponent.generated.h"

class URuntimeMeshComponent;
class URuntimeMeshProviderStatic;

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
		FVector Gravity;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Simulator Configurations")
		bool bFixedTimeStep;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "MeshMaterial")
		UMaterialInterface* MeshMaterial;

	URuntimeMeshProviderStatic* RuntimeMeshProviderStatic;
	
public:	
	// Called every frame
	virtual void TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction) override;

	// virtual void PostEditChangeProperty(FPropertyChangedEvent& PropertyChangedEvent) override;

	TSharedPtr<FTetrahedronMesh> GetTetrahedronMeshSPtr();

	void SetTetrahedronMeshSPtr(TSharedPtr<FTetrahedronMesh> TetMeshPtr);

	URuntimeMeshComponent* GetRuntimeMeshComp();

	bool InitializeRuntimeMeshComp();

	bool CreateRenderableData();

	bool UpdateRenderableData();

	FString GetInitDynamicSolidPath();

	bool Initialize();

	bool ApplyGravity(float DeltaTime);
private:
	//UPROPERTY()
	TSharedPtr<FTetrahedronMesh> TetrahedronMeshSPtr;
	//FTetrahedronMesh* TetrahedronMeshSPtr;
};
