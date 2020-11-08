// Fill out your copyright notice in the Description page of Project Settings.


#include "DynamicSolidActor.h"
#include "DynamicSolidComponent.h"

// Sets default values
ADynamicSolidActor::ADynamicSolidActor()
{
 	// Set this actor to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
	PrimaryActorTick.bCanEverTick = true;

	DynamicSolidComp = CreateDefaultSubobject<UDynamicSolidComponent>("DynamicSolidComp");
	RootComponent = DynamicSolidComp;
}

void ADynamicSolidActor::OnConstruction(const FTransform& Transform)
{
	Super::OnConstruction(Transform);

	TSharedPtr<FTetrahedronMesh> TetrahedronMeshSPtr;
	TetrahedronMeshSPtr = DynamicSolidComp->GetTetrahedronMeshSPtr();
	if (TetrahedronMeshSPtr == nullptr) return;

	DynamicSolidComp->InitializeRuntimeMeshComp();
	DynamicSolidComp->UpdateRenderableData();
}


// Called when the game starts or when spawned
void ADynamicSolidActor::BeginPlay()
{
	Super::BeginPlay();
	
}

// Called every frame
void ADynamicSolidActor::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);

}

