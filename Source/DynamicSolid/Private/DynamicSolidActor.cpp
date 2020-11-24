// Fill out your copyright notice in the Description page of Project Settings.


#include "DynamicSolidActor.h"
#include "DynamicSolidComponent.h"
#include "Providers/RuntimeMeshProviderBox.h"
#include "RuntimeMeshComponent.h"
#include "Providers/RuntimeMeshProviderStatic.h"
#include "Kismet/KismetMathLibrary.h"
#include "Misc/DateTime.h"

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
}

void ADynamicSolidActor::PostInitializeComponents()
{
    Super::PostInitializeComponents();
} 

// Called when the game starts or when spawned
void ADynamicSolidActor::BeginPlay()
{
    Super::BeginPlay();

    DynamicSolidComp->Initialize();
}

// Called every frame
void ADynamicSolidActor::Tick(float DeltaTime)
{
    Super::Tick(DeltaTime);

    // GEngine->AddOnScreenDebugMessage(-1, 3.f, FColor::Red, FString::SanitizeFloat(TestValue));
    // if (TestStrPtr == nullptr)
    // 	GEngine->AddOnScreenDebugMessage(-1, 3.f, FColor::Cyan, "TestStrPtr is Empty");
    // else
    // 	GEngine->AddOnScreenDebugMessage(-1, 3.f, FColor::Cyan, *TestStrPtr);
    // if (TestStrSPtr == nullptr)
    //     GEngine->AddOnScreenDebugMessage(-1, 3.f, FColor::Black, "TestStrSPtr is Empty");
    // else
    //     GEngine->AddOnScreenDebugMessage(-1, 3.f, FColor::Black, *TestStrSPtr);

}

