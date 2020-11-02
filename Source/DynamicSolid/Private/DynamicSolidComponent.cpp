// Fill out your copyright notice in the Description page of Project Settings.


#include "DynamicSolidComponent.h"
#include "RuntimeMeshComponent.h"

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

void UDynamicSolidComponent::PostEditChangeProperty(FPropertyChangedEvent& PropertyChangedEvent)
{
	FName PropName = PropertyChangedEvent.Property->GetFName();

	if (PropName != FName("InitialSolidPath")) return;

	//to resolve tet solid

	
	
	// UE_LOG(LogTemp, Display, TEXT("PropName: %s"), *PropName.ToString());
}

