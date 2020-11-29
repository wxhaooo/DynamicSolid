// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"


UENUM()
enum EInternalEnergyModel {
	IEM_NONE              UMETA(DisplayName = "None"),
	IEM_NEOHOOKEAN        UMETA(DisplayName = "Neo-Hookean"),
	IEM_STVK              UMETA(DisplayName = "StVk"),
	IEM_STABLE_NEOHOOKEAN UMETA(DisplayName = "Stable Neo-Hookean")
};


// class DYNAMICSOLID_API InternalEnergyModel
// {
// public:
// 	InternalEnergyModel();
// 	~InternalEnergyModel();
// };
