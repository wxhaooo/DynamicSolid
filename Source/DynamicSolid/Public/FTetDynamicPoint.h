// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "UtilityMath.h"

class FTetDynamicEdge;
class FTetDynamicFace;

class DYNAMICSOLID_API FTetDynamicPoint
{
public:
	FTetDynamicPoint();
	~FTetDynamicPoint();

	FTetDynamicPoint(const Vector3<real>& Position, int PointIndex,const Vector3<real>& RestPosition,
		const Vector3<real>& InitialPosition, const Vector3<real>& Velocity,
	const Vector3<real>& InitialVelocity):
	Position(Position),RestPosition(RestPosition),InitialPosition(InitialPosition),
	Velocity(Velocity),InitialVelocity(InitialVelocity),PointIndex(PointIndex)
	{}

	FTetDynamicPoint(const Vector3<real>& Position,int PointIndex)
	{
		this->InitialPosition = this->RestPosition = this->Position = Position;
		this->Velocity.setZero();
		this->InitialVelocity = this->Velocity;
		Accleration.setZero();

		this->PointIndex = PointIndex;

		Mass = 1.f;
		MassInv = 1.f;
	}

	Vector3<real> Position;
	Vector3<real> RestPosition;
	Vector3<real> PostPosition;
	Vector3<real> InitialPosition;

	Vector3<real> Velocity;
	Vector3<real> PostVelocity;
	Vector3<real> InitialVelocity;

	Vector3<real> Accleration;

	int PointIndex;

	real Mass;
	real MassInv;

	void SetMass(const real& MassIn)
	{
		this->Mass = MassIn;
		if (FMath::IsNearlyZero(this->Mass))
			this->MassInv = 0.f;
	}

	void AddMass(const real& AddMass)
	{
		this->Mass += AddMass;
	}
	// TArray<FTetDynamicEdge*> AdjacentEdges;
	// TArray<FTetDynamicFace*> AdjacentFaces;
};
