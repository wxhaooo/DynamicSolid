// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "UtilityMath.h"
#include "InternalEnergyModel.h"

class FTetDynamicPoint;
class FTetDynamicEdge;

class FDynamicTetrahedron
{
public:
	FDynamicTetrahedron();
	~FDynamicTetrahedron();

	FDynamicTetrahedron(TSharedPtr<FTetDynamicPoint> p0,
		TSharedPtr<FTetDynamicPoint> p1, TSharedPtr<FTetDynamicPoint> p2, 
		TSharedPtr<FTetDynamicPoint> p3,int index)
	{
		this->p0 = p0; this->p1 = p1; this->p2 = p2; this->p3 = p3;
		this->index = index;

		ComputeDmInvAndVolume();
	}

	Matrix<real, 3, 3> DmInv;

	void ComputeDmInvAndVolume();

	real ComputeMass(real Density) { return RestVolume * Density; }

	real GetVolume() { return RestVolume; }

	TTuple<int, int, int, int> PointIndices();

	//get internal force
	Vector12<real> f(real Mu, real Lambda, EInternalEnergyModel EnergyModel);
	Vector12<real> ComputeInternalForceAsSifakisWay(real Mu, real Lambda);
	Vector12<real> ComputeForceAsTensorWay(real Mu, real Lambda,EInternalEnergyModel EnergyModel = EInternalEnergyModel::IEM_NONE);

	//stiffness matrix
	Matrix<real, 12, 12> K(real Mu, real Lambda, EInternalEnergyModel EnergyModel);
	Matrix<real, 12, 12> ComputePPhiPx2(real Mu, real Lambda, EInternalEnergyModel EnergyModel);

	Matrix<real, 3, 3> ComputeStress(real Mu, real Lambda);
	Vector9<real> ComputePPhiPF(real Mu, real Lambda, EInternalEnergyModel EnergyModel);
	Matrix<real, 9, 9> ComputePPhiPF2(real Mu, real Lambda, EInternalEnergyModel EnergyModel);
	Matrix<real, 9, 9> ComputePPhiPF2StvK(real Mu, real Lambda);
	Matrix<real, 9, 12> ComputePFPx();
	Matrix<real, 3, 3> GetF();
	Matrix<real, 3, 3> GetDs();
	//points in tetrahedron element
	TSharedPtr<FTetDynamicPoint> p0, p1, p2, p3;
	int index;
	real RestVolume;
};
