// Fill out your copyright notice in the Description page of Project Settings.


#include "FTetDynamicPoint.h"
#include "FTetRenderableTriangle.h"

FTetDynamicPoint::FTetDynamicPoint()
{
	
}

FTetDynamicPoint::~FTetDynamicPoint()
{
	
}

Vector3<real> FTetDynamicPoint::ComputeNormal()
{
	Vector3<real> NormalOut;
	NormalOut.setZero();

	int n = AdjRenderableTriangleArray.Num();
	for(int i=0;i<n;i++)
	{
		NormalOut += AdjRenderableTriangleArray[i]->Normal;
	}

	NormalOut /= n;
	return NormalOut;
}
