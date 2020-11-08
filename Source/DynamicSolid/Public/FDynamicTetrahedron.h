// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

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
	}

	//points in tetrahedron element
	TSharedPtr<FTetDynamicPoint> p0, p1, p2, p3;
	int index;
};
