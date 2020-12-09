// Fill out your copyright notice in the Description page of Project Settings.


#include "FTetRenderableTriangle.h"

FTetRenderableTriangle::FTetRenderableTriangle()
{
	
}

FTetRenderableTriangle::~FTetRenderableTriangle()
{
	
}

void FTetRenderableTriangle::UpdateNormal()
{
	if (P0.IsValid() && P1.IsValid() && P2.IsValid())
		//for unreal, we inverse normal
		Normal = utility::geometry::TriangleFaceNormal(P0->Position, P1->Position, P2->Position);
	else
		UE_LOG(LogTemp, Display, TEXT("Renderable Traingle have invalid vertex"));
}
