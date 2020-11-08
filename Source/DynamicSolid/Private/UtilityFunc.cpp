// Fill out your copyright notice in the Description page of Project Settings.


#include "UtilityFunc.h"

namespace utility
{
	namespace func
	{
		bool IsValidLine(const FString& Line)
		{
			return Line.Len() > 0 && Line[0] != '#' && Line[0] != '\n';
		}
	}
}
