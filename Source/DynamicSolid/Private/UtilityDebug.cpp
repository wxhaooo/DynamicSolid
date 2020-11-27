// Fill out your copyright notice in the Description page of Project Settings.


#include "UtilityDebug.h"

namespace utility
{
	namespace debug
	{
		void AddOnScreen(int Key,float TimeToDisplay,const FColor& DisplayColor, const Matrix3x3<real>& Mat)
		{
			FString MatStr = FString::Printf(TEXT("%f %f %f || %f %f %f || %f %f %f"), 
				Mat(0, 0), Mat(0, 1), Mat(0, 2),
				Mat(1, 0), Mat(1, 1), Mat(1, 2),
				Mat(2, 0), Mat(2, 1), Mat(2, 2));
			GEngine->AddOnScreenDebugMessage(Key, TimeToDisplay, DisplayColor, MatStr);
		}

		void AddOnScreen(int Key, float TimeToDisplay, const FColor& DisplayColor, const Vector3<real>& Vec3)
		{
			FString VecStr = FString::Printf(TEXT("%f %f %f"),
				Vec3(0),Vec3(1),Vec3(2));
			GEngine->AddOnScreenDebugMessage(Key, TimeToDisplay, DisplayColor, VecStr);
		}

		void AddOnScreen(int Key, float TimeToDisplay, const FColor& DisplayColor, const VectorX<real>& VecX, int Start, int End)
		{
			int n = VecX.size();
			if (Start <0 || End>n || Start > End)
			{
				GEngine->AddOnScreenDebugMessage(Key, TimeToDisplay, DisplayColor, TEXT("Dimension of VecX is not in [Start,End]"));
				return;
			}

			// FText VecTxt;
			FString VecStr;
			for (int i = Start; i <= End; i++)
			{
				VecStr += (FString::SanitizeFloat(VecX(i)) + " ");
			}
			GEngine->AddOnScreenDebugMessage(Key, TimeToDisplay, DisplayColor, VecStr);
		}

		FString ConvertLogStr(const VectorX<real>& VecX, int Start, int End)
		{
			FString LogVecStrOut;
			int n = VecX.size();
			if (Start <0 || End>n || Start > End)
			{
				UE_LOG(LogTemp, Display, TEXT("Dimension of VecX is not in [Start,End]"));
				return LogVecStrOut;
			}

			for (int i = Start; i <= End; i++)
			{
				LogVecStrOut += (FString::SanitizeFloat(VecX(i)) + " ");
			}
			
			return LogVecStrOut;
		}
	}
}
