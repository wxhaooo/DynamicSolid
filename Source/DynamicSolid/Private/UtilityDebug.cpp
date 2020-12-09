// Fill out your copyright notice in the Description page of Project Settings.


#include "UtilityDebug.h"
#include "UtilityUnreal.h"

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
			for (int i = Start; i < End; i++)
			{
				VecStr += (FString::SanitizeFloat(VecX(i)) + " ");
			}
			GEngine->AddOnScreenDebugMessage(Key, TimeToDisplay, DisplayColor, VecStr);
		}

		void AddOnScreen(int Key, float TimeToDisplay, const FColor& DisplayColor, const Matrix<real, 9, 9>& Mat)
		{
			for (int i = 0; i < 9; i++) {
				FString MatStr = FString::Printf(TEXT("%f %f %f %f %f %f %f %f %f"),
					Mat(i, 0), Mat(i, 1), Mat(i, 2),
					Mat(i, 3), Mat(i, 4), Mat(i, 5),
					Mat(i, 6), Mat(i, 7), Mat(i, 8));
				GEngine->AddOnScreenDebugMessage(Key, TimeToDisplay, DisplayColor, MatStr);
			}
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

		FString ConvertLogStr(const Matrix3x3<real>& Mat)
		{
			FString LogMatStrOut;

			LogMatStrOut = FString::Printf(TEXT("\n%f %f %f\n%f %f %f\n%f %f %f\n"),
				Mat(0, 0), Mat(0, 1), Mat(0, 2),
				Mat(1, 0), Mat(1, 1), Mat(1, 2),
				Mat(2, 0), Mat(2, 1), Mat(2, 2));
			
			return LogMatStrOut;
		}

		FString ConvertLogStr(const Matrix<real, 9, 9>& Mat)
		{
			FString LogMatStrOut;
			LogMatStrOut = FString::Printf(TEXT(
			"\n%f %f %f %f %f %f %f %f %f\n \\\
			%f %f %f %f %f %f %f %f %f\n \\\
			%f %f %f %f %f %f %f %f %f\n \\\
			%f %f %f %f %f %f %f %f %f\n \\\
			%f %f %f %f %f %f %f %f %f\n \\\
			%f %f %f %f %f %f %f %f %f\n \\\
			%f %f %f %f %f %f %f %f %f\n \\\
			%f %f %f %f %f %f %f %f %f\n \\\
			%f %f %f %f %f %f %f %f %f\n"),
				Mat(0, 0), Mat(0, 1), Mat(0, 2),Mat(0, 3), Mat(0, 4), Mat(0, 5),Mat(0, 6), Mat(0, 7), Mat(0, 8),
				Mat(1, 0), Mat(1, 1), Mat(1, 2), Mat(1, 3), Mat(1, 4), Mat(1, 5), Mat(1, 6), Mat(1, 7), Mat(1, 8),
				Mat(2, 0), Mat(2, 1), Mat(2, 2), Mat(2, 3), Mat(2, 4), Mat(2, 5), Mat(2, 6), Mat(2, 7), Mat(2, 8),
				Mat(3, 0), Mat(3, 1), Mat(3, 2), Mat(3, 3), Mat(3, 4), Mat(3, 5), Mat(3, 6), Mat(3, 7), Mat(3, 8),
				Mat(4, 0), Mat(4, 1), Mat(4, 2), Mat(4, 3), Mat(4, 4), Mat(4, 5), Mat(4, 6), Mat(4, 7), Mat(4, 8),
				Mat(5, 0), Mat(5, 1), Mat(5, 2), Mat(5, 3), Mat(5, 4), Mat(5, 5), Mat(5, 6), Mat(5, 7), Mat(5, 8),
				Mat(6, 0), Mat(6, 1), Mat(6, 2), Mat(6, 3), Mat(6, 4), Mat(6, 5), Mat(6, 6), Mat(6, 7), Mat(6, 8),
				Mat(7, 0), Mat(7, 1), Mat(7, 2), Mat(7, 3), Mat(7, 4), Mat(7, 5), Mat(7, 6), Mat(7, 7), Mat(7, 8),
				Mat(8, 0), Mat(8, 1), Mat(8, 2), Mat(8, 3), Mat(8, 4), Mat(8, 5), Mat(8, 6), Mat(8, 7), Mat(8, 8));
			
			return LogMatStrOut;
		}

	}
}
