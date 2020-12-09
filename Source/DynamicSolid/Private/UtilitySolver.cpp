// Fill out your copyright notice in the Description page of Project Settings.


#include "UtilitySolver.h"
#include "UtilityDebug.h"
#include "Async/ParallelFor.h"

DEFINE_LOG_CATEGORY(LogSolver);

namespace utility
{
    namespace solver
    {
	VectorX<real> MPCGFilter(const VectorX<real>& V,
		const TArray<Matrix3x3<real>>& SArray)
	{
		VectorX<real> FilterOut;
		FilterOut.setZero(V.size());

		for (int Idx = 0; Idx < SArray.Num(); Idx++)
		{
			FilterOut.block<3, 1>(Idx * 3, 0) =
				SArray[Idx] * V.block<3, 1>(Idx * 3, 0);
		}
		
		// ParallelFor(SArray.Num(), [&](int64 Idx)
		// 	{
		// 		FilterOut.block<3, 1>(Idx * 3, 0) =
		// 			SArray[Idx] * V.block<3, 1>(Idx * 3, 0);
		// 	});
		
		return FilterOut;
	}
	
	VectorX<real> MPCG(const SpMat<real>& A, const VectorX<real>& b,
		const VectorX<real>& z, const TArray<Matrix3x3<real>>& SArray,
		const real Epsilon,const int MaxEpoch)
	{
		UE_LOG(LogSolver, Display, TEXT("-------------Start MPCG Solver-------------"));
		
		int Epoch = 0;
		//Output: \DeltaV
		VectorX<real> DeltaV = z;
		// utility::debug::AddOnScreen(-1, 3.f, FColor::Red, DeltaV, 0, DeltaV.size());
		//Precondition Matrix for MPCG Solver
		SpMat<real> P(A.rows(), A.cols());
		SpMat<real> PInv(A.rows(), A.cols());
		P.setZero(); PInv.setZero();

		for (int Idx = 0; Idx < A.rows(); Idx++)
		{
			real Aii = A.coeff(Idx, Idx);
			P.coeffRef(Idx, Idx) = Aii;
			PInv.coeffRef(Idx, Idx) = 1.f / Aii;
		}

		//对于稀疏矩阵先不要使用并行算法，会crash，原因未知
		// ParallelFor(A.rows(), [&](int Idx)
		// 	{
		// 		real Aii = A.coeff(Idx, Idx);
		// 		P.coeffRef(Idx, Idx) = Aii;
		// 		PInv.coeffRef(Idx, Idx) = 1.f / Aii;
		// 	});

		P.makeCompressed();
		PInv.makeCompressed();

		VectorX<real> Filterb = MPCGFilter(b, SArray);
		real Delta0 = Filterb.transpose() * (P * Filterb);
		
		VectorX<real> r = MPCGFilter(b - A * DeltaV, SArray);
		VectorX<real> c = MPCGFilter(PInv * r,SArray);
		real DeltaNew = r.transpose().dot(c);
		real DeltaOld = DeltaNew;
		real Epsilon2 = Epsilon * Epsilon;
		
		// GEngine->AddOnScreenDebugMessage(-1, 3.f, FColor::Red, FString::SanitizeFloat(DeltaNew));
		// GEngine->AddOnScreenDebugMessage(-1, 3.f, FColor::Red, FString::SanitizeFloat(Epsilon2 * Delta0));
		UE_LOG(LogSolver, Display, TEXT("Epoch:%d,DeltaNew:%f,Eps:%f"), Epoch,
			DeltaNew, Epsilon2 * Delta0);
		while (DeltaNew > Epsilon2 * Delta0)
		{
			VectorX<real> q = MPCGFilter(A * c, SArray);
			//if Alpha == NaN, The solver diverges
			real Alpha = DeltaNew / (c.transpose() *q);
			DeltaV = DeltaV + Alpha * c;
			r = r - Alpha * q;
			VectorX<real> s = PInv * r;
			DeltaOld = DeltaNew;
			DeltaNew = r.transpose() * s;
			c = MPCGFilter(s + (DeltaNew / DeltaOld) * c, SArray);
			Epoch++;
			if (Epoch > MaxEpoch) break;
			UE_LOG(LogSolver, Display, TEXT("Epoch:%d,DeltaNew:%f,Alpha:%f,Eps:%f"),
				Epoch, DeltaNew, Alpha, Epsilon2);
		}

		UE_LOG(LogSolver, Display, TEXT("-------------End MPCG Solver-------------"));
		
		// Vector3<real> tmp = DeltaV.block<3, 1>(0 * 3, 0);
		// GEngine->AddOnScreenDebugMessage(-1, 3.f, FColor::Red, FString::SanitizeFloat(Epsilon));
		// GEngine->AddOnScreenDebugMessage(-1, 3.f, FColor::Red, FString::SanitizeFloat(Epsilon2 * Delta0));
		// GEngine->AddOnScreenDebugMessage(-1, 3.f, FColor::Red, FString::SanitizeFloat(DeltaV.norm()));
		// utility::debug::AddOnScreen(-1, 3.f, FColor::Red, tmp);
		return DeltaV;
	}
    }
}
