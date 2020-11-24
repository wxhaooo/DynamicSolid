// Fill out your copyright notice in the Description page of Project Settings.


#include "UtilitySolver.h"
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

			ParallelFor(SArray.Num(), [&](int64 Idx)
				{
					FilterOut.block<3, 1>(Idx * 3, 0) =
						SArray[Idx] * V.block<3, 1>(Idx * 3, 0);
				});
			
			return FilterOut;
		}
		
		VectorX<real> MPCG(const SpMat<real>& A, const VectorX<real>& b,
			const VectorX<real>& z, const TArray<Matrix3x3<real>>& SArray,
			const real Epsilon,const int MaxEpoch)
		{
			UE_LOG(LogSolver, Display, TEXT("-------------End MPCG Solver-------------"));
			
			int Epoch = 0;
			//Output: \DeltaV
			VectorX<real> DeltaV = z;
			//Precondition Matrix for MPCG Solver
			SpMat<real> P(A.rows(), A.cols());
			SpMat<real> PInv(A.rows(), A.cols());
			ParallelFor(A.rows(), [&](int64 Idx)
				{
					real Aii = A.coeff(Idx, Idx);
					if (!FMath::IsNearlyEqual(Aii, 0)) {
						P.insert(Idx, Idx) = Aii;
						PInv.insert(Idx, Idx) = 1.f / A.coeff(Idx, Idx);
					}
				});

			P.makeCompressed();
			PInv.makeCompressed();
			
			VectorX<real> Filterb = MPCGFilter(b, SArray);
			real Delta0 = Filterb.transpose() * P * Filterb;
			VectorX<real> r = MPCGFilter(b - A * DeltaV, SArray);
			VectorX<real> c = MPCGFilter(PInv * r,SArray);
			real DeltaNew = r.transpose().dot(c);
			real DeltaOld = DeltaNew;
			real Epsilon2 = Epsilon * Epsilon;
			
			while (DeltaNew > Epsilon2 * Delta0 && Epoch < MaxEpoch)
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
				UE_LOG(LogSolver, Display, TEXT("Epoch:%d,Epsilon:%f,Alpha:{}"),
					Epoch, Epsilon, Alpha);
			}

			UE_LOG(LogSolver, Display, TEXT("-------------End MPCG Solver-------------"));
			
			return DeltaV;
		}
	}
}
