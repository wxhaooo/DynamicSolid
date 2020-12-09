// Fill out your copyright notice in the Description page of Project Settings.


#include "UtilityNumeric.h"

namespace utility
{
	namespace numeric
	{
		UVSGroup RotationVariantSVD(const Matrix3x3<real>& F)
		{
			Eigen::JacobiSVD<Matrix3x3<real>, Eigen::NoQRPreconditioner> svd(F, Eigen::ComputeFullU | Eigen::ComputeFullV);
			Matrix3x3<real> U = svd.matrixU();
			Matrix3x3<real> V = svd.matrixV();
			Vector3<real> S = svd.singularValues();

			if(U.determinant() < real(0.))
			{
				U.col(0) *= real(-1.);
				S(0) *= real(-1.);
			}

			if(V.determinant() < real(0.))
			{
				V.col(0) *= real(-1.);
				S(0) *= real(-1.);
			}

			return UVSGroup(U, V, S);
		}
	}
}