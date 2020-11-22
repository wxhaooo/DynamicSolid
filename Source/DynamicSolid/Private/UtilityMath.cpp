// Fill out your copyright notice in the Description page of Project Settings.


#include "UtilityMath.h"

namespace utility
{
	namespace math
	{
		Matrix<real, 3, 3> CrossMatrix(const Vector3<real>& f)
		{
			Matrix<real, 3, 3> res;
			res.setZero();

			res(0, 1) = -f(2);
			res(0, 2) = f(1);
			res(1, 0) = f(2);
			res(1, 2) = -f(0);
			res(2, 0) = -f(1);
			res(2, 1) = f(0);
			
			return res;
		}
	}
}
