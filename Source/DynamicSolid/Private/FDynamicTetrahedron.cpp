// Fill out your copyright notice in the Description page of Project Settings.


#include "FDynamicTetrahedron.h"
#include "FTetDynamicPoint.h"
#include "UtilityDebug.h"
#include "UtilityNumeric.h"

FDynamicTetrahedron::FDynamicTetrahedron()
{
	
}

FDynamicTetrahedron::~FDynamicTetrahedron()
{
	
}

void FDynamicTetrahedron::ComputeDmInvAndVolume()
{
	Matrix<real, 3, 3> tmp;
	tmp.col(0) = p1->RestPosition - p0->RestPosition;
	tmp.col(1) = p2->RestPosition - p0->RestPosition;
	tmp.col(2) = p3->RestPosition - p0->RestPosition;

	if (FMath::Abs(tmp.determinant()) < utility::math::eps) {
		UE_LOG(LogTemp, Display,
			TEXT("det(InvDm) is zero in tetraheron composing of point (%d %d %d %d)"),
			p0->PointIndex, p1->PointIndex, p2->PointIndex, p3->PointIndex);
		DmInv = Matrix<real, 3, 3>::Zero();
		RestVolume = 0.f;
	}
	else {
		RestVolume = 0.16666666666f * tmp.determinant();
		DmInv = tmp.inverse();
	}
}

Matrix<real, 3, 3> FDynamicTetrahedron::GetDs()
{
	Matrix<real, 3, 3> Ds;
	
	Ds.col(0) = p1->Position - p0->Position;
	Ds.col(1) = p2->Position - p0->Position;
	Ds.col(2) = p3->Position - p0->Position;

	return Ds;
}

Vector12<real> FDynamicTetrahedron::ComputeInternalForceAsSifakisWay(real Mu, real Lambda)
{
	Vector12<real> Force;
	Force.setZero();

	Matrix<real, 3, 3> P = ComputeStress(Mu, Lambda);
	real W = 0.16666666f * DmInv.inverse().determinant();
	Matrix<real, 3, 3> H = -W * P * DmInv.transpose();

	Force.block<3, 1>(3, 0) = H.col(0);
	Force.block<3, 1>(6, 0) = H.col(1);
	Force.block<3, 1>(9, 0) = H.col(2);
	Force.block<3, 1>(0, 0) = -H.col(0) - H.col(1) - H.col(2);

	return Force;
}

Vector12<real> FDynamicTetrahedron::ComputeForceAsTensorWay(real Mu, real Lambda, EInternalEnergyModel EnergyModel)
{
    Vector12<real> Force;
    Force.setZero();

    if (EnergyModel == EInternalEnergyModel::IEM_NONE) return Force;

    Matrix<real, 12, 9> PFPxT = ComputePFPx().transpose();
	Force = -RestVolume * PFPxT * ComputePPhiPF(Mu, Lambda, EnergyModel);
    
    return Force;
}

Vector12<real> FDynamicTetrahedron::f(real Mu, real Lambda, EInternalEnergyModel EnergyModel)
{
	return ComputeForceAsTensorWay(Mu, Lambda, EnergyModel);
}

Matrix<real, 9, 9> FDynamicTetrahedron::ComputePPhiPF2(real Mu,real Lambda, EInternalEnergyModel EnergyModel, bool bForcePD)
{
	Matrix<real, 9, 9> PPhiPF2;
	PPhiPF2.setZero();

	Vector9<real> g3;
	g3.setZero();

	Matrix<real, 3, 3> F = GetF();
	Vector3<real> f0 = F.col(0);
	Vector3<real> f1 = F.col(1);
	Vector3<real> f2 = F.col(2);

	real I3 = F.determinant();

	Matrix<real, 9, 9> I;
	I.setIdentity();

	Matrix3x3<real> f0hat = utility::math::CrossMatrix(f0);
	Matrix3x3<real> f1hat = utility::math::CrossMatrix(f1);
	Matrix3x3<real> f2hat = utility::math::CrossMatrix(f2);
	
	Matrix<real, 9, 9> H3;
	H3.setZero();

	H3.block<3, 3>(0, 3) = -f2hat;
	H3.block<3, 3>(3, 0) = f2hat;

	H3.block<3, 3>(0, 6) = f1hat;
	H3.block<3, 3>(6, 0) = -f1hat;
	
	H3.block<3, 3>(3, 6) = -f0hat;
	H3.block<3, 3>(6, 3) = f0hat;

	// UE_LOG(LogTemp, Display, TEXT("%s"), *utility::debug::ConvertLogStr(f0hat));
	// UE_LOG(LogTemp, Display, TEXT("%s"), *utility::debug::ConvertLogStr(f1hat));
	// UE_LOG(LogTemp, Display, TEXT("%s"), *utility::debug::ConvertLogStr(f2hat));
	// UE_LOG(LogTemp, Display, TEXT("%s"), *utility::debug::ConvertLogStr(H3));

	// utility::debug::AddOnScreen(-1, 13.f, FColor::Red, f0hat);
	//
	// utility::debug::AddOnScreen(-1, 13.f, FColor::Red, f1hat);
	//
	// utility::debug::AddOnScreen(-1, 13.f, FColor::Red, f2hat);
	//
	// utility::debug::AddOnScreen(-1, 13.f, FColor::Cyan, H3);

	Matrix3x3<real> PJPF = I3 * F.inverse().transpose();
	g3 = Map<Vector9<real>>(PJPF.data(), PJPF.rows() * PJPF.cols());

	// utility::debug::AddOnScreen(-1, 13.f, FColor::Red, PJPF);
	//
	// utility::debug::AddOnScreen(-1, 13.f, FColor::Red, g3,0,9);

	//the equation in the below is equal to one in the above.
	// g3.block<3, 1>(0, 0) = f1.cross(f2);
	// g3.block<3, 1>(3, 0) = f2.cross(f0);
	// g3.block<3, 1>(6, 0) = f0.cross(f1);

	if (EnergyModel == EInternalEnergyModel::IEM_STABLE_NEOHOOKEAN)
	{
		if (bForcePD)
			PPhiPF2 = ProjectHessianWithAnalyticalFormulasNew(Mu, Lambda, F);
		else
			PPhiPF2 = Mu * I +
				Lambda * g3 * g3.transpose() + (Lambda * (I3 - 1.f) - Mu) * H3;
	}
	else if(EnergyModel == EInternalEnergyModel::IEM_NEOHOOKEAN)
	{
		PPhiPF2 = Mu * I +
			(Mu + Lambda * (1.f - FMath::Loge(I3))) / (FMath::Pow(I3, 2)) * g3 * g3.transpose()
			+ ((Lambda * FMath::Loge(I3) - Mu) / I3) * H3;
	}
	else if(EnergyModel == EInternalEnergyModel::IEM_STVK)
	{
		Vector9<real> g1 = Map<Vector9<real>>(F.data(), F.rows() * F.cols());
		Matrix<real, 9, 9> D;
		D.setZero();

		D.block<3, 3>(0, 0) = f0 * f0.transpose();
		D.block<3, 3>(0, 3) = f1 * f0.transpose();
		D.block<3, 3>(0, 6) = f2 * f0.transpose();

		D.block<3, 3>(3, 0) = f0 * f1.transpose();
		D.block<3, 3>(3, 3) = f1 * f1.transpose();
		D.block<3, 3>(3, 6) = f2 * f1.transpose();

		D.block<3, 3>(6, 0) = f0 * f2.transpose();
		D.block<3, 3>(6, 3) = f1 * f2.transpose();
		D.block<3, 3>(6, 6) = f2 * f2.transpose();

		Matrix<real, 3, 3> FFT = F * F.transpose(), FTF = F.transpose() * F;
		Matrix<real, 9, 9> HII0, HII1;
		HII0.setZero();
		HII1.setZero();
		for (int i = 0; i < 3; i++)
		{
			HII0.block<3, 3>(i * 3, i * 3) = FFT;
		}
		for (int i = 0; i < 3; i++)
		{
			for (int j = 0; j < 3; j++)
			{
				HII1.block<3, 3>(i * 3, j * 3) = 
					Matrix3x3<real>::Identity() * FTF(i, j);
			}
		}
		real IC = FTF.trace();
		Matrix<real, 9, 9> HII = 4 * (HII0 + HII1 + D);
		PPhiPF2 = Lambda / 4.f * g1 * g1.transpose() + 
			(Lambda / 2.f * (IC - 1.f) - Mu) * I + Mu / 4.0f * HII;
		// PPhiPF2 = Lambda / 4.f * g1 * g1.transpose();
		// PPhiPF2 = Mu / 4.0f * HII;
	}

	return PPhiPF2;
}

Matrix<real, 9, 9> FDynamicTetrahedron::BuildTwistAndFlipEigenvectors(const Matrix3x3<real>& U, const Matrix3x3<real>& V)
{
	Matrix<real, 9, 9> Q;
	
	const real scale = 1.0 / std::sqrt(2.0);
	const Matrix3x3<real> sV = scale * V;

	// using M3 = Eigen::Matrix<real, 3, 3, Eigen::ColMajor>;
	
	Matrix3x3<real> A, B, C, D, E, F;
	A << sV(0, 2) * U(0, 1), sV(1, 2)* U(0, 1), sV(2, 2)* U(0, 1),
		sV(0, 2)* U(1, 1), sV(1, 2)* U(1, 1), sV(2, 2)* U(1, 1),
		sV(0, 2)* U(2, 1), sV(1, 2)* U(2, 1), sV(2, 2)* U(2, 1);

	B << sV(0, 1) * U(0, 2), sV(1, 1)* U(0, 2), sV(2, 1)* U(0, 2),
		sV(0, 1)* U(1, 2), sV(1, 1)* U(1, 2), sV(2, 1)* U(1, 2),
		sV(0, 1)* U(2, 2), sV(1, 1)* U(2, 2), sV(2, 1)* U(2, 2);

	C << sV(0, 2) * U(0, 0), sV(1, 2)* U(0, 0), sV(2, 2)* U(0, 0),
		sV(0, 2)* U(1, 0), sV(1, 2)* U(1, 0), sV(2, 2)* U(1, 0),
		sV(0, 2)* U(2, 0), sV(1, 2)* U(2, 0), sV(2, 2)* U(2, 0);

	D << sV(0, 0) * U(0, 2), sV(1, 0)* U(0, 2), sV(2, 0)* U(0, 2),
		sV(0, 0)* U(1, 2), sV(1, 0)* U(1, 2), sV(2, 0)* U(1, 2),
		sV(0, 0)* U(2, 2), sV(1, 0)* U(2, 2), sV(2, 0)* U(2, 2);
	
	E << sV(0, 1) * U(0, 0), sV(1, 1)* U(0, 0), sV(2, 1)* U(0, 0),
		sV(0, 1)* U(1, 0), sV(1, 1)* U(1, 0), sV(2, 1)* U(1, 0),
		sV(0, 1)* U(2, 0), sV(1, 1)* U(2, 0), sV(2, 1)* U(2, 0);

	F << sV(0, 0) * U(0, 1), sV(1, 0)* U(0, 1), sV(2, 0)* U(0, 1),
		sV(0, 0)* U(1, 1), sV(1, 0)* U(1, 1), sV(2, 0)* U(1, 1),
		sV(0, 0)* U(2, 1), sV(1, 0)* U(2, 1), sV(2, 0)* U(2, 1);

	// Twist eigenvectors
	Eigen::Map<Matrix<real, 3, 3>>(Q.data()) = B - A;
	Eigen::Map<Matrix<real, 3, 3>>(Q.data() + 9) = D - C;
	Eigen::Map<Matrix<real, 3, 3>>(Q.data() + 18) = F - E;

	// Flip eigenvectors
	Eigen::Map<Matrix<real, 3, 3>>(Q.data() + 27) = A + B;
	Eigen::Map<Matrix<real, 3, 3>>(Q.data() + 36) = C + D;
	Eigen::Map<Matrix<real, 3, 3>>(Q.data() + 45) = E + F;

	return Q;
}

Matrix<real,9,9> FDynamicTetrahedron::ProjectHessianWithAnalyticalFormulasNew(const real& mu, const real& lambda, const Matrix3x3<real>& F)
{
	Vector9<real> eigenvalues;
	Matrix<real, 9, 9> eigenvectors;

	UVSGroup uvsGroup = utility::numeric::RotationVariantSVD(F);

	Matrix3x3<real> U = uvsGroup.Get<0>();
	Matrix3x3<real> V = uvsGroup.Get<1>();
	Vector3<real> S = uvsGroup.Get<2>();
	
	const real J = F.determinant();
	
	// Compute the twist and flip eigenvalues
	{
		// Twist eigenvalues
		eigenvalues.segment<3>(0) = S;
		// Flip eigenvalues
		eigenvalues.segment<3>(3) = -S;
		const real evScale = lambda * (J - 1.0) - mu;
		eigenvalues.segment<6>(0) *= evScale;
		eigenvalues.segment<6>(0).array() += mu;
	}
	
	// Compute the twist and flip eigenvectors
	eigenvectors = BuildTwistAndFlipEigenvectors(U, V);
	
	// Compute the remaining three eigenvalues and eigenvectors
	{
		Matrix3x3<real> A;
		const real s0s0 = S(0) * S(0);
		const real s1s1 = S(1) * S(1);
		const real s2s2 = S(2) * S(2);
		A(0, 0) = mu + lambda * s1s1 * s2s2;
		A(1, 1) = mu + lambda * s0s0 * s2s2;
		A(2, 2) = mu + lambda * s0s0 * s1s1;
		const real evScale = lambda * (2.0 * J - 1.0) - mu;
		A(0, 1) = evScale * S(2);
		A(1, 0) = A(0, 1);
		A(0, 2) = evScale * S(1);
		A(2, 0) = A(0, 2);
		A(1, 2) = evScale * S(0);
		A(2, 1) = A(1, 2);
	
		const Eigen::SelfAdjointEigenSolver<Matrix3x3<real>> Aeigs(A);
		eigenvalues.segment<3>(6) = Aeigs.eigenvalues();
	
		Eigen::Map<Matrix3x3<real>>(eigenvectors.data() + 54) = U * Aeigs.eigenvectors().col(0).asDiagonal() * V.transpose();
		Eigen::Map<Matrix3x3<real>>(eigenvectors.data() + 63) = U * Aeigs.eigenvectors().col(1).asDiagonal() * V.transpose();
		Eigen::Map<Matrix3x3<real>>(eigenvectors.data() + 72) = U * Aeigs.eigenvectors().col(2).asDiagonal() * V.transpose();
	}
	
	// Clamp the eigenvalues
	for (int i = 0; i < 9; i++)
	{
		if (eigenvalues(i) < 0.0)
		{
			eigenvalues(i) = 0.0;
		}
	}

	return eigenvectors * eigenvalues.asDiagonal() * eigenvectors.transpose();
}

Matrix<real, 9, 9> FDynamicTetrahedron::ComputePPhiPF2StvK(real Mu, real Lambda)
{
	Matrix<real, 9, 9> PPhiPF2;
	PPhiPF2.setZero();

	// Matrix<real, 3, 3> F = GetF();
	// real I3 = F.determinant();

	return PPhiPF2;
}

Matrix<real, 9, 12> FDynamicTetrahedron::ComputePFPx()
{
	real m = DmInv(0, 0);
	real n = DmInv(0, 1);
	real o = DmInv(0, 2);
	real p = DmInv(1, 0);
	real q = DmInv(1, 1);
	real r = DmInv(1, 2);
	real s = DmInv(2, 0);
	real t = DmInv(2, 1);
	real u = DmInv(2, 2);

	real t1 = -m - p - s, t2 = -n - q - t, t3 = -o - r - u;
	
	Matrix<real, 9, 12> PFPx;
	PFPx.setZero();

	PFPx(0, 0) = t1;
	PFPx(0, 3) = m;
	PFPx(0, 6) = p;
	PFPx(0, 9) = s;
	PFPx(1, 1) = t1;
	PFPx(1, 4) = m;
	PFPx(1, 7) = p;
	PFPx(1, 10) = s;
	PFPx(2, 2) = t1;
	PFPx(2, 5) = m;
	PFPx(2, 8) = p;
	PFPx(2, 11) = s;
	PFPx(3, 0) = t2;
	PFPx(3, 3) = n;
	PFPx(3, 6) = q;
	PFPx(3, 9) = t;
	PFPx(4, 1) = t2;
	PFPx(4, 4) = n;
	PFPx(4, 7) = q;
	PFPx(4, 10) = t;
	PFPx(5, 2) = t2;
	PFPx(5, 5) = n;
	PFPx(5, 8) = q;
	PFPx(5, 11) = t;
	PFPx(6, 0) = t3;
	PFPx(6, 3) = o;
	PFPx(6, 6) = r;
	PFPx(6, 9) = u;
	PFPx(7, 1) = t3;
	PFPx(7, 4) = o;
	PFPx(7, 7) = r;
	PFPx(7, 10) = u;
	PFPx(8, 2) = t3;
	PFPx(8, 5) = o;
	PFPx(8, 8) = r;
	PFPx(8, 11) = u;

	return PFPx;
}

Matrix<real, 12, 12> FDynamicTetrahedron::ComputePPhiPx2(real Mu, real Lambda, EInternalEnergyModel EnergyModel, bool bForcePD)
{
	Matrix<real, 12, 12> PPhiPx2;
	PPhiPx2.setZero();

	if (EnergyModel == EInternalEnergyModel::IEM_NONE) return PPhiPx2;

	Matrix<real, 9, 12> PFPx = ComputePFPx();
	PPhiPx2 = -RestVolume * PFPx.transpose() * ComputePPhiPF2(Mu, Lambda, EnergyModel,bForcePD) * PFPx;
	
	return PPhiPx2;
}

Matrix<real, 12, 12> FDynamicTetrahedron::K(real Mu, real Lambda,EInternalEnergyModel EnergyModel,bool bForcePD)
{
	return ComputePPhiPx2(Mu, Lambda, EnergyModel, bForcePD);
}

Matrix<real, 3, 3> FDynamicTetrahedron::ComputeStress(real Mu, real Lambda)
{
	Matrix<real, 3, 3> P;
	P.setZero();

	Matrix<real, 3, 3> F = GetF();
	Matrix<real, 3, 3> E = 0.5f * (F.transpose() * F - Matrix<real, 3, 3>::Identity());
	P = F * (2.f * Mu * E + Lambda * E.trace() * Matrix<real, 3, 3>::Identity());
	
	return P;
}

Vector9<real> FDynamicTetrahedron::ComputePPhiPF(real Mu, real Lambda,EInternalEnergyModel EnergyModel)
{
	Matrix<real, 3, 3> PPhiPFMat;

	Matrix<real, 3, 3> F = GetF();
	real J = F.determinant();
	Matrix3x3<real> I = Matrix3x3<real>::Identity();

	// GEngine->AddOnScreenDebugMessage(-1, 5.f, FColor::Red, FString::SanitizeFloat(J));
	
	if (J < 0)
	{
		// GEngine->AddOnScreenDebugMessage(-1, 5.f, FColor::Red, FString::SanitizeFloat(J));

		// GEngine->AddOnScreenDebugMessage(-1, 5.f, FColor::Red, "Det(F)<0!");
	}
	
	// Vector3<real> f0 = F.col(0), f1 = F.col(1), f2 = F.col(2);

	// Matrix<real, 3, 3> PJPF;
	// PJPF.col(0) = f1.cross(f2);
	// PJPF.col(1) = f2.cross(f0);
	// PJPF.col(2) = f0.cross(f1);

	Matrix<real, 3, 3> FInvT = F.inverse().transpose();

	// GEngine->AddOnScreenDebugMessage(-1, 5.f, FColor::Red, FString::SanitizeFloat(FInvT.norm()));

	if (EnergyModel == EInternalEnergyModel::IEM_STABLE_NEOHOOKEAN)
	{
		// Matrix<real, 3, 3> FInvT = F.inverse().transpose();
		// PPhiPFMat = F * (2.f * Mu * E + Lambda * E.trace() * Matrix<real, 3, 3>::Identity());
		// PPhiPFMat = Mu * F + (Lambda * (J - 1.f) - Mu) * PJPF;
		PPhiPFMat = Mu * F + (Lambda * (J - 1.f) - Mu) * FInvT;
	}
	else if(EnergyModel == EInternalEnergyModel::IEM_NEOHOOKEAN)
	{
		// PPhiPFMat = Mu * (F - 1.f / J * PJPF) + (Lambda * FMath::Loge(J) / J) * PJPF;
		PPhiPFMat = Mu * (F - FInvT) + Lambda * FMath::Loge(J) * FInvT;
		// GEngine->AddOnScreenDebugMessage(-1, 5.f, FColor::Red, FString::SanitizeFloat(FMath::Loge(J)));
	}
	else if(EnergyModel == EInternalEnergyModel::IEM_STVK)
	{
		Matrix<real, 3, 3> E = 0.5f * (F.transpose() * F - I);
		PPhiPFMat = F * (2 * Mu * E + Lambda * E.trace() * I);
	}
	
	Vector9<real> PPhiPF;

	PPhiPF = Map<Vector9<real>>(PPhiPFMat.data(), PPhiPFMat.cols() * PPhiPFMat.rows());
	// GEngine->AddOnScreenDebugMessage(-1, 5.f, FColor::Red, FString::SanitizeFloat(PPhiPF.norm()));

	return PPhiPF;
}

Matrix<real, 3, 3> FDynamicTetrahedron::GetF()
{
	return GetDs() * DmInv;
}

TTuple<int, int, int, int> FDynamicTetrahedron::PointIndices()
{
	return TTuple<int, int, int, int>(p0->PointIndex, p1->PointIndex,
		p2->PointIndex, p3->PointIndex);
}
