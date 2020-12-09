// Fill out your copyright notice in the Description page of Project Settings.


#include "UtilityGeometry.h"
#include "Misc/FileHelper.h"
#include "UtilityFunc.h"
#include "UtilityMath.h"
#include "FTetrahedronMesh.h"
#include "GenericPlatform/GenericPlatformMath.h"

namespace utility
{
	namespace geometry
	{
		void ParseTetFile(const FString& TetFilePath, FTetrahedronMesh* TetrahedronMeshPt)
		{
			if (TetrahedronMeshPt == nullptr) return;

			FString FileData;
			FFileHelper::LoadFileToString(FileData, *TetFilePath);
			TArray<FString> Lines;
			int32 LineCount = FileData.ParseIntoArrayLines(Lines);

			FString SpaceDelimiter(" ");
			FString SlashDelimiter("/");
			FString DoubleSlashDelimiter("//");

			TArray<Vector3<real>> PositionArray;
			TArray<Vector2<real>> UvArray;
			TArray<Vector3<real>> NormalArray;

			TArray<Vector4<int>> TetPointIndexArray;
			TArray<int> TrianglePointIndexArray;
			TArray<int> TriangleUvIndexArray;
			TArray<int> TriangleNormalIndexArray;

			for (int i = 0; i < LineCount; i++)
			{
				auto CurLine = Lines[i];
				if (!utility::func::IsValidLine(CurLine)) continue;

				TArray<FString> LineElements;
				CurLine.ParseIntoArray(LineElements, *SpaceDelimiter);

				if (LineElements[0] == "v")	//points in mesh
				{
					// UE_LOG(LogTemp, Display, TEXT("233333"));
					// if(LineElements[1].IsNumeric() && 
					// 	LineElements[2].IsNumeric() && 
					// 	LineElements[3].IsNumeric())
					{
						float x, y, z;
						x = FCString::Atof(*LineElements[1]);
						y = FCString::Atof(*LineElements[2]);
						z = FCString::Atof(*LineElements[3]);
						PositionArray.Add(Vector3<real>(x, y, z));
					}
					// else
					// {
					// 	UE_LOG(LogTemp, Warning, TEXT("vertex position in tet is not numeric type"));
					// }

				}
				else if (LineElements[0] == "tet")	//tetrahedron element
				{
					// if (LineElements[1].IsNumeric() &&
					// 	LineElements[2].IsNumeric() &&
					// 	LineElements[3].IsNumeric() &&
					// 	LineElements[4].IsNumeric())
					{
						int index0, index1, index2, index3;
						index0 = FCString::Atoi(*LineElements[1]);
						index1 = FCString::Atoi(*LineElements[2]);
						index2 = FCString::Atoi(*LineElements[3]);
						index3 = FCString::Atoi(*LineElements[4]);

						TetPointIndexArray.Add(Vector4<int>(index0, index1, index2, index3));
					}
					// else
					// {
					// 	UE_LOG(LogTemp, Warning, TEXT("vertex index in tet is not numeric type"));
					// }
				}
				else if (LineElements[0] == "f")		//point indices of triangle faces
				{
					for (int j = 1; j < LineElements.Num(); j++)
					{
						FString CurLineElement = LineElements[j];
						TArray<FString> IndexElements;
						CurLineElement.ParseIntoArray(IndexElements, *SlashDelimiter);

						if (IndexElements.Num() >= 1)
						{
							TrianglePointIndexArray.Add(FCString::Atoi(*IndexElements[0]));
						}

						if (IndexElements.Num() == 2)
						{
							if (!CurLineElement.Contains(DoubleSlashDelimiter))
								TriangleUvIndexArray.Add(FCString::Atoi(*IndexElements[1]));
							else
								TriangleNormalIndexArray.Add(FCString::Atoi(*IndexElements[1]));
						}

						if (IndexElements.Num() == 3)
						{
							TriangleUvIndexArray.Add(FCString::Atoi(*IndexElements[1]));
							TriangleNormalIndexArray.Add(FCString::Atoi(*IndexElements[2]));
						}
					}
				}
				else if (LineElements[0] == "vt")	//uv coordinates
				{
					// if (LineElements[1].IsNumeric() &&
					// 	LineElements[2].IsNumeric())
					// {
					float u, v;
					u = FCString::Atof(*LineElements[1]);
					v = FCString::Atof(*LineElements[2]);
					UvArray.Add(Vector2<real>(u, v));
					// }
					// else
					// {
					// 	UE_LOG(LogTemp, Warning, TEXT("uv coordinate in tet is not numeric type"));
					// }
				}
				else if (LineElements[0] == "vn")	//normals
				{
					// if (LineElements[1].IsNumeric() &&
					// 	LineElements[2].IsNumeric() &&
					// 	LineElements[3].IsNumeric())
					{
						float nx, ny, nz;
						nx = FCString::Atof(*LineElements[1]);
						ny = FCString::Atof(*LineElements[2]);
						nz = FCString::Atof(*LineElements[3]);
						NormalArray.Add(Vector3<real>(nx, ny, nz));
					}
					// else
					// {
					// 	UE_LOG(LogTemp, Warning, TEXT("vertex normal in tet is not numeric type"));
					// }
				}
			}

			// UE_LOG(LogTemp, Display, TEXT("DynamicPointNumber: %d\n"), PositionArray.Num());
			for (int i = 0; i < PositionArray.Num(); i++)
			{
				TetrahedronMeshPt->DynamicPointArray
					.Add(MakeShared<FTetDynamicPoint>(PositionArray[i], i));
			}

			for (int i = 0; i < TetPointIndexArray.Num(); i++)
			{
				Vector4<int> CurPointIndices = TetPointIndexArray[i];
				int Index0 = CurPointIndices(0);
				int Index1 = CurPointIndices(1);
				int Index2 = CurPointIndices(2);
				int Index3 = CurPointIndices(3);

				// UE_LOG(LogTemp, Display, TEXT("%d %d %d %d\n"), Index0, Index1, Index2, Index3);

				TetrahedronMeshPt->DynamicTetrahedronArray
					.Add(
						MakeShared<FDynamicTetrahedron>
					(TetrahedronMeshPt->DynamicPointArray[Index0],
						TetrahedronMeshPt->DynamicPointArray[Index1],
						TetrahedronMeshPt->DynamicPointArray[Index2],
						TetrahedronMeshPt->DynamicPointArray[Index3],
						i));
			}

			TetrahedronMeshPt->RenderableUvArray = UvArray;
			TetrahedronMeshPt->RenderableNormalArray = NormalArray;
			TetrahedronMeshPt->RenderableTriangleIndexArray = TrianglePointIndexArray;

			TSet<int> AuxSet;

			int MaxIndexNumber = -1;
			for (int i = 0; i < TrianglePointIndexArray.Num(); i++)
			{
				MaxIndexNumber = std::max(MaxIndexNumber, TrianglePointIndexArray[i]);
				if (!AuxSet.Contains(TrianglePointIndexArray[i]))
				{
					AuxSet.Add(TrianglePointIndexArray[i]);
					TetrahedronMeshPt->RenderablePointIndexArray
						.Add(TrianglePointIndexArray[i]);
				}
			}

			// UE_LOG(LogTemp, Display, TEXT("Max : %d\n"), MaxIndexNumber);

			TetrahedronMeshPt->RenderablePointIndexArray.Sort();

			// for (int i = 0; i < TetrahedronMeshPt->RenderablePointIndexArray.Num(); i++)
			// {
			// 	UE_LOG(LogTemp, Display, TEXT("%d\n"), TetrahedronMeshPt->RenderablePointIndexArray[i]);
			// }
		}

		Vector3<real> TriangleFaceNormal(const Vector3<real> P0, const Vector3<real> P1, const Vector3<real> P2)
		{
			return (P1 - P0).cross(P2 - P0).normalized();
		}
	}
}

