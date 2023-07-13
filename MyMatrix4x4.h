#pragma once
#include"MyVector3.h"

class MyMatrix4x4
{
public:
	enum RotateType {
		kXYZ,
		kYXZ,
		kZXY,
		kXZY,
		kYZX,
		kZYX
	};

public:
	static MyMatrix4x4 Add(const MyMatrix4x4& m1, const MyMatrix4x4& m2);

	static MyMatrix4x4 Subtract(const MyMatrix4x4& m1, const MyMatrix4x4& m2);

	static MyMatrix4x4 Multiply(const MyMatrix4x4& m1, const MyMatrix4x4& m2);

	static MyMatrix4x4 Multiply(const float& num, const MyMatrix4x4& m);

	static MyVector3 Multiply(const MyVector3& vector, const MyMatrix4x4& m);

	static MyMatrix4x4 Inverse(const MyMatrix4x4& m);

	//転置行列
	static MyMatrix4x4 Transpose(const MyMatrix4x4& m);

	//単位行列
	static MyMatrix4x4 MakeIdentity4x4();

	static MyMatrix4x4 MakeTranslateMatrix(const MyVector3& translate);

	static MyMatrix4x4 MakeScaleMatrix(const MyVector3& scale);

	static MyVector3 Transform(const MyVector3& vector, const MyMatrix4x4& m);

	static MyMatrix4x4 MakeRotateXMatrix(float radian);

	static MyMatrix4x4 MakeRotateYMatrix(float radian);

	static MyMatrix4x4 MakeRotateZMatrix(float radian);

	static MyMatrix4x4 MakeRotateXYZMatrix(const MyVector3& rotate);

	static MyMatrix4x4 MakeRotateMatrix(const MyVector3& rotate, RotateType rotateOrder);

	static MyMatrix4x4 MakeAffinMatrix(const MyVector3& scale, const MyVector3& rotate, const MyVector3& translate);

	static MyMatrix4x4 MakeAffinMatrix(const MyVector3& scale, const MyVector3& rotate, const MyVector3& translate, RotateType rotateOrder);

	//透視投影行列
	static MyMatrix4x4 MakePerspectiveFovMatrix(float fovY, float aspectRatio, float nearClip, float farClip);
	//正射影行列
	static MyMatrix4x4 MakeOrthographicMatrix(float left, float top, float right, float bottom, float nearClip, float farClip);
	//ビューポート変換行列
	static MyMatrix4x4 MakeViewportMatrix(float left, float top, float width, float height, float minDepth, float maxDepth);

	/*static MyMatrix4x4 MakeWvpVpMatrix(const MyVector2& objectScale, const float& objectRadian, const MyVector2& objectTranslate,
		const MyVector2& cameraScale, const float& cameraRadian, const MyVector2& cameraTranslate,
		const MyVector2& orthoTopLeft, const MyVector2& orthoBottomRight, const MyVector2& viewportTopLeft, const MyVector2& viewportBottomRight);*/

public:
	float m[4][4];
};