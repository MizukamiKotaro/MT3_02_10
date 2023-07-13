#pragma once
#include"MyVector2.h"

class MyMatrix3x3 
{
public:
	static MyMatrix3x3 Add(const MyMatrix3x3 m1, const MyMatrix3x3& m2);

	static MyMatrix3x3 Subtract(const MyMatrix3x3& m1, const MyMatrix3x3& m2);

	static MyMatrix3x3 Multiply(const MyMatrix3x3& m1, const MyMatrix3x3& m2);

	static MyMatrix3x3 Multiply(const float& num, const MyMatrix3x3& m);

	static MyVector2 Multiply(const MyVector2& vector, const MyMatrix3x3& m);

	static MyMatrix3x3 Inverse(const MyMatrix3x3& m);

	static MyMatrix3x3 MakeAffineTransformMatrix(const MyVector2& scale, const float& radian, const MyVector2& translate);

	static float SarrasRule(const MyMatrix3x3& m);

	static MyMatrix3x3 MakeOrthographicMatrix(const float& left, const float& top, const float& right, const float& bottom);

	static MyMatrix3x3 MakeViewportMatrix(const float& left, const  float& top, const  float& right, const  float& bottom);

	static MyMatrix3x3 MakeWvpVpMatrix(const MyVector2& objectScale, const float& objectRadian, const MyVector2& objectTranslate,
		const MyVector2& cameraScale, const float& cameraRadian, const MyVector2& cameraTranslate,
		const MyVector2& orthoTopLeft, const MyVector2& orthoBottomRight, const MyVector2& viewportTopLeft, const MyVector2& viewportBottomRight);

public:
	float m[3][3];
};

