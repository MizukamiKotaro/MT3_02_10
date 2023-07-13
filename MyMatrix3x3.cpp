#include "MyMatrix3x3.h"
#include<cmath>
#include<assert.h>

MyMatrix3x3 MyMatrix3x3::Add (const MyMatrix3x3 m1, const MyMatrix3x3& m2) {
	MyMatrix3x3 result = {};
	for (int row = 0; row < 3; row++) {
		for (int column = 0; column < 3; column++) {
			result.m[row][column] = m1.m[row][column] + m2.m[row][column];
		}
	}
	return result;
}

MyMatrix3x3 MyMatrix3x3::Subtract(const MyMatrix3x3& m1, const MyMatrix3x3& m2) {
	MyMatrix3x3 result = {};
	for (int row = 0; row < 3; row++) {
		for (int column = 0; column < 3; column++) {
			result.m[row][column] = m1.m[row][column] - m2.m[row][column];
		}
	}
	return result;
}

MyMatrix3x3 MyMatrix3x3::Multiply(const MyMatrix3x3& m1, const MyMatrix3x3& m2) {
	MyMatrix3x3 result = {};
	for (int i = 0; i < 3; i++) {
		for (int row = 0; row < 3; row++) {
			for (int column = 0; column < 3; column++) {
				result.m[i][row] += m1.m[i][column] * m2.m[column][row];
			}
		}
	}
	return result;
}

MyMatrix3x3 MyMatrix3x3::Multiply(const float& num, const MyMatrix3x3& m) {
	MyMatrix3x3 result = {};
	for (int row = 0; row < 3; row++) {
		for (int column = 0; column < 3; column++) {
			result.m[row][column] = m.m[row][column] * num;
		}
	}
	return result;
}

MyVector2 MyMatrix3x3::Multiply(const MyVector2& vector, const MyMatrix3x3& m) {
	MyVector2 result = {};
	result.x = vector.x * m.m[0][0] + vector.y * m.m[1][0] + 1.0f * m.m[2][0];
	result.y = vector.x * m.m[0][1] + vector.y * m.m[1][1] + 1.0f * m.m[2][1];
	float w = vector.x * m.m[0][2] + vector.y * m.m[1][2] + 1.0f * m.m[2][2];
	assert(w != 0.0f);
	result.x /= w;
	result.y /= w;

	return result;
}

MyMatrix3x3 MyMatrix3x3::Inverse(const MyMatrix3x3& m) {
	float num = MyMatrix3x3::SarrasRule(m);
	struct MyMatrix2x2
	{
		float m[2][2];
	};
	MyMatrix2x2 m2[3][3] = {};

	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {
			for (int row = 0; row < 2; row++) {
				for (int column = 0; column < 2; column++) {
					if (row >= i) {
						if (column >= j) {
							m2[i][j].m[row][column] = m.m[row + 1][column + 1];
						}
						else {
							m2[i][j].m[row][column] = m.m[row + 1][column];
						}
					}
					else {
						if (column >= j) {
							m2[i][j].m[row][column] = m.m[row][column + 1];
						}
						else {
							m2[i][j].m[row][column] = m.m[row][column];
						}
					}
				}
			}
		}
	}

	if (num != 0) {
		num = 1.0f / num;
	}
	MyMatrix3x3 result = {};

	for (int row = 0; row < 3; row++) {
		for (int column = 0; column < 3; column++) {
			if ((row + column) % 2 == 0) {
				result.m[row][column] = num * (m2[column][row].m[0][0] * m2[column][row].m[1][1] - m2[column][row].m[0][1] * m2[column][row].m[1][0]);
			}
			else {
				result.m[row][column] = -num * (m2[column][row].m[0][0] * m2[column][row].m[1][1] - m2[column][row].m[0][1] * m2[column][row].m[1][0]);
			}
		}
	}

	return result;
}

MyMatrix3x3 MyMatrix3x3::MakeAffineTransformMatrix(const MyVector2& scale, const float& radian, const MyVector2& translate) {
	MyMatrix3x3 matrix = {
	   {{float(scale.x * std::cos(radian)), float(scale.x * std::sin(radian)),0},
	   {float(-scale.y * std::sin(radian)), float(scale.y * std::cos(radian)),0},
	   {translate.x,translate.y,1}
	   }
	};
	return matrix;
}

float MyMatrix3x3::SarrasRule(const MyMatrix3x3& m) {
	float result = m.m[0][0] * m.m[1][1] * m.m[2][2] + m.m[0][1] * m.m[1][2] * m.m[2][0] + m.m[0][2] * m.m[1][0] * m.m[2][1] -
		m.m[0][2] * m.m[1][1] * m.m[2][0] - m.m[0][1] * m.m[1][0] * m.m[2][2] - m.m[0][0] * m.m[1][2] * m.m[2][1];
	return result;
}

MyMatrix3x3 MyMatrix3x3::MakeOrthographicMatrix(const float& left, const  float& top, const  float& right, const  float& bottom) {
	MyMatrix3x3 matrix = {};
	matrix.m[0][0] = 2.0f / (right - left); matrix.m[0][1] = 0; matrix.m[0][2] = 0;
	matrix.m[1][0] = 0; matrix.m[1][1] = 2.0f / (top - bottom); matrix.m[1][2] = 0;
	matrix.m[2][0] = (left + right) / (left - right); matrix.m[2][1] = (top + bottom) / (bottom - top); matrix.m[2][2] = 1;
	return matrix;
}

MyMatrix3x3 MyMatrix3x3::MakeViewportMatrix(const float& left, const  float& top, const  float& right, const  float& bottom) {
	MyMatrix3x3 matrix = {};
	matrix.m[0][0] = (right - left) / 2; matrix.m[0][1] = 0; matrix.m[0][2] = 0;
	matrix.m[1][0] = 0; matrix.m[1][1] = (top - bottom) / 2; matrix.m[1][2] = 0;
	matrix.m[2][0] = left + (right - left) / 2; matrix.m[2][1] = top + (bottom - top) / 2; matrix.m[2][2] = 1;
	return matrix;
}

MyMatrix3x3 MyMatrix3x3::MakeWvpVpMatrix(const MyVector2& objectScale, const float& objectRadian, const MyVector2& objectTranslate,
	const MyVector2& cameraScale, const float& cameraRadian, const MyVector2& cameraTranslate,
	const MyVector2& orthoTopLeft, const MyVector2& orthoBottomRight, const MyVector2& viewportTopLeft, const MyVector2& viewportBottomRight) {

	MyMatrix3x3 objectWorldMatrix = MyMatrix3x3::MakeAffineTransformMatrix(objectScale, objectRadian, objectTranslate);
	MyMatrix3x3 cameraWorldMatrix = MyMatrix3x3::MakeAffineTransformMatrix(cameraScale, cameraRadian, cameraTranslate);
	MyMatrix3x3 viewMatrix = MyMatrix3x3::Inverse(cameraWorldMatrix);
	MyMatrix3x3 orthoMatrix = MyMatrix3x3::MakeOrthographicMatrix(orthoTopLeft.x, orthoTopLeft.y, orthoBottomRight.x, orthoBottomRight.y);
	MyMatrix3x3 ViewportMatrix = MyMatrix3x3::MakeViewportMatrix(viewportTopLeft.x, viewportTopLeft.y, viewportBottomRight.x, viewportBottomRight.y);

	MyMatrix3x3 wvpVpMatrix = MyMatrix3x3::Multiply(objectWorldMatrix, viewMatrix);
	wvpVpMatrix = MyMatrix3x3::Multiply(wvpVpMatrix, orthoMatrix);
	wvpVpMatrix = MyMatrix3x3::Multiply(wvpVpMatrix, ViewportMatrix);

	return wvpVpMatrix;
}


