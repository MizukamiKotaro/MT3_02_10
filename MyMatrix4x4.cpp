#include "MyMatrix4x4.h"
#include "MyMatrix3x3.h"
#include<cmath>
#include<assert.h>

MyMatrix4x4 MyMatrix4x4::Add(const MyMatrix4x4& m1, const MyMatrix4x4& m2) {
	MyMatrix4x4 result = {};
	for (int row = 0; row < 4; row++) {
		for (int column = 0; column < 4; column++) {
			result.m[row][column] = m1.m[row][column] + m2.m[row][column];
		}
	}
	return result;
}

MyMatrix4x4 MyMatrix4x4::Subtract(const MyMatrix4x4& m1, const MyMatrix4x4& m2) {
	MyMatrix4x4 result = {};
	for (int row = 0; row < 4; row++) {
		for (int column = 0; column < 4; column++) {
			result.m[row][column] = m1.m[row][column] - m2.m[row][column];
		}
	}
	return result;
}

MyMatrix4x4 MyMatrix4x4::Multiply(const MyMatrix4x4& m1, const MyMatrix4x4& m2) {
	MyMatrix4x4 result = {};
	for (int i = 0; i < 4; i++) {
		for (int row = 0; row < 4; row++) {
			for (int column = 0; column < 4; column++) {
				result.m[i][row] += m1.m[i][column] * m2.m[column][row];
			}
		}
	}
	return result;
}

MyMatrix4x4 MyMatrix4x4::Multiply(const float& num, const MyMatrix4x4& m) {
	MyMatrix4x4 result = {};
	for (int row = 0; row < 4; row++) {
		for (int column = 0; column < 4; column++) {
			result.m[row][column] = num * m.m[row][column];
		}
	}
	return result;
}

MyVector3 MyMatrix4x4::Multiply(const MyVector3& vector, const MyMatrix4x4& m) {
	MyVector3 result = {};
	result.x = vector.x * m.m[0][0] + vector.y * m.m[1][0] + vector.z * m.m[2][0] + 1.0f * m.m[3][0];
	result.y = vector.x * m.m[0][1] + vector.y * m.m[1][1] + vector.z * m.m[2][1] + 1.0f * m.m[3][1];
	result.z = vector.x * m.m[0][2] + vector.y * m.m[1][2] + vector.z * m.m[2][2] + 1.0f * m.m[3][2];
	float w = vector.x * m.m[0][3] + vector.y * m.m[1][3] + vector.z * m.m[2][3] + 1.0f * m.m[3][3];
	assert(w != 0.0f);
	result.x /= w;
	result.y /= w;
	result.z /= w;

	return result;
}

MyMatrix4x4 MyMatrix4x4::Inverse(const MyMatrix4x4& m) {
	MyMatrix4x4 result = {};
	
	float A[4] = {};

	//小行列
	MyMatrix3x3 b[4][4] = {};
	int B[4][4] = {};
	for (int i = 0; i < 4; i++) {
		A[i] = m.m[i][0];
		for(int j=0;j<4;j++){
			B[i][j] = i + j;
			for (int row = 0; row < 3; row++) {
				for (int column = 0; column < 3; column++) {
					if (i <= row) {
						if (j <= column) {
							b[i][j].m[row][column] = m.m[row + 1][column + 1];
						}
						else {
							b[i][j].m[row][column] = m.m[row + 1][column];
						}
					}
					else {
						if (j <= column) {
							b[i][j].m[row][column] = m.m[row][column + 1];
						}
						else {
							b[i][j].m[row][column] = m.m[row][column];
						}
					}
				}
			}
		}
	}

	float num = 0;

	MyMatrix4x4 c = {};
	for (int row = 0; row < 4; row++) {
		if (row == 0 || row == 2) {
			num += A[row] * MyMatrix3x3::SarrasRule(b[row][0]);
		}
		else {
			num -= A[row] * MyMatrix3x3::SarrasRule(b[row][0]);
		}
		for (int column = 0; column < 4; column++) {
			if (B[row][column] % 2 == 0) {
				c.m[row][column] = MyMatrix3x3::SarrasRule(b[column][row]);
			}
			else {
				c.m[row][column] = -1 * MyMatrix3x3::SarrasRule(b[column][row]);
			}
		}
	}

	if (num != 0) {
		num = float(1 / num);
	}

	result = Multiply(num, c);

	return result;
}

//転置行列
MyMatrix4x4 MyMatrix4x4::Transpose(const MyMatrix4x4& m) {
	MyMatrix4x4 result = {};
	for (int row = 0; row < 4; row++) {
		for (int column = 0; column < 4; column++) {
			result.m[row][column] = m.m[column][row];
		}
	}
	return result;
}

//単位行列
MyMatrix4x4 MyMatrix4x4::MakeIdentity4x4() {
	MyMatrix4x4 result = {};
	for (int row = 0; row < 4; row++) {
		for (int column = 0; column < 4; column++) {
			if (row == column) {
				result.m[row][column] = 1;
			}
			else {
				result.m[row][column] = 0;
			}
		}
	}
	return result;
}

MyMatrix4x4 MyMatrix4x4::MakeTranslateMatrix(const MyVector3& translate) {
	MyMatrix4x4 result = {
		 1,0,0,0,
		 0,1,0,0,
		 0,0,1,0,
		 translate.x,translate.y,translate.z,1
	};

	return result;
}

MyMatrix4x4 MyMatrix4x4::MakeScaleMatrix(const MyVector3& scale) {
	MyMatrix4x4 result = {
		 scale.x,0,0,0,
		 0,scale.y,0,0,
		 0,0,scale.z,0,
		 0,0,0,1
	};

	return result;
}

MyVector3 MyMatrix4x4::Transform(const MyVector3& vector, const MyMatrix4x4& m) {
	MyVector3 result = MyMatrix4x4::Multiply(vector, m);

	return result;
}

MyMatrix4x4 MyMatrix4x4::MakeRotateXMatrix(float radian) {
	MyMatrix4x4 result = {
		1,0,0,0,
		0,std::cos(radian),std::sin(radian),0,
		0,-std::sin(radian),std::cos(radian),0,
		0,0,0,1
	};
	return result;
}

MyMatrix4x4 MyMatrix4x4::MakeRotateYMatrix(float radian) {
	MyMatrix4x4 result = {
		std::cos(radian),0,-std::sin(radian),0,
		0,1,0,0,
		std::sin(radian),0,std::cos(radian),0,
		0,0,0,1
	};
	return result;
}

MyMatrix4x4 MyMatrix4x4::MakeRotateZMatrix(float radian) {
	MyMatrix4x4 result = {
		std::cos(radian),std::sin(radian),0,0,
		-std::sin(radian),std::cos(radian),0,0,
		0,0,1,0,
		0,0,0,1
	};
	return result;
}

MyMatrix4x4 MyMatrix4x4::MakeRotateXYZMatrix(const MyVector3& rotate) {
	MyMatrix4x4 result = MyMatrix4x4::Multiply(MyMatrix4x4::MakeRotateXMatrix(rotate.x),
		MyMatrix4x4::Multiply(MyMatrix4x4::MakeRotateYMatrix(rotate.y), MyMatrix4x4::MakeRotateZMatrix(rotate.z)));
	return result;
}

MyMatrix4x4 MyMatrix4x4::MakeRotateMatrix(const MyVector3& rotate, RotateType rotateOrder) {
	MyMatrix4x4 result = {};
	switch (rotateOrder)
	{
	case MyMatrix4x4::kXYZ:
		result = MyMatrix4x4::Multiply(MyMatrix4x4::MakeRotateXMatrix(rotate.x),
			MyMatrix4x4::Multiply(MyMatrix4x4::MakeRotateYMatrix(rotate.y), MyMatrix4x4::MakeRotateZMatrix(rotate.z)));
		return result;
		break;
	case MyMatrix4x4::kYXZ:
		result = MyMatrix4x4::Multiply(MyMatrix4x4::MakeRotateYMatrix(rotate.y),
			MyMatrix4x4::Multiply(MyMatrix4x4::MakeRotateXMatrix(rotate.x), MyMatrix4x4::MakeRotateZMatrix(rotate.z)));
		return result;
		break;
	case MyMatrix4x4::kZXY:
		result = MyMatrix4x4::Multiply(MyMatrix4x4::MakeRotateZMatrix(rotate.z),
			MyMatrix4x4::Multiply(MyMatrix4x4::MakeRotateXMatrix(rotate.x), MyMatrix4x4::MakeRotateYMatrix(rotate.y)));
		return result;
		break;
	case MyMatrix4x4::kXZY:
		result = MyMatrix4x4::Multiply(MyMatrix4x4::MakeRotateXMatrix(rotate.x),
			MyMatrix4x4::Multiply(MyMatrix4x4::MakeRotateZMatrix(rotate.z), MyMatrix4x4::MakeRotateYMatrix(rotate.y)));
		return result;
		break;
	case MyMatrix4x4::kYZX:
		result = MyMatrix4x4::Multiply(MyMatrix4x4::MakeRotateYMatrix(rotate.y),
			MyMatrix4x4::Multiply(MyMatrix4x4::MakeRotateZMatrix(rotate.z), MyMatrix4x4::MakeRotateXMatrix(rotate.x)));
		return result;
		break;
	case MyMatrix4x4::kZYX:
		result = MyMatrix4x4::Multiply(MyMatrix4x4::MakeRotateZMatrix(rotate.z),
			MyMatrix4x4::Multiply(MyMatrix4x4::MakeRotateYMatrix(rotate.y), MyMatrix4x4::MakeRotateXMatrix(rotate.x)));
		return result;
		break;
	default:
		return result;
		break;
	}
	return result;
}

MyMatrix4x4 MyMatrix4x4::MakeAffinMatrix(const MyVector3& scale, const MyVector3& rotate, const MyVector3& translate) {
	/*MyMatrix4x4 result = MyMatrix4x4::Multiply(MyMatrix4x4::MakeScaleMatrix(scale),
		MyMatrix4x4::Multiply(MyMatrix4x4::MakeRotateXYZMatrix(rotate), MyMatrix4x4::MakeTranslateMatrix(translate)));*/

	MyMatrix4x4 rotateMatrix = MyMatrix4x4::MakeRotateXYZMatrix(rotate);

	MyMatrix4x4 result = {
		scale.x * rotateMatrix.m[0][0],scale.x * rotateMatrix.m[0][1],scale.x * rotateMatrix.m[0][2],0,
		scale.y * rotateMatrix.m[1][0],scale.y * rotateMatrix.m[1][1],scale.y * rotateMatrix.m[1][2],0,
		scale.z * rotateMatrix.m[2][0],scale.z * rotateMatrix.m[2][1],scale.z * rotateMatrix.m[2][2],0,
		translate.x,translate.y,translate.z,1
	};

	return result;
}

MyMatrix4x4 MyMatrix4x4::MakeAffinMatrix(const MyVector3& scale, const MyVector3& rotate, const MyVector3& translate, RotateType rotateOrder) {
	/*MyMatrix4x4 result = MyMatrix4x4::Multiply(MyMatrix4x4::MakeScaleMatrix(scale),
		MyMatrix4x4::Multiply(MyMatrix4x4::MakeRotateXYZMatrix(rotate), MyMatrix4x4::MakeTranslateMatrix(translate)));*/

	MyMatrix4x4 rotateMatrix = MyMatrix4x4::MakeRotateMatrix(rotate , rotateOrder);

	MyMatrix4x4 result = {
		scale.x * rotateMatrix.m[0][0],scale.x * rotateMatrix.m[0][1],scale.x * rotateMatrix.m[0][2],0,
		scale.y * rotateMatrix.m[1][0],scale.y * rotateMatrix.m[1][1],scale.y * rotateMatrix.m[1][2],0,
		scale.z * rotateMatrix.m[2][0],scale.z * rotateMatrix.m[2][1],scale.z * rotateMatrix.m[2][2],0,
		translate.x,translate.y,translate.z,1
	};

	return result;
}

MyMatrix4x4 MyMatrix4x4::MakePerspectiveFovMatrix(float fovY, float aspectRatio, float nearClip, float farClip) {
	MyMatrix4x4 result = {
		1.0f / (aspectRatio * tan(fovY / 2)),0,0,0,
		0,1.0f / tan(fovY / 2),0,0,
		0,0,farClip / (farClip - nearClip),1,
		0,0,-nearClip * farClip / (farClip - nearClip),0
	};
	return result;
}

MyMatrix4x4 MyMatrix4x4::MakeOrthographicMatrix(float left, float top, float right, float bottom, float nearClip, float farClip) {
	MyMatrix4x4 result = {
		2.0f / (right - left),0,0,0,
		0,2.0f / (top - bottom),0,0,
		0,0,1.0f / (farClip - nearClip),0,
		(left + right) / (left - right),(top + bottom) / (bottom - top),nearClip / (nearClip - farClip),1
	};
	return result;
}

MyMatrix4x4 MyMatrix4x4::MakeViewportMatrix(float left, float top, float width, float height, float minDepth, float maxDepth) {
	MyMatrix4x4 result = {
		width / 2,0,0,0,
		0,-height / 2,0,0,
		0,0,maxDepth - minDepth,0,
		left + width / 2,top + height / 2,minDepth,1
	};
	return result;
}