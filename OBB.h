#pragma once
#include"MyVector3.h"
#include"MyMatrix4x4.h"
#include<cstdint>

class OBB
{
public:

	void Draw(const MyMatrix4x4& viewProjectionMatrix, const MyMatrix4x4& viewportMatrix, uint32_t color);

	void SetOrientations(const MyMatrix4x4& rotateMat);

public:
	
	MyVector3 center;
	MyVector3 orientations[3]; // 座標軸。正規化、直交必須
	MyVector3 size;
};

