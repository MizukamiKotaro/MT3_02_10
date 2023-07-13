#pragma once
#include"MyVector3.h"
#include"MyMatrix4x4.h"
#include<cstdint>

class Triangle
{
public:
	void Draw(const MyMatrix4x4& viewProjectionMatrix, const MyMatrix4x4& viewportMatrix, uint32_t color);
public:
	MyVector3 vertices[3];
};

