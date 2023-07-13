#pragma once
#include"MyVector3.h"
#include"MyMatrix4x4.h"
#include<cstdint>

class AABB
{
public:
	
	void Draw(const MyMatrix4x4& viewProjectionMatrix, const MyMatrix4x4& viewportMatrix, uint32_t color);

private:


public:
	MyVector3 min;
	MyVector3 max;
};

