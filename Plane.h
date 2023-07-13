#pragma once
#include"MyVector3.h"
#include"MyMatrix4x4.h"
#include<cstdint>

class Plane {
public:
	void Initialize();

	void Update();

	static void Draw(const Plane& plane, const MyMatrix4x4& viewProjectionMatrix, const MyMatrix4x4& viewportMatrix, uint32_t color);

public:
	MyVector3 normal; // 法線
	float distance; // 距離
	bool isLimit = false;
};
