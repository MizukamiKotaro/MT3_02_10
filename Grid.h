#pragma once
#include"MyMatrix4x4.h"
#include<stdint.h>

class Grid
{
public:
	static void DrawGrid(const MyMatrix4x4& viewProjectionMatrixMatrix, const MyMatrix4x4& viewportMatrix);
};

