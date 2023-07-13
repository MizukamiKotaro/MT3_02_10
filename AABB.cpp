#include "AABB.h"
#include "Novice.h"

void AABB::Draw(const MyMatrix4x4& viewProjectionMatrix, const MyMatrix4x4& viewportMatrix, uint32_t color) {

	MyVector3 radius = (max - min) / 2.0f;

	MyVector3 center = radius + min;

	MyVector3 verteces[8] = {
		{-radius.x,radius.y,-radius.z},
		{radius.x,radius.y,-radius.z},
		{radius.x,radius.y,radius.z},
		{-radius.x,radius.y,radius.z},
		{-radius.x,-radius.y,-radius.z},
		{radius.x,-radius.y,-radius.z},
		{radius.x,-radius.y,radius.z},
		{-radius.x,-radius.y,radius.z},
	};

	MyMatrix4x4 worldViewProjectionMatrix = MyMatrix4x4::Multiply(MyMatrix4x4::MakeAffinMatrix({1.0f,1.0f,1.0f}, {0.0f,0.0f,0.0f}, center), viewProjectionMatrix);
	MyVector3 screen[8] = {};

	for (int i = 0; i < 8; i++) {
		screen[i] = MyMatrix4x4::Transform(MyMatrix4x4::Transform(verteces[i], worldViewProjectionMatrix), viewportMatrix);
	}

	Novice::DrawLine(int(screen[2].x), int(screen[2].y), int(screen[1].x), int(screen[1].y), color);
	Novice::DrawLine(int(screen[0].x), int(screen[0].y), int(screen[3].x), int(screen[3].y), color);
	Novice::DrawLine(int(screen[3].x), int(screen[3].y), int(screen[2].x), int(screen[2].y), color);
	Novice::DrawLine(int(screen[0].x), int(screen[0].y), int(screen[1].x), int(screen[1].y), color);

	Novice::DrawLine(int(screen[5].x), int(screen[5].y), int(screen[6].x), int(screen[6].y), color);
	Novice::DrawLine(int(screen[5].x), int(screen[5].y), int(screen[4].x), int(screen[4].y), color);
	Novice::DrawLine(int(screen[6].x), int(screen[6].y), int(screen[7].x), int(screen[7].y), color);
	Novice::DrawLine(int(screen[4].x), int(screen[4].y), int(screen[7].x), int(screen[7].y), color);

	Novice::DrawLine(int(screen[0].x), int(screen[0].y), int(screen[4].x), int(screen[4].y), color);
	Novice::DrawLine(int(screen[1].x), int(screen[1].y), int(screen[5].x), int(screen[5].y), color);
	Novice::DrawLine(int(screen[2].x), int(screen[2].y), int(screen[6].x), int(screen[6].y), color);
	Novice::DrawLine(int(screen[3].x), int(screen[3].y), int(screen[7].x), int(screen[7].y), color);
}


