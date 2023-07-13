#include "Grid.h"
#include "Novice.h"

void Grid::DrawGrid(const MyMatrix4x4& viewProjectionMatrix, const MyMatrix4x4& viewportMatrix) {
	const float kGridHalfWidth = 2.0f; // Gridの半分の幅
	const uint32_t kSubdivision = 10; // 分割数
	const float kGridEvery = (kGridHalfWidth * 2.0f) / float(kSubdivision); // 1つ分の長さ

	MyMatrix4x4 worldMatrix = MyMatrix4x4::MakeAffinMatrix({ 1.0f,1.0f,1.0f }, { 0.0f,0.0f,0.0f }, { 0.0f,0.0f,0.0f });
	MyMatrix4x4 worldViewProjectionMatrix = MyMatrix4x4::Multiply(worldMatrix, viewProjectionMatrix);

	for (uint32_t index = 0; index <= kSubdivision; index++) {

		MyVector3 xLinePosition[2] = {
			{kGridHalfWidth - kGridEvery * index,0.0f,kGridHalfWidth},
			{kGridHalfWidth - kGridEvery * index,0.0f,-kGridHalfWidth}
		};
		MyVector3 zLinePosition[2] = {
			{-kGridHalfWidth,0.0f,kGridHalfWidth - kGridEvery * index},
			{kGridHalfWidth,0.0f,kGridHalfWidth - kGridEvery * index}
		};

		MyVector3 screenXLinePosition[2] = {};
		MyVector3 screenZLinePosition[2] = {};

		for (int i = 0; i < 2; i++) {
			MyVector3 nbcXLinePosition = MyMatrix4x4::Transform(xLinePosition[i], worldViewProjectionMatrix);
			MyVector3 nbcZLinePosition = MyMatrix4x4::Transform(zLinePosition[i], worldViewProjectionMatrix);
			screenXLinePosition[i] = MyMatrix4x4::Transform(nbcXLinePosition, viewportMatrix);
			screenZLinePosition[i] = MyMatrix4x4::Transform(nbcZLinePosition, viewportMatrix);
		}

		if (xLinePosition[0].x != 0.0f) {
			Novice::DrawLine(int(screenXLinePosition[0].x), int(screenXLinePosition[0].y), int(screenXLinePosition[1].x), int(screenXLinePosition[1].y), 0xAAAAAAFF);
			Novice::DrawLine(int(screenZLinePosition[0].x), int(screenZLinePosition[0].y), int(screenZLinePosition[1].x), int(screenZLinePosition[1].y), 0xAAAAAAFF);
		}
		else {
			Novice::DrawLine(int(screenXLinePosition[0].x), int(screenXLinePosition[0].y), int(screenXLinePosition[1].x), int(screenXLinePosition[1].y), 0x000000FF);
			Novice::DrawLine(int(screenZLinePosition[0].x), int(screenZLinePosition[0].y), int(screenZLinePosition[1].x), int(screenZLinePosition[1].y), 0x000000FF);
		}
	}
}
