#include"Plane.h"
#include"Novice.h"
#include"calc.h"

void Plane::Initialize() {

}

void Plane::Update() {

}

void Plane::Draw(const Plane& plane, const MyMatrix4x4& viewProjectionMatrix, const MyMatrix4x4& viewportMatrix, uint32_t color) {
	MyVector3 center = plane.normal;
	center *= plane.distance;

	MyVector3 perpendiculars[4];

	perpendiculars[0] = Calc::Normalize(Calc::Perpendicular(plane.normal));
	perpendiculars[1] = { -perpendiculars[0].x,-perpendiculars[0].y, -perpendiculars[0].z };
	perpendiculars[2] = Calc::Cross(plane.normal, perpendiculars[0]);
	perpendiculars[3] = { -perpendiculars[2].x,-perpendiculars[2].y,-perpendiculars[2].z };

	MyVector3 points[4];
	for (int32_t index = 0; index < 4; index++) {
		MyVector3 extend = perpendiculars[index] * 2.0f;
		MyVector3 point = center + extend;
		points[index] = MyMatrix4x4::Transform(MyMatrix4x4::Transform(point, viewProjectionMatrix), viewportMatrix);
	}

	Novice::DrawLine(int(points[0].x), int(points[0].y), int(points[2].x), int(points[2].y), color);
	Novice::DrawLine(int(points[0].x), int(points[0].y), int(points[3].x), int(points[3].y), color);
	Novice::DrawLine(int(points[1].x), int(points[1].y), int(points[2].x), int(points[2].y), color);
	Novice::DrawLine(int(points[1].x), int(points[1].y), int(points[3].x), int(points[3].y), color);

}