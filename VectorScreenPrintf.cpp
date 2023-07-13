#include "VectorScreenPrintf.h"
#include "Novice.h"

static const int kRowHeight = 20;
static const int kColumnWidth = 60;

void VectorScreenPrintf(int x, int y, const MyVector2& vector, const char* text) {
	Novice::ScreenPrintf(x, y, "%s", text);
	float num[2] = {};
	num[0] = vector.x;
	num[1] = vector.y;
	for (int i = 0; i < 2; i++) {
		Novice::ScreenPrintf(x + i * kColumnWidth, y + kRowHeight,"%6.02f", num[i]);
	}
}

void VectorScreenPrintf(int x, int y, const MyVector3& vector, const char* text) {
	Novice::ScreenPrintf(x, y, "%s", text);
	float num[3] = {};
	num[0] = vector.x;
	num[1] = vector.y;
	num[2] = vector.z;
	for (int i = 0; i < 3; i++) {
		Novice::ScreenPrintf(x + i * kColumnWidth, y + kRowHeight, "%6.02f", num[i]);
	}
}


