#include "MatrixScreenPrintf.h"
#include<Novice.h>

static const int kRowHeight = 20;
static const int kColumnWidth = 60;

void MatrixScreenPrintf(int x, int y, const MyMatrix3x3& matrix, const char* text) {
	Novice::ScreenPrintf(x, y, "%s", text);
	for (int row = 0; row < 4; row++) {
		for (int column = 0; column < 4; column++) {
			Novice::ScreenPrintf(x + column * kColumnWidth, y + row * kRowHeight + kRowHeight,
				"%6.02f", matrix.m[row][column]);
		}
	}
}

void MatrixScreenPrintf(int x, int y, const MyMatrix4x4& matrix, const char* text) {
	Novice::ScreenPrintf(x, y,"%s", text);
	for (int row = 0; row < 4; row++) {
		for (int column = 0; column < 4; column++) {
			Novice::ScreenPrintf(x + column * kColumnWidth, y + row * kRowHeight + kRowHeight,
				"%6.02f", matrix.m[row][column]);
		}
	}
}
