#pragma once
#include "MyVector3.h"

struct Line
{
	MyVector3 origin; // 始点
	MyVector3 diff; // 終点への差分ベクトル
};

struct Ray
{
	MyVector3 origin; // 始点
	MyVector3 diff; // 終点への差分ベクトル
};

struct Segment
{
	MyVector3 origin; // 始点
	MyVector3 diff; // 終点への差分ベクトル
};