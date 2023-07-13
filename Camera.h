#pragma once
#include"MyVector3.h"
#include"MyVector2.h"
class Camera
{
public:

	void Initialize(const MyVector3& translate, const MyVector3& rotate);
	void Update(char* keys, char* preKeys);

	MyVector3 GetTranslate() { return translate_; }
	MyVector3 GetRotate() { return rotate_; }
	MyVector3 GetScale() { return scale_; }

private:
	MyVector3 translate_ = {};
	MyVector3 scale_ = {};
	MyVector3 rotate_ = {};


	MyVector3 subRotate_ = {};
	MyVector3 subTranslate_ = {};
	MyVector2 mousePos_ = {};

	bool isTranslate_ = false;
	bool isRotate_ = false;
	bool isRotateTranslate_ = false;
	bool isDebug_ = false;
};

