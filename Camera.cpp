#include "Camera.h"
#include"Novice.h"
#include"MyMatrix4x4.h"
#include<imgui.h>

void Camera::Initialize(const MyVector3& translate, const MyVector3& rotate) {
	translate_ = translate;
	rotate_ = rotate;
	scale_ = { 1.0f,1.0f,1.0f };
}

void Camera::Update(char* keys, char* preKeys) {
	
	ImGui::Begin("Camera");
	ImGui::Text("LShift : Debug\nLeftButton : RotateTranslate\nRightButton : Rotate\nCenterButton : Translate\nwheel : FrontRearMove");
	ImGui::Checkbox("IsDebugCamera", &isDebug_);
	ImGui::SliderFloat3("Translate", &translate_.x, -10.0f, 10.0f);
	ImGui::SliderFloat3("Rotate", &rotate_.x, -10.0f, 10.0f);
	ImGui::End();

	if (keys[DIK_LSHIFT] && !preKeys[DIK_LSHIFT]) {
		if (isDebug_) {
			isDebug_ = false;
		}
		else {
			isDebug_ = true;
		}
	}

	if (isDebug_) {
		if (Novice::IsTriggerMouse(0) && !isTranslate_ && !isRotate_) {
			int x = 0;
			int y = 0;
			Novice::GetMousePosition(&x, &y);
			mousePos_ = { float(x),float(y) };
			isRotateTranslate_ = true;
			subRotate_ = rotate_;
			subTranslate_ = translate_;
		}

		if (Novice::IsPressMouse(0) && isRotateTranslate_) {
			int x = 0;
			int y = 0;
			Novice::GetMousePosition(&x, &y);
			MyVector2 length = { x - mousePos_.x,y - mousePos_.y };

			float pi = 3.141592f;

			rotate_.y = subRotate_.y - length.x / 640 * pi;
			rotate_.x = subRotate_.x + length.y / 360 * pi;

			translate_ = MyMatrix4x4::Multiply(subTranslate_, MyMatrix4x4::MakeRotateXYZMatrix({ length.y / 360 * pi, -length.x / 640 * pi,0.0f }));

		}
		else {
			isRotateTranslate_ = false;
		}

		if (Novice::IsTriggerMouse(1) && !isTranslate_ && !isRotateTranslate_) {
			int x = 0;
			int y = 0;
			Novice::GetMousePosition(&x, &y);
			mousePos_ = { float(x),float(y) };
			isRotate_ = true;
			subRotate_ = rotate_;
		}

		if (Novice::IsPressMouse(1) && isRotate_) {
			int x = 0;
			int y = 0;
			Novice::GetMousePosition(&x, &y);
			MyVector2 length = { x - mousePos_.x,y - mousePos_.y };

			float pi = 3.141592f;


			rotate_.y = subRotate_.y + length.x / 640 * pi;
			rotate_.x = subRotate_.x + length.y / 360 * pi;
		}
		else {
			isRotate_ = false;
		}

		if (Novice::IsTriggerMouse(2) && !isRotate_ && !isRotateTranslate_) {
			int x = 0;
			int y = 0;
			Novice::GetMousePosition(&x, &y);
			mousePos_ = { float(x),float(y) };
			isTranslate_ = true;
			subTranslate_ = translate_;
		}

		if (Novice::IsPressMouse(2) && isTranslate_) {
			int x = 0;
			int y = 0;
			Novice::GetMousePosition(&x, &y);

			MyVector3 move = { -(x - mousePos_.x) / 160,(y - mousePos_.y) / 90,0.0f };
			move = MyMatrix4x4::Multiply(move, MyMatrix4x4::MakeRotateXYZMatrix(rotate_));

			translate_ = subTranslate_ + move;
		}
		else {
			isTranslate_ = false;
		}

		if (Novice::GetWheel() && !isRotate_ && !isRotateTranslate_ && !isTranslate_) {
			float moveLength = float(Novice::GetWheel()) / 120;
			MyVector3 move = { 0.0f,0.0f,1.0f };
			move = MyMatrix4x4::Multiply(move, MyMatrix4x4::MakeRotateXYZMatrix(rotate_));
			move *= moveLength;
			translate_ += move;
		}
	}
}
