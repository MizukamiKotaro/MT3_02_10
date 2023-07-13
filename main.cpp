#include <Novice.h>
#include"MyMatrix4x4.h"
#include"MatrixScreenPrintf.h"
#include"MyVector3.h"
#include"VectorScreenPrintf.h"
#include"calc.h"
#include"Grid.h"
#include"Sphere.h"
#include<imgui.h>
#include"Line.h"
#include"Collision.h"
#include"Camera.h"
#include"Plane.h"
#include"Triangle.h"
#include"AABB.h"
#include"OBB.h"

const char kWindowTitle[] = "学籍番号";

static const int kRowHeight = 20;
static const int kColumnWidth = 60;

const int kWindowWidth = 1280;
const int kWindowHeight = 720;

// Windowsアプリでのエントリーポイント(main関数)
int WINAPI WinMain(_In_ HINSTANCE, _In_opt_ HINSTANCE, _In_ LPSTR lpCmdLine, _In_ int nShowCmd) {

	// ライブラリの初期化
	Novice::Initialize(kWindowTitle, kWindowWidth, kWindowHeight);

	// キー入力結果を受け取る箱
	char keys[256] = {0};
	char preKeys[256] = {0};

	Camera* camera = new Camera();
	camera->Initialize({ 0.0f,1.9f,-6.49f }, { 0.26f,0.0f,0.0f });

	MyMatrix4x4 originMatrix = MyMatrix4x4::MakeAffinMatrix({ 1.0f,1.0f,1.0f }, { 0.0f,0.0f,0.0f }, { 0.0f,0.0f,0.0f });
	
	OBB a = {
		{0.0f,0.0f,0.0f},
		{{1.0f,0.0f,0.0f},
		{0.0f,1.0f,0.0f},
		{0.0f,0.0f,1.0f}},
		{0.83f,0.26f,0.24f}
	};

	MyVector3 rotate0 = {};

	OBB b = {
		{-1.0f,0.0f,0.0f},
		{{1.0f,0.0f,0.0f},
		{0.0f,1.0f,0.0f},
		{0.0f,0.0f,1.0f}},
		{0.5f,0.37f,0.5f}
	};
	MyVector3 rotate1 = { -0.05f,2.49f,0.15f };

	MyMatrix4x4 projectionMatrix = MyMatrix4x4::MakePerspectiveFovMatrix(0.45f, float(kWindowWidth) / float(kWindowHeight), 0.1f, 100.0f);
	MyMatrix4x4 viewportMatrix = MyMatrix4x4::MakeViewportMatrix(0.0f, 0.0f, float(kWindowWidth), float(kWindowHeight), 0.0f, 1.0f);

	// ウィンドウの×ボタンが押されるまでループ
	while (Novice::ProcessMessage() == 0) {
		// フレームの開始
		Novice::BeginFrame();

		// キー入力を受け取る
		memcpy(preKeys, keys, 256);
		Novice::GetHitKeyStateAll(keys);

		///
		/// ↓更新処理ここから
		///
		
		ImGui::Begin("Window");
		ImGui::Text("R : Reset");
		ImGui::DragFloat3("obb1.center", &a.center.x, 0.01f);
		ImGui::SliderFloat3("obb1Rotate", &rotate0.x, -3.14f, 3.14f);
		ImGui::DragFloat3("obb1.size", &a.size.x, 0.01f);
		ImGui::DragFloat3("obb2.center", &b.center.x, 0.01f);
		ImGui::SliderFloat3("obb2Rotate", &rotate1.x, -3.14f, 3.14f);
		ImGui::DragFloat3("obb2.size", &b.size.x, 0.01f);
		ImGui::End();

		a.SetOrientations(MyMatrix4x4::MakeRotateXYZMatrix(rotate0));
		b.SetOrientations(MyMatrix4x4::MakeRotateXYZMatrix(rotate1));
		
		if (keys[DIK_R]) {
			a = {
				{0.0f,0.0f,0.0f},
				{{1.0f,0.0f,0.0f},
				{0.0f,1.0f,0.0f},
				{0.0f,0.0f,1.0f}},
				{0.83f,0.26f,0.24f}
			};

			rotate0 = {};

			b = {
				{-1.0f,0.0f,0.0f},
				{{1.0f,0.0f,0.0f},
				{0.0f,1.0f,0.0f},
				{0.0f,0.0f,1.0f}},
				{0.5f,0.37f,0.5f}
			};
			rotate1 = { -0.05f,2.49f,0.15f };

			camera->Initialize({ 0.0f,1.9f,-6.49f }, { 0.26f,0.0f,0.0f });
		}

		camera->Update(keys, preKeys);

		MyMatrix4x4 cameraMatrix = MyMatrix4x4::MakeAffinMatrix(camera->GetScale(), camera->GetRotate(), camera->GetTranslate());
		MyMatrix4x4 viewMatrix = MyMatrix4x4::Inverse(cameraMatrix);
		MyMatrix4x4 viewProjectionMatrix = MyMatrix4x4::Multiply(viewMatrix, projectionMatrix);

		///
		/// ↑更新処理ここまで
		///

		///
		/// ↓描画処理ここから
		///
		


		Grid::DrawGrid(viewProjectionMatrix, viewportMatrix);

		if (Collision::IsCollision(a, b)) {
			a.Draw(viewProjectionMatrix, viewportMatrix, 0xFF0000FF);
		}
		else {
			a.Draw(viewProjectionMatrix, viewportMatrix, 0xFFFFFFFF);
		}
		
		b.Draw(viewProjectionMatrix, viewportMatrix, 0xFFFFFFFF);

		///
		/// ↑描画処理ここまで
		///

		// フレームの終了
		Novice::EndFrame();

		// ESCキーが押されたらループを抜ける
		if (preKeys[DIK_ESCAPE] == 0 && keys[DIK_ESCAPE] != 0) {
			break;
		}
	}

	delete camera;

	// ライブラリの終了
	Novice::Finalize();
	return 0;
}
