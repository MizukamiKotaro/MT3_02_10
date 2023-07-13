#pragma once
class MyVector2
{
public:
	MyVector2() { x = 0; y = 0; }
	MyVector2(float x, float y) { this->x = x; this->y = y; }

	MyVector2(const MyVector2& obj) {
		x = obj.x;
		y = obj.y;
	}

	MyVector2& operator=(MyVector2 obj) {
		x = obj.x;
		y = obj.y;
		return *this;
	}

	MyVector2 operator+(MyVector2 obj) {
		MyVector2 tmp;
		tmp.x = this->x + obj.x;
		tmp.y = this->y + obj.y;
		return tmp;
	}

	void operator+=(MyVector2 obj) {
		this->x = this->x + obj.x;
		this->y = this->y + obj.y;
	}

	MyVector2 operator-(MyVector2 obj) {
		MyVector2 tmp;
		tmp.x = this->x - obj.x;
		tmp.y = this->y - obj.y;
		return tmp;
	}

	void  operator-=(MyVector2 obj) {
		this->x -= obj.x;
		this->y -= obj.y;
	}

	MyVector2 operator*(float a) {
		MyVector2 tmp;

		tmp.x = this->x * a;
		tmp.y = this->y * a;
		return tmp;
	}

	void operator*=(float a) {
		this->x *= a;
		this->y *= a;
	}

	MyVector2 operator/(float a) {
		MyVector2 tmp;

		tmp.x = this->x / a;
		tmp.y = this->y / a;
		return tmp;
	}

	void operator/=(float a) {
		this->x /= a;
		this->y /= a;
	}

public:
	float x;
	float y;
};

