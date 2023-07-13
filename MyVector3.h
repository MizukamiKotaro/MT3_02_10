#pragma once
class MyVector3
{
public:

	MyVector3() { x = 0; y = 0; z = 0; }
	MyVector3(float x, float y, float z) { this->x = x; this->y = y; this->z = z; }

	MyVector3(const MyVector3& obj) {
		x = obj.x;
		y = obj.y;
		z = obj.z;
	}

	bool operator==(MyVector3 obj) {
		if (x == obj.x && y == obj.y && z == obj.z) {
			return true;
		}
		return false;
	}

	//変換
	MyVector3& operator=(MyVector3 obj) {
		x = obj.x;
		y = obj.y;
		z = obj.z;
		return *this;
	}

	//加算
	MyVector3 operator+(MyVector3 obj) {
		MyVector3 tmp;
		tmp.x = this->x + obj.x;
		tmp.y = this->y + obj.y;
		tmp.z = this->z + obj.z;
		return tmp;
	}
	void operator+=(MyVector3 obj) {
		this->x = this->x + obj.x;
		this->y = this->y + obj.y;
		this->z = this->z + obj.z;
	}

	//減算
	MyVector3 operator-(MyVector3 obj) {
		MyVector3 tmp;
		tmp.x = this->x - obj.x;
		tmp.y = this->y - obj.y;
		tmp.z = this->z - obj.z;
		return tmp;
	}
	void  operator-=(MyVector3 obj) {
		this->x -= obj.x;
		this->y -= obj.y;
		this->z -= obj.z;
	}

	//スカラー

	//乗算
	MyVector3 operator*(float a) {
		MyVector3 tmp;

		tmp.x = this->x * a;
		tmp.y = this->y * a;
		tmp.z = this->z * a;
		return tmp;
	}
	void operator*=(float a) {
		this->x *= a;
		this->y *= a;
		this->z *= a;
	}

	//除算
	MyVector3 operator/(float a) {
		MyVector3 tmp;

		tmp.x = this->x / a;
		tmp.y = this->y / a;
		tmp.z = this->z / a;
		return tmp;
	}
	void operator/=(float a) {
		this->x /= a;
		this->y /= a;
		this->z /= a;
	}

public:
	float x;
	float y;
	float z;
};

