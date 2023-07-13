#include "calc.h"
#include <math.h>

MyVector3 Calc::Perpendicular(const MyVector3& vector) {
	if (vector.x != 0.0f || vector.y != 0.0f) {
		return { -vector.y,vector.x,0.0f };
	}
	return { 0.0f,-vector.z,vector.y };
}

float Calc::MakeLength(const MyVector2& v) {
	float result = static_cast<float>(sqrt(powf(v.x, 2) + powf(v.y, 2)));
	return result;
}
float Calc::MakeLength(const MyVector3& v) {
	float result = static_cast<float>(sqrt(powf(v.x, 2) + powf(v.y, 2) + powf(v.z, 2)));
	return result;
}
float Calc::MakeLength(const MyVector2& v1, const MyVector2& v2) {
	float result = static_cast<float>(sqrt(powf(v1.x - v2.x, 2) + powf(v1.y - v2.y, 2)));
	return result;
}
float Calc::MakeLength(const MyVector3& v1, const MyVector3& v2) {
	float result = static_cast<float>(sqrt(powf(v1.x - v2.x, 2) + powf(v1.y - v2.y, 2) + powf(v1.z - v2.z, 2)));
	return result;
}

float Calc::MakeLength(const MyVector2& v1, const MyVector2& v2, const MyVector2& v3) {
	MyVector2 lineV[2] = { v3,v1 };
	lineV[0] -= v2;
	lineV[1] -= v2;

	float dot = Dot(lineV[0], lineV[1]);

	float lineLength = MakeLength(v2, v3);

	float t = dot / lineLength / lineLength;

	MyVector2 pos = {};
	if (t < 0) {
		pos = v2;
	}
	else if (t > 1) {
		pos = v3;
	}
	else {
		pos = lineV[0] * t + v2;
	}

	float length = MakeLength(v1, pos);
	return length;
}

float Calc::MakeLength(const MyVector3& v1, const MyVector3& v2, const MyVector3& v3) {
	MyVector3 lineV[2] = { v3,v1 };
	lineV[0] -= v2;
	lineV[1] -= v2;

	float dot = Dot(lineV[0], lineV[1]);

	float lineLength = MakeLength(v2, v3);

	float t = dot / lineLength / lineLength;

	MyVector3 pos = {};
	if (t < 0) {
		pos = v2;
	}
	else if (t > 1) {
		pos = v3;
	}
	else {
		pos = lineV[0] * t + v2;
	}

	float length = MakeLength(v1, pos);
	return length;
}

float Calc::MakeLength(const MyVector2& v1, const MyVector2& v2, const MyVector2& v3, LineType linetype) {
	MyVector2 lineV[2] = { v3,v1 };
	lineV[0] -= v2;
	lineV[1] -= v2;

	float dot = Dot(lineV[0], lineV[1]);

	float lineLength = MakeLength(v2, v3);

	float t = dot / lineLength / lineLength;

	MyVector2 pos = {};

	switch (linetype)
	{
	case Calc::LineType::Line:
		pos = lineV[0] * t + v2;
		break;
	case Calc::LineType::Ray:
		if (t < 0) {
			pos = v2;
		}
		else {
			pos = lineV[0] * t + v2;
		}
		break;
	case Calc::LineType::Segment:
		if (t < 0) {
			pos = v2;
		}
		else if (t > 1) {
			pos = v3;
		}
		else {
			pos = lineV[0] * t + v2;
		}
		break;
	default:
		break;
	}

	float length = MakeLength(v1, pos);
	return length;
}

float Calc::MakeLength(const MyVector3& v1, const MyVector3& v2, const MyVector3& v3, LineType linetype) {
	MyVector3 lineV[2] = { v3,v1 };
	lineV[0] -= v2;
	lineV[1] -= v2;

	float dot = Dot(lineV[0], lineV[1]);

	float lineLength = MakeLength(v2, v3);

	float t = dot / lineLength / lineLength;

	MyVector3 pos = {};

	switch (linetype)
	{
	case Calc::LineType::Line:
		pos = lineV[0] * t + v2;
		break;
	case Calc::LineType::Ray:
		if (t < 0) {
			pos = v2;
		}
		else {
			pos = lineV[0] * t + v2;
		}
		break;
	case Calc::LineType::Segment:
		if (t < 0) {
			pos = v2;
		}
		else if (t > 1) {
			pos = v3;
		}
		else {
			pos = lineV[0] * t + v2;
		}
		break;
	default:
		break;
	}

	float length = MakeLength(v1, pos);
	return length;
}

MyVector3 Calc::Project(const MyVector3& v1, const MyVector3& v2) {
	float dot = Dot(v1, v2);
	float length = MakeLength(v2);

	float t = dot / length / length;

	MyVector3 result = v2;
	result *= t;
	return result;
}

MyVector3 Calc::ClosestPoint(const MyVector3& point, const Segment& segment) {
	MyVector3 lineV = point;
	lineV -= segment.origin;

	float dot = Dot(lineV, segment.diff);

	float lineLength = MakeLength(segment.diff);

	float t = dot / lineLength / lineLength;

	MyVector3 pos = segment.origin;

	if (t > 1) {
		pos += segment.diff;
	}
	else if(t > 0 ) {
		MyVector3 project = segment.diff;
		pos += project * t;
	}

	return pos;
}

MyVector3 Calc::ClosestPoint(const MyVector3& point, const Ray& ray) {
	MyVector3 lineV = point;
	lineV -= ray.origin;

	float dot = Dot(lineV, ray.diff);

	float lineLength = MakeLength(ray.diff);

	float t = dot / lineLength / lineLength;

	MyVector3 pos = ray.origin;

	if (t > 0) {
		MyVector3 project = ray.diff;
		pos += project * t;
	}

	return pos;
}

MyVector3 Calc::ClosestPoint(const MyVector3& point, const Line& line) {
	MyVector3 lineV = point;
	lineV -= line.origin;

	float dot = Dot(lineV, line.diff);

	float lineLength = MakeLength(line.diff);

	float t = dot / lineLength / lineLength;

	MyVector3 pos = line.origin;

	MyVector3 project = line.diff;
	pos += project * t;

	return pos;
}

float Calc::MakeT(const MyVector3& point, const Segment& segment) {
	MyVector3 lineV = point;
	lineV -= segment.origin;

	float dot = Dot(lineV, segment.diff);

	float lineLength = MakeLength(segment.diff);

	float t = dot / lineLength / lineLength;

	return t;
}

float Calc::MakeT(const MyVector3& point, const Ray& ray) {
	MyVector3 lineV = point;
	lineV -= ray.origin;

	float dot = Dot(lineV, ray.diff);

	float lineLength = MakeLength(ray.diff);

	float t = dot / lineLength / lineLength;

	return t;
}

float Calc::MakeT(const MyVector3& point, const Line& line) {
	MyVector3 lineV = point;
	lineV -= line.origin;

	float dot = Dot(lineV, line.diff);

	float lineLength = MakeLength(line.diff);

	float t = dot / lineLength / lineLength;

	return t;
}

float Calc::MakeLength(const Sphere& sphere, const Plane& plane) {
	MyVector3 pos = plane.normal;
	pos *= plane.distance;
	return MakeLength(Project(pos - sphere.center_, plane.normal));
}

float Calc::MakeLength(const MyVector3& position, const Plane& plane) {
	MyVector3 pos = plane.normal;
	pos *= plane.distance;
	return MakeLength(Project(pos - position, plane.normal));
}


MyVector2 Calc::Normalize(const MyVector2& v) {
	float len = MakeLength(v);

	MyVector2 result = {};

	if (len != 0) {
		result = { v.x / len,v.y / len };
	}

	return result;
}

MyVector3 Calc::Normalize(const MyVector3& v) {
	float len = MakeLength(v);

	MyVector3 result = {};

	if (len != 0) {
		result = { v.x / len, v.y / len, v.z / len };
	}

	return result;
}

float Calc::Dot(const MyVector2& v1, const MyVector2& v2) {
	float dot = v1.x * v2.x + v1.y * v2.y;
	return dot;
}

float Calc::Dot(const MyVector3& v1, const MyVector3& v2) {
	float dot = v1.x * v2.x + v1.y * v2.y + v1.z * v2.z;
	return dot;
}

float Calc::Outer(const MyVector2& v1, const MyVector2& v2) {
	float outer = v1.x * v2.y - v1.y * v2.x;
	return outer;
}

MyVector3 Calc::Cross(const MyVector3& v1, const MyVector3& v2) {
	MyVector3 result = { v1.y * v2.z - v1.z * v2.y,v1.z * v2.x - v1.x * v2.z ,v1.x * v2.y - v1.y * v2.x };
	return result;
}
