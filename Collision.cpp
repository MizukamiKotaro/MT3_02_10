#include "Collision.h"
#include"calc.h"
#include"MyVector3.h"
#include<cmath>
#include<algorithm>

bool Collision::IsCollision(const Sphere& s1, const Sphere& s2) {
	float length = Calc::MakeLength(s1.center_, s2.center_);

	//スケールはいじるな！お願いします。
	if (length <= s1.radius_ + s2.radius_) {
		return true;
	}
	return false;
}

bool Collision::IsCollision(const Sphere& sphere, const Plane& plane) {
	float length = Calc::MakeLength(sphere, plane);

	if (length <= sphere.radius_) {
		if (plane.isLimit) {
			MyVector3 center = plane.normal;
			center *= plane.distance;

			MyVector3 perpendiculars[4];

			perpendiculars[0] = Calc::Normalize(Calc::Perpendicular(plane.normal));
			perpendiculars[1] = { -perpendiculars[0].x,-perpendiculars[0].y, -perpendiculars[0].z };
			perpendiculars[2] = Calc::Cross(plane.normal, perpendiculars[0]);
			perpendiculars[3] = { -perpendiculars[2].x,-perpendiculars[2].y,-perpendiculars[2].z };

			for (int32_t index = 0; index < 4; index++) {
				perpendiculars[index] = center + perpendiculars[index] * 2.0f;
			}

			MyVector3 pos = Calc::Project(center - sphere.center_, plane.normal) + sphere.center_;
			float radius = sqrtf(powf(sphere.radius_, 2.0f) - powf(length, 2.0f));

			float len;
			len = Calc::MakeLength(pos, perpendiculars[0], perpendiculars[2]);
			if (len <= radius) {
				return true;
			}
			len = Calc::MakeLength(pos, perpendiculars[0], perpendiculars[3]);
			if (len <= radius) {
				return true;
			}
			len = Calc::MakeLength(pos, perpendiculars[1], perpendiculars[2]);
			if (len <= radius) {
				return true;
			}
			len = Calc::MakeLength(pos, perpendiculars[1], perpendiculars[2]);
			if (len <= radius) {
				return true;
			}

			float dot[4] = {};

			dot[0] = Calc::Dot(plane.normal, Calc::Cross(pos - perpendiculars[2], perpendiculars[0] - pos));
			dot[1] = Calc::Dot(plane.normal, Calc::Cross(pos - perpendiculars[1], perpendiculars[2] - pos));
			dot[2] = Calc::Dot(plane.normal, Calc::Cross(pos - perpendiculars[3], perpendiculars[1] - pos));
			dot[3] = Calc::Dot(plane.normal, Calc::Cross(pos - perpendiculars[0], perpendiculars[3] - pos));

			if ((dot[0] >= 0 && dot[1] >= 0 && dot[2] >= 0 && dot[3] >= 0) || (dot[0] <= 0 && dot[1] <= 0 && dot[2] <= 0 && dot[3] <= 0)) {
				return true;
			}
		}
		else {
			return true;
		}
	}
	return false;
}

bool Collision::IsCollision(const Segment& segment, const Plane& plane) {

	if (Calc::Dot(segment.diff, plane.normal) == 0) {

		MyVector3 pos = plane.normal;
		pos *= plane.distance;

		Line tmpLine = { segment.origin,plane.normal };

		pos = Calc::ClosestPoint(pos, tmpLine);

		if (Calc::MakeLength(pos, segment.origin) == 0) {

			return true;
		}
		return false;
	}

	float t = (plane.distance - Calc::Dot(segment.origin, plane.normal)) / Calc::Dot(segment.diff, plane.normal);

	if (t >= 0 && t <= 1) {
		if (plane.isLimit) {
			return IsInPlane(segment, plane);
		}

		return true;
	}
	return false;

}

bool Collision::IsCollision(const Ray& ray, const Plane& plane) {
	if (Calc::Dot(ray.diff, plane.normal) == 0) {
		MyVector3 pos = plane.normal;
		pos *= plane.distance;

		Line tmpLine = { ray.origin,plane.normal };

		pos = Calc::ClosestPoint(pos, tmpLine);

		if (Calc::MakeLength(pos, ray.origin) == 0) {

			return true;
		}
		return false;
	}

	float t = (plane.distance - Calc::Dot(ray.origin, plane.normal)) / Calc::Dot(ray.diff, plane.normal);

	if (t >= 0) {
		if (plane.isLimit) {
			return IsInPlane(ray, plane);
		}

		return true;
	}
	return false;

}

bool Collision::IsCollision(const Line& line, const Plane& plane) {

	if (Calc::Dot(line.diff, plane.normal) == 0) {
		MyVector3 pos = plane.normal;
		pos *= plane.distance;

		Line tmpLine = { line.origin,plane.normal };

		pos = Calc::ClosestPoint(pos, tmpLine);

		if (Calc::MakeLength(pos, line.origin) == 0) {

			return true;
		}
		return false;
	}

	if (plane.isLimit) {
		return IsInPlane(line, plane);
	}

	return true;
}

bool Collision::IsInPlane(const Segment& segment, const Plane& plane) {

	MyVector3 pos = plane.normal;
	pos *= plane.distance;

	MyVector3 perpendiculars[4];

	perpendiculars[0] = Calc::Normalize(Calc::Perpendicular(plane.normal));
	perpendiculars[1] = { -perpendiculars[0].x,-perpendiculars[0].y, -perpendiculars[0].z };
	perpendiculars[2] = Calc::Cross(plane.normal, perpendiculars[0]);
	perpendiculars[3] = { -perpendiculars[2].x,-perpendiculars[2].y,-perpendiculars[2].z };

	for (int32_t index = 0; index < 4; index++) {
		perpendiculars[index] = pos + perpendiculars[index] * 2.0f;
	}

	float t = (plane.distance - Calc::Dot(segment.origin, plane.normal)) / Calc::Dot(segment.diff, plane.normal);

	pos = MyVector3(segment.origin) + MyVector3(segment.diff) * t;

	float dot[4] = {};

	dot[0] = Calc::Dot(plane.normal, Calc::Cross(pos - perpendiculars[2], perpendiculars[0] - pos));
	dot[1] = Calc::Dot(plane.normal, Calc::Cross(pos - perpendiculars[1], perpendiculars[2] - pos));
	dot[2] = Calc::Dot(plane.normal, Calc::Cross(pos - perpendiculars[3], perpendiculars[1] - pos));
	dot[3] = Calc::Dot(plane.normal, Calc::Cross(pos - perpendiculars[0], perpendiculars[3] - pos));

	if ((dot[0] >= 0 && dot[1] >= 0 && dot[2] >= 0 && dot[3] >= 0) || (dot[0] <= 0 && dot[1] <= 0 && dot[2] <= 0 && dot[3] <= 0)) {
		return true;
	}

	return false;
}

bool Collision::IsInPlane(const Ray& ray, const Plane& plane) {

	MyVector3 pos = plane.normal;
	pos *= plane.distance;

	MyVector3 perpendiculars[4];

	perpendiculars[0] = Calc::Normalize(Calc::Perpendicular(plane.normal));
	perpendiculars[1] = { -perpendiculars[0].x,-perpendiculars[0].y, -perpendiculars[0].z };
	perpendiculars[2] = Calc::Cross(plane.normal, perpendiculars[0]);
	perpendiculars[3] = { -perpendiculars[2].x,-perpendiculars[2].y,-perpendiculars[2].z };

	for (int32_t index = 0; index < 4; index++) {
		perpendiculars[index] = pos + perpendiculars[index] * 2.0f;
	}

	float t = (plane.distance - Calc::Dot(ray.origin, plane.normal)) / Calc::Dot(ray.diff, plane.normal);

	pos = MyVector3(ray.origin) + MyVector3(ray.diff) * t;

	float dot[4] = {};

	dot[0] = Calc::Dot(plane.normal, Calc::Cross(pos - perpendiculars[2], perpendiculars[0] - pos));
	dot[1] = Calc::Dot(plane.normal, Calc::Cross(pos - perpendiculars[1], perpendiculars[2] - pos));
	dot[2] = Calc::Dot(plane.normal, Calc::Cross(pos - perpendiculars[3], perpendiculars[1] - pos));
	dot[3] = Calc::Dot(plane.normal, Calc::Cross(pos - perpendiculars[0], perpendiculars[3] - pos));

	if ((dot[0] >= 0 && dot[1] >= 0 && dot[2] >= 0 && dot[3] >= 0) || (dot[0] <= 0 && dot[1] <= 0 && dot[2] <= 0 && dot[3] <= 0)) {
		return true;
	}

	return false;
}

bool Collision::IsInPlane(const Line& line, const Plane& plane) {

	MyVector3 pos = plane.normal;
	pos *= plane.distance;

	MyVector3 perpendiculars[4];

	perpendiculars[0] = Calc::Normalize(Calc::Perpendicular(plane.normal));
	perpendiculars[1] = { -perpendiculars[0].x,-perpendiculars[0].y, -perpendiculars[0].z };
	perpendiculars[2] = Calc::Cross(plane.normal, perpendiculars[0]);
	perpendiculars[3] = { -perpendiculars[2].x,-perpendiculars[2].y,-perpendiculars[2].z };

	for (int32_t index = 0; index < 4; index++) {
		perpendiculars[index] = pos + perpendiculars[index] * 2.0f;
	}

	float t = (plane.distance - Calc::Dot(line.origin, plane.normal)) / Calc::Dot(line.diff, plane.normal);

	pos = MyVector3(line.origin) + MyVector3(line.diff) * t;

	float dot[4] = {};

	dot[0] = Calc::Dot(plane.normal, Calc::Cross(pos - perpendiculars[2], perpendiculars[0] - pos));
	dot[1] = Calc::Dot(plane.normal, Calc::Cross(pos - perpendiculars[1], perpendiculars[2] - pos));
	dot[2] = Calc::Dot(plane.normal, Calc::Cross(pos - perpendiculars[3], perpendiculars[1] - pos));
	dot[3] = Calc::Dot(plane.normal, Calc::Cross(pos - perpendiculars[0], perpendiculars[3] - pos));

	if ((dot[0] >= 0 && dot[1] >= 0 && dot[2] >= 0 && dot[3] >= 0) || (dot[0] <= 0 && dot[1] <= 0 && dot[2] <= 0 && dot[3] <= 0)) {
		return true;
	}

	return false;
}

bool Collision::IsCollision(const Triangle& triangle, const Segment& segment) {

	MyVector3 normal = Calc::Normalize(Calc::Cross(MyVector3(triangle.vertices[1]) - MyVector3(triangle.vertices[0]),
		MyVector3(triangle.vertices[2]) - MyVector3(triangle.vertices[1])));

	if (Calc::Dot(segment.diff, normal) == 0) {

		/*Line tmpLine = { segment.origin,normal };

		MyVector3 pos = Calc::ClosestPoint(triangle.vertices[0], tmpLine);


		if (Calc::MakeLength(pos, segment.origin) == 0) {

			return true;
		}*/
		return false;
	}

	Line tmpLine = { {},normal};

	MyVector3 closePoint = Calc::ClosestPoint(triangle.vertices[0], tmpLine);

	float distance = Calc::MakeLength(closePoint);

	if (distance != 0) {
		normal = Calc::Normalize(closePoint);
	}

	float t = (distance - Calc::Dot(segment.origin, normal)) / Calc::Dot(segment.diff, normal);

	if (t >= 0 && t <= 1) {
		MyVector3 pos = MyVector3(segment.origin) + MyVector3(segment.diff) * t;

		float dot[3] = {};

		dot[0] = Calc::Dot(normal, Calc::Cross(pos - triangle.vertices[2], MyVector3(triangle.vertices[0]) - pos));
		dot[1] = Calc::Dot(normal, Calc::Cross(pos - triangle.vertices[1], MyVector3(triangle.vertices[2]) - pos));
		dot[2] = Calc::Dot(normal, Calc::Cross(pos - triangle.vertices[0], MyVector3(triangle.vertices[1]) - pos));

		if ((dot[0] >= 0 && dot[1] >= 0 && dot[2] >= 0) || (dot[0] <= 0 && dot[1] <= 0 && dot[2] <= 0)) {
			return true;
		}
	}

	return false;
}

bool Collision::IsCollision(const Triangle& triangle, const Ray& ray) {

	MyVector3 normal = Calc::Normalize(Calc::Cross(MyVector3(triangle.vertices[1]) - MyVector3(triangle.vertices[0]),
		MyVector3(triangle.vertices[2]) - MyVector3(triangle.vertices[1])));

	if (Calc::Dot(ray.diff, normal) == 0) {

		/*Line tmpLine = { ray.origin,normal };

		MyVector3 pos = Calc::ClosestPoint(triangle.vertices[0], tmpLine);


		if (Calc::MakeLength(pos, ray.origin) == 0) {

			return true;
		}*/
		return false;
	}

	Line tmpLine = { {},normal };

	MyVector3 closePoint = Calc::ClosestPoint(triangle.vertices[0], tmpLine);

	float distance = Calc::MakeLength(closePoint);

	if (distance != 0) {
		normal = Calc::Normalize(closePoint);
	}

	float t = (distance - Calc::Dot(ray.origin, normal)) / Calc::Dot(ray.diff, normal);

	if (t >= 0) {
		MyVector3 pos = MyVector3(ray.origin) + MyVector3(ray.diff) * t;

		float dot[3] = {};

		dot[0] = Calc::Dot(normal, Calc::Cross(pos - triangle.vertices[2], MyVector3(triangle.vertices[0]) - pos));
		dot[1] = Calc::Dot(normal, Calc::Cross(pos - triangle.vertices[1], MyVector3(triangle.vertices[2]) - pos));
		dot[2] = Calc::Dot(normal, Calc::Cross(pos - triangle.vertices[0], MyVector3(triangle.vertices[1]) - pos));

		if ((dot[0] >= 0 && dot[1] >= 0 && dot[2] >= 0) || (dot[0] <= 0 && dot[1] <= 0 && dot[2] <= 0)) {
			return true;
		}
	}

	return false;
}

bool Collision::IsCollision(const Triangle& triangle, const Line& line) {

	MyVector3 normal = Calc::Normalize(Calc::Cross(MyVector3(triangle.vertices[1]) - MyVector3(triangle.vertices[0]),
		MyVector3(triangle.vertices[2]) - MyVector3(triangle.vertices[1])));

	if (Calc::Dot(line.diff, normal) == 0) {

		/*Line tmpLine = { line.origin,normal };

		MyVector3 pos = Calc::ClosestPoint(triangle.vertices[0], tmpLine);


		if (Calc::MakeLength(pos, line.origin) == 0) {

			return true;
		}*/
		return false;
	}

	Line tmpLine = { {},normal };

	MyVector3 closePoint = Calc::ClosestPoint(triangle.vertices[0], tmpLine);

	float distance = Calc::MakeLength(closePoint);

	if (distance != 0) {
		normal = Calc::Normalize(closePoint);
	}

	float t = (distance - Calc::Dot(line.origin, normal)) / Calc::Dot(line.diff, normal);

	MyVector3 pos = MyVector3(line.origin) + MyVector3(line.diff) * t;

	float dot[3] = {};

	dot[0] = Calc::Dot(normal, Calc::Cross(pos - triangle.vertices[2], MyVector3(triangle.vertices[0]) - pos));
	dot[1] = Calc::Dot(normal, Calc::Cross(pos - triangle.vertices[1], MyVector3(triangle.vertices[2]) - pos));
	dot[2] = Calc::Dot(normal, Calc::Cross(pos - triangle.vertices[0], MyVector3(triangle.vertices[1]) - pos));

	if ((dot[0] >= 0 && dot[1] >= 0 && dot[2] >= 0) || (dot[0] <= 0 && dot[1] <= 0 && dot[2] <= 0)) {
		return true;
	}

	return false;
}

bool Collision::IsCollision(const Quadrangle& quadrangle, const Segment& segment) {

	MyVector3 normal = Calc::Normalize(Calc::Cross(MyVector3(quadrangle.vertices[1]) - MyVector3(quadrangle.vertices[0]),
		MyVector3(quadrangle.vertices[2]) - MyVector3(quadrangle.vertices[1])));

	if (Calc::Dot(segment.diff, normal) == 0) {

		/*Line tmpLine = { segment.origin,normal };

		MyVector3 pos = Calc::ClosestPoint(triangle.vertices[0], tmpLine);


		if (Calc::MakeLength(pos, segment.origin) == 0) {

			return true;
		}*/
		return false;
	}


	Line tmpLine = { {},normal };

	MyVector3 closePoint = Calc::ClosestPoint(quadrangle.vertices[0], tmpLine);

	float distance = Calc::MakeLength(closePoint);

	if (distance != 0) {
		normal = Calc::Normalize(closePoint);
	}

	float t = (distance - Calc::Dot(segment.origin, normal)) / Calc::Dot(segment.diff, normal);

	if (t >= 0 && t <= 1) {
		MyVector3 pos = MyVector3(segment.origin) + MyVector3(segment.diff) * t;

		float dot[4] = {};

		dot[0] = Calc::Dot(normal, Calc::Cross(pos - quadrangle.vertices[1], MyVector3(quadrangle.vertices[0]) - pos));
		dot[1] = Calc::Dot(normal, Calc::Cross(pos - quadrangle.vertices[2], MyVector3(quadrangle.vertices[1]) - pos));
		dot[2] = Calc::Dot(normal, Calc::Cross(pos - quadrangle.vertices[3], MyVector3(quadrangle.vertices[2]) - pos));
		dot[3] = Calc::Dot(normal, Calc::Cross(pos - quadrangle.vertices[0], MyVector3(quadrangle.vertices[3]) - pos));

		if ((dot[0] >= 0 && dot[1] >= 0 && dot[2] >= 0 && dot[3] >= 0) || (dot[0] <= 0 && dot[1] <= 0 && dot[2] <= 0 && dot[3] <= 0)) {
			return true;
		}
	}

	return false;
}

bool Collision::IsCollision(const Quadrangle& quadrangle, const Ray& ray) {

	MyVector3 normal = Calc::Normalize(Calc::Cross(MyVector3(quadrangle.vertices[1]) - MyVector3(quadrangle.vertices[0]),
		MyVector3(quadrangle.vertices[2]) - MyVector3(quadrangle.vertices[1])));

	if (Calc::Dot(ray.diff, normal) == 0) {

		/*Line tmpLine = { ray.origin,normal };

		MyVector3 pos = Calc::ClosestPoint(triangle.vertices[0], tmpLine);


		if (Calc::MakeLength(pos, ray.origin) == 0) {

			return true;
		}*/
		return false;
	}

	Line tmpLine = { {},normal };

	MyVector3 closePoint = Calc::ClosestPoint(quadrangle.vertices[0], tmpLine);

	float distance = Calc::MakeLength(closePoint);

	if (distance != 0) {
		normal = Calc::Normalize(closePoint);
	}

	float t = (distance - Calc::Dot(ray.origin, normal)) / Calc::Dot(ray.diff, normal);

	if (t >= 0) {
		MyVector3 pos = MyVector3(ray.origin) + MyVector3(ray.diff) * t;

		float dot[4] = {};

		dot[0] = Calc::Dot(normal, Calc::Cross(pos - quadrangle.vertices[1], MyVector3(quadrangle.vertices[0]) - pos));
		dot[1] = Calc::Dot(normal, Calc::Cross(pos - quadrangle.vertices[2], MyVector3(quadrangle.vertices[1]) - pos));
		dot[2] = Calc::Dot(normal, Calc::Cross(pos - quadrangle.vertices[3], MyVector3(quadrangle.vertices[2]) - pos));
		dot[3] = Calc::Dot(normal, Calc::Cross(pos - quadrangle.vertices[0], MyVector3(quadrangle.vertices[3]) - pos));

		if ((dot[0] >= 0 && dot[1] >= 0 && dot[2] >= 0 && dot[3] >= 0) || (dot[0] <= 0 && dot[1] <= 0 && dot[2] <= 0 && dot[3] <= 0)) {
			return true;
		}
	}

	return false;
}

bool Collision::IsCollision(const Quadrangle& quadrangle, const Line& line) {

	MyVector3 normal = Calc::Normalize(Calc::Cross(MyVector3(quadrangle.vertices[1]) - MyVector3(quadrangle.vertices[0]),
		MyVector3(quadrangle.vertices[2]) - MyVector3(quadrangle.vertices[1])));

	if (Calc::Dot(line.diff, normal) == 0) {

		/*Line tmpLine = { line.origin,normal };

		MyVector3 pos = Calc::ClosestPoint(triangle.vertices[0], tmpLine);


		if (Calc::MakeLength(pos, line.origin) == 0) {

			return true;
		}*/
		return false;
	}

	Line tmpLine = { {},normal };

	MyVector3 closePoint = Calc::ClosestPoint(quadrangle.vertices[0], tmpLine);

	float distance = Calc::MakeLength(closePoint);

	if (distance != 0) {
		normal = Calc::Normalize(closePoint);
	}

	float t = (distance - Calc::Dot(line.origin, normal)) / Calc::Dot(line.diff, normal);

	MyVector3 pos = MyVector3(line.origin) + MyVector3(line.diff) * t;

	float dot[4] = {};

	dot[0] = Calc::Dot(normal, Calc::Cross(pos - quadrangle.vertices[1], MyVector3(quadrangle.vertices[0]) - pos));
	dot[1] = Calc::Dot(normal, Calc::Cross(pos - quadrangle.vertices[2], MyVector3(quadrangle.vertices[1]) - pos));
	dot[2] = Calc::Dot(normal, Calc::Cross(pos - quadrangle.vertices[3], MyVector3(quadrangle.vertices[2]) - pos));
	dot[3] = Calc::Dot(normal, Calc::Cross(pos - quadrangle.vertices[0], MyVector3(quadrangle.vertices[3]) - pos));

	if ((dot[0] >= 0 && dot[1] >= 0 && dot[2] >= 0 && dot[3] >= 0) || (dot[0] <= 0 && dot[1] <= 0 && dot[2] <= 0 && dot[3] <= 0)) {
		return true;
	}

	return false;
}

bool Collision::IsCollision(const AABB& a, const AABB& b) {

	if ((a.min.x <= b.max.x && a.max.x >= b.min.x) &&
		(a.min.y <= b.max.y && a.max.y >= b.min.y) &&
		(a.min.z <= b.max.z && a.max.z >= b.min.z)) {
		return true;
	}
	return false;
}

bool Collision::IsCollision(const AABB& a, const Sphere& sphere) {
	MyVector3 closestPoint = { std::clamp(sphere.center_.x,a.min.x,a.max.x),
		std::clamp(sphere.center_.y,a.min.y,a.max.y),
		std::clamp(sphere.center_.z,a.min.z,a.max.z) };

	float distance = Calc::MakeLength(closestPoint, sphere.center_);

	if (distance <= sphere.radius_) {
		return true;
	}

	return false;
}

bool Collision::IsCollision(const AABB& a, const Segment& segment) {

	AABB b = a;

	if (a.max.x < a.min.x) {
		b.min.x = a.max.x;
		b.max.x = a.min.x;
	}
	if (a.max.y < a.min.y) {
		b.min.y = a.max.y;
		b.max.y = a.min.y;
	}
	if (a.max.z < a.min.z) {
		b.min.z = a.max.z;
		b.max.z = a.min.z;
	}

	if (((b.min.x <= segment.origin.x && b.max.x >= segment.origin.x) &&
		(b.min.y <= segment.origin.y && b.max.y >= segment.origin.y) &&
		(b.min.z <= segment.origin.z && b.max.z >= segment.origin.z)) ||
		((b.min.x <= segment.origin.x + segment.diff.x && b.max.x >= segment.origin.x + segment.diff.x) &&
		(b.min.y <= segment.origin.y + segment.diff.y && b.max.y >= segment.origin.y + segment.diff.y) &&
		(b.min.z <= segment.origin.z + segment.diff.z && b.max.z >= segment.origin.z + segment.diff.z))) {
		return true;
	}

	MyVector3 radius = (MyVector3(b.max) - b.min) / 2.0f;

	MyVector3 center = radius + b.min;

	MyVector3 verteces[8] = {
		{-radius.x,radius.y,-radius.z},
		{radius.x,radius.y,-radius.z},
		{radius.x,radius.y,radius.z},
		{-radius.x,radius.y,radius.z},
		{-radius.x,-radius.y,-radius.z},
		{radius.x,-radius.y,-radius.z},
		{radius.x,-radius.y,radius.z},
		{-radius.x,-radius.y,radius.z},
	};

	for (uint32_t index = 0; index < 8; index++) {
		verteces[index] += center;
	}

	Quadrangle quadrangle = { verteces[0],verteces[1] ,verteces[2] ,verteces[3] };
	if (IsCollision(quadrangle, segment)) {
		return true;
	}
	quadrangle = { verteces[4],verteces[5] ,verteces[6] ,verteces[7] };
	if (IsCollision(quadrangle, segment)) {
		return true;
	}

	quadrangle = { verteces[0],verteces[1] ,verteces[5] ,verteces[4] };
	if (IsCollision(quadrangle, segment)) {
		return true;
	}

	quadrangle = { verteces[7],verteces[6] ,verteces[2] ,verteces[3] };
	if (IsCollision(quadrangle, segment)) {
		return true;
	}

	quadrangle = { verteces[5],verteces[1] ,verteces[2] ,verteces[6] };
	if (IsCollision(quadrangle, segment)) {
		return true;
	}

	quadrangle = { verteces[0],verteces[4] ,verteces[7] ,verteces[3] };
	if (IsCollision(quadrangle, segment)) {
		return true;
	}

	return false;

}

bool Collision::IsCollision(const AABB& a, const Ray& ray) {

	AABB b = a;

	if (a.max.x < a.min.x) {
		b.min.x = a.max.x;
		b.max.x = a.min.x;
	}
	if (a.max.y < a.min.y) {
		b.min.y = a.max.y;
		b.max.y = a.min.y;
	}
	if (a.max.z < a.min.z) {
		b.min.z = a.max.z;
		b.max.z = a.min.z;
	}

	if ((b.min.x <= ray.origin.x && b.max.x >= ray.origin.x) &&
		(b.min.y <= ray.origin.y && b.max.y >= ray.origin.y) &&
		(b.min.z <= ray.origin.z && b.max.z >= ray.origin.z)) {
		return true;
	}

	MyVector3 radius = (MyVector3(b.max) - b.min) / 2.0f;

	MyVector3 center = radius + b.min;

	MyVector3 verteces[8] = {
		{-radius.x,radius.y,-radius.z},
		{radius.x,radius.y,-radius.z},
		{radius.x,radius.y,radius.z},
		{-radius.x,radius.y,radius.z},
		{-radius.x,-radius.y,-radius.z},
		{radius.x,-radius.y,-radius.z},
		{radius.x,-radius.y,radius.z},
		{-radius.x,-radius.y,radius.z},
	};

	for (uint32_t index = 0; index < 8; index++) {
		verteces[index] += center;
	}

	Quadrangle quadrangle = { verteces[0],verteces[1] ,verteces[2] ,verteces[3] };
	if (IsCollision(quadrangle, ray)) {
		return true;
	}
	quadrangle = { verteces[4],verteces[5] ,verteces[6] ,verteces[7] };
	if (IsCollision(quadrangle, ray)) {
		return true;
	}

	quadrangle = { verteces[0],verteces[1] ,verteces[5] ,verteces[4] };
	if (IsCollision(quadrangle, ray)) {
		return true;
	}

	quadrangle = { verteces[7],verteces[6] ,verteces[2] ,verteces[3] };
	if (IsCollision(quadrangle, ray)) {
		return true;
	}

	quadrangle = { verteces[5],verteces[1] ,verteces[2] ,verteces[6] };
	if (IsCollision(quadrangle, ray)) {
		return true;
	}

	quadrangle = { verteces[0],verteces[4] ,verteces[7] ,verteces[3] };
	if (IsCollision(quadrangle, ray)) {
		return true;
	}

	return false;
}

bool Collision::IsCollision(const AABB& a, const Line& line) {

	MyVector3 radius = (MyVector3(a.max) - a.min) / 2.0f;

	MyVector3 center = radius + a.min;

	MyVector3 verteces[8] = {
		{-radius.x,radius.y,-radius.z},
		{radius.x,radius.y,-radius.z},
		{radius.x,radius.y,radius.z},
		{-radius.x,radius.y,radius.z},
		{-radius.x,-radius.y,-radius.z},
		{radius.x,-radius.y,-radius.z},
		{radius.x,-radius.y,radius.z},
		{-radius.x,-radius.y,radius.z},
	};

	for (uint32_t index = 0; index < 8; index++) {
		verteces[index] += center;
	}

	Quadrangle quadrangle = { verteces[0],verteces[1] ,verteces[2] ,verteces[3] };
	if (IsCollision(quadrangle, line)) {
		return true;
	}
	quadrangle = { verteces[4],verteces[5] ,verteces[6] ,verteces[7] };
	if (IsCollision(quadrangle, line)) {
		return true;
	}

	quadrangle = { verteces[0],verteces[1] ,verteces[5] ,verteces[4] };
	if (IsCollision(quadrangle, line)) {
		return true;
	}

	quadrangle = { verteces[7],verteces[6] ,verteces[2] ,verteces[3] };
	if (IsCollision(quadrangle, line)) {
		return true;
	}

	quadrangle = { verteces[5],verteces[1] ,verteces[2] ,verteces[6] };
	if (IsCollision(quadrangle, line)) {
		return true;
	}

	quadrangle = { verteces[0],verteces[4] ,verteces[7] ,verteces[3] };
	if (IsCollision(quadrangle, line)) {
		return true;
	}

	return false;
}

bool Collision::IsCollision(const OBB& a, const Sphere& sphere) {

	MyMatrix4x4 worldMat = {
		a.orientations[0].x,a.orientations[0].y,a.orientations[0].z,0,
		a.orientations[1].x,a.orientations[1].y,a.orientations[1].z,0,
		a.orientations[2].x,a.orientations[2].y,a.orientations[2].z,0,
		a.center.x,a.center.y,a.center.z,1
	};

	MyMatrix4x4 worldMatInverse = MyMatrix4x4::Inverse(worldMat);

	MyVector3 centerInOBBLocalSpase = MyMatrix4x4::Transform(sphere.center_, worldMatInverse);

	AABB aabb = { MyVector3(a.size) * (-1), a.size };
	
	Sphere b = { centerInOBBLocalSpase,sphere.scale_,sphere.rotate_,sphere.radius_ };
	
	return IsCollision(aabb, b);
}

bool Collision::IsCollision(const OBB& a, const Segment& segment) {

	MyMatrix4x4 worldMat = {
		a.orientations[0].x,a.orientations[0].y,a.orientations[0].z,0,
		a.orientations[1].x,a.orientations[1].y,a.orientations[1].z,0,
		a.orientations[2].x,a.orientations[2].y,a.orientations[2].z,0,
		a.center.x,a.center.y,a.center.z,1
	};

	MyMatrix4x4 rotateMat = {
		a.orientations[0].x,a.orientations[0].y,a.orientations[0].z,0,
		a.orientations[1].x,a.orientations[1].y,a.orientations[1].z,0,
		a.orientations[2].x,a.orientations[2].y,a.orientations[2].z,0,
		0,0,0,1
	};

	MyMatrix4x4 worldMatInverse = MyMatrix4x4::Inverse(worldMat);

	MyMatrix4x4 rotateMatInverse = MyMatrix4x4::Inverse(rotateMat);

	AABB aabb = { MyVector3(a.size) * (-1), a.size };

	Segment s = { MyMatrix4x4::Transform(segment.origin, worldMatInverse),MyMatrix4x4::Transform(segment.diff, rotateMatInverse) };

	return IsCollision(aabb, s);
}

bool Collision::IsCollision(const OBB& a, const Ray& ray) {

	MyMatrix4x4 worldMat = {
		a.orientations[0].x,a.orientations[0].y,a.orientations[0].z,0,
		a.orientations[1].x,a.orientations[1].y,a.orientations[1].z,0,
		a.orientations[2].x,a.orientations[2].y,a.orientations[2].z,0,
		a.center.x,a.center.y,a.center.z,1
	};

	MyMatrix4x4 rotateMat = {
		a.orientations[0].x,a.orientations[0].y,a.orientations[0].z,0,
		a.orientations[1].x,a.orientations[1].y,a.orientations[1].z,0,
		a.orientations[2].x,a.orientations[2].y,a.orientations[2].z,0,
		0,0,0,1
	};

	MyMatrix4x4 worldMatInverse = MyMatrix4x4::Inverse(worldMat);

	MyMatrix4x4 rotateMatInverse = MyMatrix4x4::Inverse(rotateMat);

	AABB aabb = { MyVector3(a.size) * (-1), a.size };

	Ray r = { MyMatrix4x4::Transform(ray.origin, worldMatInverse),MyMatrix4x4::Transform(ray.diff, rotateMatInverse) };

	return IsCollision(aabb, r);
}

bool Collision::IsCollision(const OBB& a, const Line& line) {

	MyMatrix4x4 worldMat = {
		a.orientations[0].x,a.orientations[0].y,a.orientations[0].z,0,
		a.orientations[1].x,a.orientations[1].y,a.orientations[1].z,0,
		a.orientations[2].x,a.orientations[2].y,a.orientations[2].z,0,
		a.center.x,a.center.y,a.center.z,1
	};

	MyMatrix4x4 rotateMat = {
		a.orientations[0].x,a.orientations[0].y,a.orientations[0].z,0,
		a.orientations[1].x,a.orientations[1].y,a.orientations[1].z,0,
		a.orientations[2].x,a.orientations[2].y,a.orientations[2].z,0,
		0,0,0,1
	};

	MyMatrix4x4 worldMatInverse = MyMatrix4x4::Inverse(worldMat);

	MyMatrix4x4 rotateMatInverse = MyMatrix4x4::Inverse(rotateMat);

	AABB aabb = { MyVector3(a.size) * (-1), a.size };

	Line l = { MyMatrix4x4::Transform(line.origin, worldMatInverse),MyMatrix4x4::Transform(line.diff, rotateMatInverse) };

	return IsCollision(aabb, l);
}

bool Collision::IsCollision(const AABB& a, const OBB& b) {

	MyMatrix4x4 worldMat = {
		b.orientations[0].x,b.orientations[0].y,b.orientations[0].z,0,
		b.orientations[1].x,b.orientations[1].y,b.orientations[1].z,0,
		b.orientations[2].x,b.orientations[2].y,b.orientations[2].z,0,
		b.center.x,b.center.y,b.center.z,1
	};

	MyVector3 radius = b.size;

	MyVector3 verteces[8] = {
		{-radius.x,radius.y,-radius.z},
		{radius.x,radius.y,-radius.z},
		{radius.x,radius.y,radius.z},
		{-radius.x,radius.y,radius.z},
		{-radius.x,-radius.y,-radius.z},
		{radius.x,-radius.y,-radius.z},
		{radius.x,-radius.y,radius.z},
		{-radius.x,-radius.y,radius.z},
	};

	for (int i = 0; i < 8; i++) {
		verteces[i] = MyMatrix4x4::Transform(verteces[i], worldMat);
	}

	MyVector3 verteces1[8] = {
		{a.min.x,a.max.y,a.min.z},
		{a.max.x,a.max.y,a.min.z},
		{a.max.x,a.max.y,a.max.z},
		{a.min.x,a.max.y,a.max.z},
		{a.min.x,a.min.y,a.min.z},
		{a.max.x,a.min.y,a.min.z},
		{a.max.x,a.min.y,a.max.z},
		{a.min.x,a.min.y,a.max.z},
	};

	MyVector3 min[2] = {};
	MyVector3 max[2] = {};

	MyVector3 hoge[3] = {
		{1.0f,0.0f,0.0f},
		{0.0f,1.0f,0.0f},
		{0.0f,0.0f,1.0f}
	};

	float len[2] = {};
	float length = 0;

	MyVector3 close = {};

	MyVector3 ma = {};
	MyVector3 mi = {};

	Line line = {};
	for (int j = 0; j < 6; j++) {

		if (j < 3) {
			line.diff = b.orientations[j];
		}
		else {
			line.diff = hoge[j - 3];
		}

		for (int i = 0; i < 8; i++) {
			if (i == 0) {
				min[0] = Calc::ClosestPoint(verteces[i], line);
				max[0] = min[0];
				min[1] = Calc::ClosestPoint(verteces1[i], line);
				max[1] = min[0];
			}
			else {
				close = Calc::ClosestPoint(verteces[i], line);
				min[0].x = (std::min)(min[0].x, close.x);
				min[0].y = (std::min)(min[0].y, close.y);
				min[0].z = (std::min)(min[0].z, close.z);
				max[0].x = (std::max)(max[0].x, close.x);
				max[0].y = (std::max)(max[0].y, close.y);
				max[0].z = (std::max)(max[0].z, close.z);
				close = Calc::ClosestPoint(verteces1[i], line);
				min[1].x = (std::min)(min[1].x, close.x);
				min[1].y = (std::min)(min[1].y, close.y);
				min[1].z = (std::min)(min[1].z, close.z);
				max[1].x = (std::max)(max[1].x, close.x);
				max[1].y = (std::max)(max[1].y, close.y);
				max[1].z = (std::max)(max[1].z, close.z);
			}
		}

		len[0] = Calc::MakeLength(max[0], min[0]);
		len[1] = Calc::MakeLength(max[1], min[1]);

		ma.x = (std::max)(max[0].x, max[1].x);
		ma.y = (std::max)(max[0].y, max[1].y);
		ma.z = (std::max)(max[0].z, max[1].z);
		mi.x = (std::min)(min[0].x, min[1].x);
		mi.y = (std::min)(min[0].y, min[1].y);
		mi.z = (std::min)(min[0].z, min[1].z);
		length = Calc::MakeLength(ma, mi);

		if (len[0] + len[1] < length) {
			return false;
		}
	}
	

	return true;

}

bool Collision::IsCollision(const OBB& a, const OBB& b) {

	MyMatrix4x4 worldMat = {
		a.orientations[0].x,a.orientations[0].y,a.orientations[0].z,0,
		a.orientations[1].x,a.orientations[1].y,a.orientations[1].z,0,
		a.orientations[2].x,a.orientations[2].y,a.orientations[2].z,0,
		a.center.x,a.center.y,a.center.z,1
	};

	MyMatrix4x4 rotateMat = {
		a.orientations[0].x,a.orientations[0].y,a.orientations[0].z,0,
		a.orientations[1].x,a.orientations[1].y,a.orientations[1].z,0,
		a.orientations[2].x,a.orientations[2].y,a.orientations[2].z,0,
		0,0,0,1
	};

	MyMatrix4x4 worldMatInverse = MyMatrix4x4::Inverse(worldMat);

	MyMatrix4x4 rotateMatInverse = MyMatrix4x4::Inverse(rotateMat);

	AABB aabb = { MyVector3(a.size) * (-1) + a.center, MyVector3(a.size) + a.center};
	
	OBB obb = {};
	obb.center = MyMatrix4x4::Transform(b.center, worldMatInverse);
	obb.size = MyMatrix4x4::Transform(b.size, rotateMatInverse);
	for (int i = 0; i < 3; i++) {
		obb.orientations[i] = MyMatrix4x4::Transform(b.orientations[i], rotateMatInverse);
	}

	return IsCollision(aabb, obb);
}