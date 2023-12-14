#pragma once
#include <Vector.h>
namespace GEOMETRY {

	struct RayIntersectionInfo {
		bool isIntersected = false;
		MATH::Vec3 intersectionPoint;
		float t = 0.0f; // handy to know if we are going forward or backward
	};

	struct Ray {
		MATH::Vec3 start; // by default this is (0, 0, 0)
		MATH::Vec3 dir;
		Ray() {
			// Moses thinks (1, 1,0) is a good default direction
			// But Umer says thats not normalized
			// Max says default to the x-axis
			dir.set(1.0f, 0.0f, 0.0f);
		}
		Ray(const MATH::Vec3& start_, const MATH::Vec3& dir_) {
			set(start_, dir_);
		}
		void set(const MATH::Vec3& start_, const MATH::Vec3& dir_) {
			start = start_;
			dir = dir_;
		}

		MATH::Vec3 currentPosition(float t) const { // returns start + t * dir
			return start + (dir * t);
		}
	};

}