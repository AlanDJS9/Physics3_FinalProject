#pragma once
#include "Shape.h"
#include <Vector.h>

namespace GEOMETRY {
	struct Capsule : public Shape
	{
		/// A capsule is represented by two spheres swept along connecting axis. 
		/// REFERENCE: Robust Contact Creation for Physics Simulations, D.Gregorius
		/// http://media.steampowered.com/apps/valve/2015/DirkGregorius_Contacts.pdf
		float r = 1.0f;
		MATH::Vec3 sphereCentrePosA;
		MATH::Vec3 sphereCentrePosB;

		// The constructors sets the radius and centre positions of the caps, generates 
		// the vertices and normal for the shape, and stores the mesh data in order to
		// render
		Capsule() {
			// The default capsule
			set(1.0f, MATH::Vec3(0.0f, 1.0f, 0.0f), MATH::Vec3(0.0f, 0.0f, 0.0f));
		}

		Capsule(float r_, MATH::Vec3 sphereCentrePosA_, MATH::Vec3 sphereCentrePosB_) {
			set(r_, sphereCentrePosA_, sphereCentrePosB_);
		}

		void set(float r_, MATH::Vec3 sphereCentrePosA_, MATH::Vec3 sphereCentrePosB_) {
			r = r_;
			sphereCentrePosA = sphereCentrePosA_;
			sphereCentrePosB = sphereCentrePosB_;
			generateVerticesAndNormals();
		}
		// Fill the vertices and normals list with Vec3's to represent a capsule
		// Try spheres for the top and bottom, with a cylinder for the main body
		RayIntersectionInfo rayIntersectionInfo(const Ray& ray) const override;
		RayIntersectionInfo checkEndSphere(MATH::Vec3 sphereCentre, const Ray& ray) const;
		RayIntersectionInfo checkHalfSphere(const RayIntersectionInfo& rayInfoFullSphere, const MATH::Vec3& sphereCentre) const;
		RayIntersectionInfo checkInfiniteCylinder(const Ray& ray) const;

		void generateVerticesAndNormals() override;

	};
}
