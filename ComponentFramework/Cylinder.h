#pragma once
#include "Shape.h"
#include <Vector.h>
namespace GEOMETRY
{
	struct Cylinder: public Shape
	{
		// Represent a cylinder by the two end cap centre positions and the radius.
		// REFERENCE: p.195 of Real Time Collision Detection by Ericson
		float r = 1.0f;
		MATH::Vec3 capCentrePosA;
		MATH::Vec3 capCentrePosB;

		// Default constructor sets the radius and cap centre positions, and fires off 
		// the generateVerticesAndNormals method
		Cylinder() {
			// Vlad thinks a good default cylinder is sitting up straight with a radius of 1.0f
			// and a height of 2.0f
			set(1.0f, MATH::Vec3(0.0f, 2.0f, 0.0f), MATH::Vec3(0.0f, 0.0f, 0.0f));
		}
		// Don’t forget to execute generateVerticesAndNormals at the end of this three 
		// parameter constructor too
		Cylinder(float r_, MATH::Vec3 capCentrePosA_, MATH::Vec3 capCentrePosB_) {
			set(r_, capCentrePosA_, capCentrePosB_);
		}
		// A set method is handy, just remember to fire off generateVerticesAndNormals
		//and StoreMeshData(GL_POINTS) at the end
		void set(float r_, MATH::Vec3 capCentrePosA_, MATH::Vec3 capCentrePosB_) {
			r = r_;
			capCentrePosA = capCentrePosA_;
			capCentrePosB = capCentrePosB_;
			generateVerticesAndNormals();
		}
		// Fill the vertices and normals list with Vec3's to represent a cylinder
		RayIntersectionInfo rayIntersectionInfo(const Ray& ray) const override;
		RayIntersectionInfo checkInfiniteCylinder(const Ray& ray) const;
		RayIntersectionInfo checkEndCap(const Ray& ray, float t) const;
		void generateVerticesAndNormals() override;
	};
}
