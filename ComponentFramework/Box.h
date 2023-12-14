#pragma once
#include "Shape.h"
#include <Quaternion.h>
#include <Vector.h>
#include "Vector.h"
#include <vector>
#include <array>
namespace GEOMETRY {
	struct Box : public Shape
	{
		// Represent an Orientated Bounding Box by centre position, half extents, and 
		// orientation.REFERENCE: Real Time Collision Detection by Ericson
		// Ericson recommends keeping the orientation as a matrix for fast collision 
		// detection, but I love quaternions too much
		MATH::Vec3 centre;
		MATH::Vec3 halfExtents;
		MATH::Quaternion orientation;

		// The constructors set the centre position, the half extents along x, y, and 
		// z, and the orientation quaterion.Then it generates the vertices and normal for
		// the shape and stores the mesh data in order to render
		Box() {
			// TODO Assignment 1
			set(MATH::Vec3(1.0f,1.0f,1.0f), MATH::Vec3(2.0f, 2.0f, 2.0f), MATH::Quaternion());
		}

		Box(MATH::Vec3 centre_, MATH::Vec3 halfExtents_, MATH::Quaternion orientation_) {

			set(centre_, halfExtents_, orientation_);
			// TODO Assignment 1
		}
		void set(MATH::Vec3 centre_, MATH::Vec3 halfExtents_, MATH::Quaternion
			orientation_) {
			centre = centre_;
			halfExtents = halfExtents_;
			orientation = orientation_;
			generateVerticesAndNormals();
			// TODO Assignment 1 - done
		}

		// Fill the vertices and normals list with Vec3's to represent a box
		RayIntersectionInfo rayIntersectionInfo(const Ray& ray) const override;

		void generateVerticesAndNormals() override;
		struct Slab
		{
			// REFERENCE: Ch. 4 Real Time Collision Detection by Ericson
			// This is called a Kay-Kajiya Slab volume
			// Region R = { (x, y, z) | distNear <= a*x + b*y + c*z <= distFar }
			MATH::Vec3 normal;
			float distNear;		// Signed distance from origin for near plane 
			float distFar;		// Signed distance from origin for far plane 
		};

	};
}
