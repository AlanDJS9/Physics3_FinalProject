#include "Sphere.h"
#include <MMath.h>
#include "QuadraticSolver.h"

using namespace MATH;
using namespace GEOMETRY;
using namespace std;

RayIntersectionInfo GEOMETRY::Sphere::rayIntersectionInfo(const Ray& ray) const
{
	RayIntersectionInfo rayInfo;
	const Vec3 sphereToRayStart = ray.start - Vec3(x, y, z);
	Vec3 D = ray.dir;
	Vec3 S = ray.start;
	Vec3 C = Vec3(x, y, z);
	// Solve the quadratic equation for this intersection
	// REFERENCE: Chapter 5 of Real Time Collision Detection by Ericson
	float a = VMath::dot(D, D);
	float b = 2.0f * (VMath::dot(S, D) - VMath::dot(D, C));
	float c = VMath::dot(S, S) + VMath::dot(C, C) - 2.0f * VMath::dot(S, C) - r * r;

	QuadraticSolution solution = solveQuadratic(a, b, c);

	if (solution.numSolutions == NumSolutions::zero) {
		rayInfo.isIntersected = false;
		cout << "no inter" << endl;
	}
		
	else if (solution.secondSolution < 0.0f)
	{
		rayInfo.isIntersected = true;
		rayInfo.intersectionPoint = ray.currentPosition(solution.secondSolution);
	}
	else
	{
		rayInfo.isIntersected = true;
		rayInfo.intersectionPoint = ray.currentPosition(solution.firstSolution);
	}

	return rayInfo;
}

void Sphere::generateVerticesAndNormals()
{
	// We need to fill the vertices and normals arrays with the correct data for a sphere
	// deltaTheta governs how close each ring will be around our sphere. Try messing with it
	const float deltaTheta = 5.0f;
	// deltaPhi governs how close each point will be per ring. Try messing with it
	const float deltaPhi = 2.0f;
	for (float thetaDeg = 0.0f; thetaDeg <= 360.0f; thetaDeg += deltaTheta)
	{
		// Build a ring
		Vec3 circle(r * cos(thetaDeg * DEGREES_TO_RADIANS), r * sin(thetaDeg * DEGREES_TO_RADIANS), 0.0f);
		for (float phiDeg = 0.0f; phiDeg <= 360.0f; phiDeg += deltaPhi) {
			// Rotate a point in the ring around the y-axis to build a sphere!
			Matrix3 rotationMatrix = MMath::rotate(deltaPhi, Vec3(0.0f, 1.0f, 0.0f));
			circle = rotationMatrix * circle;
			// Push the circle point to our vertices array
			vertices.push_back(circle);
			// The normal of a sphere points outwards from the center position Vec3(x, y, z)
			normals.push_back(circle - Vec3(x, y, z));
		}
	}
	// Once we are all done filling the vertices and normals, use the base class method to store the data in the GPU
	StoreMeshData(GL_POINTS);
}