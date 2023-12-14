#include "Capsule.h"
#include <MMath.h>
#include "QuadraticSolver.h"

using namespace MATH;
using namespace GEOMETRY;



void GEOMETRY::Capsule::generateVerticesAndNormals()
{
	// Temporarily, lets make a sphere
	float difference = VMath::distance(sphereCentrePosA, sphereCentrePosB);

	// We need to fill the vertices and normals arrays with the correct data for a sphere
	// deltaTheta governs how close each ring will be around our sphere. Try messing with it
	const float deltaTheta = 2.0f;
	// deltaPhi governs how close each point will be per ring. Try messing with it
	const float deltaPhi = 2.0f;
	
	//CapsuleTop
	for (float thetaDeg = 0.0f; thetaDeg <= 180.0f; thetaDeg += deltaTheta)
	{
		Vec3 circle(r * cos(thetaDeg * DEGREES_TO_RADIANS), r * sin(thetaDeg * DEGREES_TO_RADIANS), 0.0f);
		circle += sphereCentrePosA;
		for (float phiDeg = 0.0f; phiDeg <= 360.0f; phiDeg += deltaPhi) {
			Matrix3 rotationMatrix = MMath::rotate(deltaPhi, Vec3(0.0f, 1.0f, 0.0f));
			circle = rotationMatrix * circle;
			vertices.push_back(circle);
			normals.push_back(circle);
		}
	}

	//CapsuleBottom
	for (float thetaDeg = 180.0f; thetaDeg <= 360.0f; thetaDeg += deltaTheta)
	{
		Vec3 circle(r * cos(thetaDeg * DEGREES_TO_RADIANS), r * sin(thetaDeg * DEGREES_TO_RADIANS), 0.0f);
		circle += sphereCentrePosA;
		for (float phiDeg = 0.0f; phiDeg <= 360.0f; phiDeg += deltaPhi) {
			Matrix3 rotationMatrix = MMath::rotate(deltaPhi, Vec3(0.0f, 1.0f, 0.0f));
			circle = rotationMatrix * circle;
			vertices.push_back(circle + Vec3(0.0f, -difference, 0.0f));
			normals.push_back(circle);
		}
	}

	// TODO Assignment 1
	// Build another sphere (half sphere would be cool) for marios feet
	// And a cylinder between the two spheres for the capsule body
	
	//BODY
	for (float thetaDeg = 0.0f; thetaDeg <= 360.0f; thetaDeg += deltaTheta)
	{
		Vec3 circle(r * cos(thetaDeg * DEGREES_TO_RADIANS), r * sin(thetaDeg * DEGREES_TO_RADIANS),1.0);
		// Build a ring
		circle = MMath::rotate(90.0, Vec3(1.0f, 0.0f, 0.0f)) * circle;

		for (float i = 0.0; i < difference; i += 0.05f) {
			//	cout << "Difference -> " << i << endl;
			// Push the circle point to our vertices array
			vertices.push_back(circle + Vec3(0.0f, i, 0.0f));
			// The normal of a sphere points outwards from the center position Vec3(x, y, z)
			normals.push_back(circle);
		}
	}

	// Once we are all done filling the vertices and normals, use the base class method to store the data in the GPU
	StoreMeshData(GL_POINTS);
}


RayIntersectionInfo Capsule::rayIntersectionInfo(const Ray& ray) const
{
	RayIntersectionInfo rayInfo;

	// First check sphere A
	rayInfo = checkEndSphere(sphereCentrePosA, ray);

	// If sphere A has intersection, check halfsphere
	if (rayInfo.isIntersected)
	{
		//cout << "sphereCentrePosA" << endl;
		rayInfo = checkHalfSphere(rayInfo, sphereCentrePosA);

		if (rayInfo.isIntersected)
			return rayInfo;
	}

	// If sphere A has no intersection, check sphere B
	//cout << "Checking B" << endl;
	rayInfo = checkEndSphere(sphereCentrePosB, ray);

	// If sphere B has intersection, check halfsphere
	if (rayInfo.isIntersected)
	{
		//cout << "sphereCentrePosB" << endl;
		rayInfo = checkHalfSphere(rayInfo, sphereCentrePosB);

		if (rayInfo.isIntersected)
			return rayInfo;
	}

	// If sphere B has no intersection, check infinite cylinder
	//cout << "Checking cylinder" << endl;
	rayInfo = checkInfiniteCylinder(ray);

	if (rayInfo.isIntersected)
		return rayInfo;

	//cout << "End of main function" << endl;
	return rayInfo;
}

RayIntersectionInfo Capsule::checkEndSphere(MATH::Vec3 sphereCentre, const Ray& ray) const
{
	/**
	// Change to object space
	Vec3 sphereSpace;
	sphereSpace -= sphereCentrePosA;
	/**/

	RayIntersectionInfo rayInfo;
	const Vec3 sphereToRayStart = ray.start - sphereCentre;

	// Solve the quadratic equation for this intersection
	// REFERENCE: Chapter 5 of Real Time Collision Detection by Ericson
	const float a = VMath::dot(ray.dir, ray.dir);
	const float b = 2.0f * VMath::dot(sphereToRayStart, ray.dir);
	const float c = VMath::dot(sphereToRayStart, sphereToRayStart) - r * r;

	QuadraticSolution soln = solveQuadratic(a, b, c);

	if (soln.numSolutions == NumSolutions::zero)
		rayInfo.isIntersected = false;
	else if (soln.secondSolution < 0.0f)
	{
		rayInfo.isIntersected = true;
		rayInfo.intersectionPoint = ray.currentPosition(soln.secondSolution);
	}
	else
	{
		rayInfo.isIntersected = true;
		rayInfo.intersectionPoint = ray.currentPosition(soln.firstSolution);
	}

	return rayInfo;
}

RayIntersectionInfo Capsule::checkHalfSphere(const RayIntersectionInfo& rayInfoFullSphere, const MATH::Vec3& sphereCentre) const
{
	RayIntersectionInfo rayInfo;

	Vec3 CP = rayInfoFullSphere.intersectionPoint - sphereCentre;
	Vec3 AB = sphereCentrePosB - sphereCentrePosA;

	float dot_CP_AB = VMath::dot(CP, AB);

	/// <summary>
	/// If sphereCentre is A and the dot product is bigger than 0, it's the wrong side
	/// If sphereCentre is B and the dot product is smaller than 0, it's the wrong side
	/// </summary>
	/// <param name="rayInfoFullSphere"></param>
	/// <param name="sphereCentre"></param>
	/// <returns></returns>

	if (sphereCentre == sphereCentrePosA)
	{
		//cout << "Same as A" << endl;

		if (dot_CP_AB > 0)
		{
			// Ignore - Wrong side
			//cout << "Wrong side of the cylinder" << endl;
			rayInfo.isIntersected = false;
			return rayInfo;
		}

		// Correct side
		//cout << "In half of sphere" << endl;
		rayInfo.isIntersected = true;
		return rayInfo;
	}
	else if (sphereCentre == sphereCentrePosB)
	{
		//cout << "Same as B" << endl;

		if (dot_CP_AB > 0)
		{
			// Correct side
			//cout << "In half of sphere" << endl;
			rayInfo.isIntersected = true;
			return rayInfo;
		}

		// Ignore - Wrong side
		//cout << "Wrong side of the cylinder" << endl;

		rayInfo.isIntersected = false;
		return rayInfo;
	}
}

RayIntersectionInfo Capsule::checkInfiniteCylinder(const Ray& ray) const
{
	RayIntersectionInfo rayInfo;

	Vec3 V = ray.dir;								// Direction of ray
	Vec3 AB = sphereCentrePosB - sphereCentrePosA;	// Vector from A to B (so B - A)

	Vec3 S = ray.start;								// Origin of ray
	Vec3 AS = S - sphereCentrePosA;					// Vector from A to the origin of the ray (S - A)

	// A, B and C value
	float A = VMath::dot(V, V) - (VMath::dot(V, VMath::normalize(AB)) * VMath::dot(V, VMath::normalize(AB)));
	float B = 2 * (VMath::dot(AS, V) - (VMath::dot(V, VMath::normalize(AB)) * VMath::dot(AS, VMath::normalize(AB))));
	float C = VMath::dot(AS, AS) - (VMath::dot(AS, VMath::normalize(AB)) * VMath::dot(AS, VMath::normalize(AB))) - r * r;

	QuadraticSolution soln = solveQuadratic(A, B, C);

	if (soln.numSolutions == NumSolutions::zero)
	{
		rayInfo.isIntersected = false;
		return rayInfo;
	}
	else if (soln.secondSolution < 0.0f)
	{
		rayInfo.isIntersected = true;
		rayInfo.intersectionPoint = ray.currentPosition(soln.secondSolution);
		rayInfo.t = soln.secondSolution;
	}
	else
	{
		rayInfo.isIntersected = true;
		rayInfo.intersectionPoint = ray.currentPosition(soln.firstSolution);
		rayInfo.t = soln.firstSolution;
	}

	/**/
	Vec3 P = rayInfo.intersectionPoint;				// Hit point

	Vec3 AP = P - sphereCentrePosA;					// Distance between pos A and the hit point
	Vec3 BS = S - sphereCentrePosB;					// Vector from B to the origin of the ray (S - A)

	float dot_AB_AB = VMath::dot(AB, AB);
	float dot_AP_AB = VMath::dot(AP, AB);
	float dot_AS_AB = VMath::dot(AS, AB);
	float dot_V_AB = VMath::dot(V, AB);
	float dot_BS_AB = VMath::dot(BS, AB);

	if (dot_AP_AB < 0)
	{
		// We are outside of cap centre A's plane
		if (VMath::dot(ray.dir, AB) > 0.0f) {
			//cout << "Return true for if" << endl;
			return rayInfo;
		}
		else
		{
			// Ray is going the wrong way
			//cout << "Return false for if" << endl;
			rayInfo.isIntersected = false;
			return rayInfo;
		}
	}
	else if (dot_AP_AB > dot_AB_AB)
	{
		// We are outside of cap centre B's plane
		// Means that the point is further than the length of the cylinder from endcap B
		if (VMath::dot(ray.dir, AB) < 0.0f) {
			//cout << "Return true for else if" << endl;
			return rayInfo;
		}
		else
		{
			// Ray is going the wrong way
			//cout << "Return false for else if" << endl;
			rayInfo.isIntersected = false;
			return rayInfo;
		}
	}
	else {
		//cout << "End of loop" << endl;
		return rayInfo;
	}
	/**/
}
