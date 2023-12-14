#include "Cylinder.h"
#include <MMath.h>
#include "QuadraticSolver.h"

using namespace GEOMETRY;
using namespace MATH;
using namespace std;

void GEOMETRY::Cylinder::generateVerticesAndNormals()
{
	// For now, lets just build a sphere for debugging purposes
	const float deltaTheta = 0.8f;
	// deltaPhi governs how close each point will be per ring. Try messing with it
	const float deltaPhi = 0.8f;

	
	cout << "CAP A center " << capCentrePosA.y << endl;
	cout << "CAP B center " << capCentrePosB.y << endl;
	
	
	float difference = VMath::distance(capCentrePosA, capCentrePosB);
	cout << "Difference " << difference << endl;

	//CAP A
	for (float thetaDeg = 0.0f; thetaDeg <= 360.0f; thetaDeg += deltaTheta)
	{
		for (float t = 0.0; t < r; t += 0.1f) {
			Vec3 circleTopx(
							( t ) * cos(thetaDeg * DEGREES_TO_RADIANS), 
							( t ) * sin(thetaDeg * DEGREES_TO_RADIANS), 
							0.0f);
			// Push the circle point to our vertices array
			vertices.push_back(circleTopx + Vec3(0.0f, 0.0f, 0.0f));
			// The normal of a sphere points outwards from the center position Vec3(x, y, z)
			normals.push_back(circleTopx);	
		}
	}

	//BODY
	for (float thetaDeg = 0.0f; thetaDeg <= 360.0f; thetaDeg += deltaTheta)
	{
			Vec3 circle(r * cos(thetaDeg * DEGREES_TO_RADIANS), r * sin(thetaDeg * DEGREES_TO_RADIANS), 0.0f);
			// Build a ring
			for (float i = 0.0; i < difference; i += 0.2f) {
			//	cout << "Difference -> " << i << endl;
			// Push the circle point to our vertices array
			vertices.push_back(circle + Vec3(0.0f, 0.0f, i));
			// The normal of a sphere points outwards from the center position Vec3(x, y, z)
			normals.push_back(circle);
			}
	}

	//CAP B
	for (float thetaDeg = 0.0f; thetaDeg <= 360.0f; thetaDeg += deltaTheta)
	{
		for (float t = 0.0; t < r; t += 0.1f) {
			Vec3 circleTopx(
				(t)*cos(thetaDeg * DEGREES_TO_RADIANS),
				(t)*sin(thetaDeg * DEGREES_TO_RADIANS),
				0.0f);
			// Push the circle point to our vertices array
			vertices.push_back(circleTopx + Vec3(0.0f, 0.0f, difference));
			// The normal of a sphere points outwards from the center position Vec3(x, y, z)
			normals.push_back(circleTopx);
		}
	}

	
	// Once we are all done filling the vertices and normals, use the base class method to store the data in the GPU
	StoreMeshData(GL_POINTS);
}
RayIntersectionInfo GEOMETRY::Cylinder::rayIntersectionInfo(const Ray& ray) const
{
	RayIntersectionInfo rayInfo;

	rayInfo = checkInfiniteCylinder(ray);

	if (rayInfo.isIntersected == false)
		return rayInfo;							// Return it now, as there are no roots (or we are going backwards)

	Vec3 S = ray.start;							// Origin of ray
	Vec3 V = ray.dir;							// Direction of ray
	Vec3 P = rayInfo.intersectionPoint;			// Hit point

	Vec3 AB = capCentrePosB - capCentrePosA;	// Distance between pos A and B
	Vec3 AP = P - capCentrePosA;				// Distance between pos A and the hit point
	Vec3 AS = S - capCentrePosA;				// Vector from A to the origin of the ray (S - A)
	Vec3 BS = S - capCentrePosB;				// Vector from B to the origin of the ray (S - A)

	float dot_AB_AB = VMath::dot(AB, AB);
	float dot_AP_AB = VMath::dot(AP, AB);
	float dot_AS_AB = VMath::dot(AS, AB);
	float dot_V_AB = VMath::dot(V, AB);
	float dot_BS_AB = VMath::dot(BS, AB);

	if (dot_AP_AB < 0)
	{
		// We are outside of cap centre A's plane
		if (VMath::dot(ray.dir, AB) > 0.0f)
		{
			// Need to check for ray coming at endcap A
			float t = -dot_AS_AB / dot_V_AB;
			return checkEndCap(ray, t);
		}
		else
		{
			// Ray is going the wrong way
			rayInfo.isIntersected = false;
			return rayInfo;
		}
	}
	else if (dot_AP_AB > dot_AB_AB)
	{
		// We are outside of cap centre B's plane
		// Means that the point is further than the length of the cylinder from endcap B
		if (VMath::dot(ray.dir, AB) < 0.0f)
		{
			// Ray is coming for endcap B
			float t = -dot_BS_AB / dot_V_AB;
			return checkEndCap(ray, t);
		}
		else
		{
			// Ray is going the wrong way
			rayInfo.isIntersected = false;
			return rayInfo;
		}
	}

	return rayInfo;
}


RayIntersectionInfo Cylinder::checkInfiniteCylinder(const Ray& ray) const
{
	RayIntersectionInfo rayInfo;

	Vec3 V = ray.dir;							// Direction of ray
	Vec3 AB = capCentrePosB - capCentrePosA;	// Vector from A to B (so B - A)

	Vec3 S = ray.start;							// Origin of ray
	Vec3 AS = S - capCentrePosA;				// Vector from A to the origin of the ray (S - A)

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

	return rayInfo;
}


RayIntersectionInfo Cylinder::checkEndCap(const Ray& ray, float t) const
{
	RayIntersectionInfo rayInfo;

	Vec3 hitPoint = ray.currentPosition(t);

	float AQ = VMath::mag(hitPoint - capCentrePosA);
	float BQ = VMath::mag(hitPoint - capCentrePosB);

	// Inside of the cap (smaller than the radius)
	if (AQ <= r || BQ <= r)
	{
		rayInfo.isIntersected = true;
		rayInfo.intersectionPoint = hitPoint;
		rayInfo.t = t;
		return rayInfo;
	}

	// Outside of the cap (larger than the radius)
	rayInfo.isIntersected = false;
	return rayInfo;
}
