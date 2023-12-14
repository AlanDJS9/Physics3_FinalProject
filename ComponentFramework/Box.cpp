#include "Box.h"
#include <QMath.h>
using namespace MATH;
using namespace GEOMETRY;
#include "QuadraticSolver.h"


void GEOMETRY::Box::generateVerticesAndNormals()
{
	// Scott needs 2 triangles for each face
	// There are 6 faces
	// Let's start by identifying the 8 vertices
	// TODO Assignment 1: Add the missing 5 vertices




	const Vec3 bottomLeftBack0	= QMath::rotate((centre + Vec3(-halfExtents.x, -halfExtents.y, -halfExtents.z)), orientation);
	const Vec3 bottomRightBack1	= QMath::rotate((centre + Vec3(+halfExtents.x, -halfExtents.y, -halfExtents.z)), orientation);
	const Vec3 topRighBack2 = QMath::rotate((centre + Vec3(+halfExtents.x, +halfExtents.y, -halfExtents.z)), orientation);
	const Vec3 topLeftBack3 = QMath::rotate((centre + Vec3(-halfExtents.x, +halfExtents.y, -halfExtents.z)), orientation);
	const Vec3 bottomLeftFront4 = QMath::rotate((centre + Vec3(-halfExtents.x, -halfExtents.y, +halfExtents.z)), orientation);
	const Vec3 bottomRightFront5 = QMath::rotate((centre + Vec3(+halfExtents.x, -halfExtents.y, +halfExtents.z)), orientation);
	const Vec3 topRightFront6 = QMath::rotate((centre + Vec3(+halfExtents.x, +halfExtents.y, +halfExtents.z)), orientation);
	const Vec3 topLeftFront7 = QMath::rotate((centre + Vec3(-halfExtents.x, +halfExtents.y, +halfExtents.z)), orientation);


	//FACE1 frontNormal
	vertices.push_back(bottomLeftBack0);
	vertices.push_back(bottomRightBack1);
	vertices.push_back(topRighBack2);
	normals.push_back(QMath::rotate(Vec3(0, 0, -1), orientation));
	normals.push_back(QMath::rotate(Vec3(0, 0, -1), orientation));
	normals.push_back(QMath::rotate(Vec3(0, 0, -1), orientation));

	vertices.push_back(topRighBack2);
	vertices.push_back(topLeftBack3);
	vertices.push_back(bottomLeftBack0);
	normals.push_back(QMath::rotate(Vec3(0, 0, -1), orientation));
	normals.push_back(QMath::rotate(Vec3(0, 0, -1), orientation));
	normals.push_back(QMath::rotate(Vec3(0, 0, -1), orientation));

	//FACE2 backNormal
	vertices.push_back(bottomRightFront5);
	vertices.push_back(bottomLeftFront4);
	vertices.push_back(topLeftFront7);
	normals.push_back(QMath::rotate(Vec3(0, 0, 1), orientation));
	normals.push_back(QMath::rotate(Vec3(0, 0, 1), orientation));
	normals.push_back(QMath::rotate(Vec3(0, 0, 1), orientation));
	
	
	vertices.push_back(topLeftFront7);
	vertices.push_back(topRightFront6);
	vertices.push_back(bottomRightFront5);
	normals.push_back(QMath::rotate(Vec3(0, 0, 1), orientation));
	normals.push_back(QMath::rotate(Vec3(0, 0, 1), orientation));
	normals.push_back(QMath::rotate(Vec3(0, 0, 1), orientation));

	////FACE3 topNormal	
	vertices.push_back(topRighBack2);
	vertices.push_back(bottomRightBack1);
	vertices.push_back(bottomRightFront5);
	normals.push_back(QMath::rotate(Vec3(0, 1, 0), orientation));
	normals.push_back(QMath::rotate(Vec3(0, 1, 0), orientation));
	normals.push_back(QMath::rotate(Vec3(0, 1, 0), orientation));


	vertices.push_back(bottomRightFront5);
	vertices.push_back(topRightFront6);
	vertices.push_back(topRighBack2);
	normals.push_back(QMath::rotate(Vec3(0, 1, 0), orientation));
	normals.push_back(QMath::rotate(Vec3(0, 1, 0), orientation));
	normals.push_back(QMath::rotate(Vec3(0, 1, 0), orientation));

	////FACE4 bottomNormal
	vertices.push_back(bottomLeftFront4);
	vertices.push_back(bottomLeftBack0);
	vertices.push_back(topLeftBack3);
	normals.push_back(QMath::rotate(Vec3(0, -1, 0), orientation));
	normals.push_back(QMath::rotate(Vec3(0, -1, 0), orientation));
	normals.push_back(QMath::rotate(Vec3(0, -1, 0), orientation));


	vertices.push_back(topLeftBack3);
	vertices.push_back(topLeftFront7);
	vertices.push_back(bottomLeftFront4);
	normals.push_back(QMath::rotate(Vec3(0, -1, 0), orientation));
	normals.push_back(QMath::rotate(Vec3(0, -1, 0), orientation));
	normals.push_back(QMath::rotate(Vec3(0, -1, 0), orientation));

	////FACE5 leftNormal
	vertices.push_back(bottomLeftBack0);
	vertices.push_back(bottomLeftFront4);
	vertices.push_back(bottomRightFront5);
	normals.push_back(QMath::rotate(Vec3(-1, 0, 0), orientation));
	normals.push_back(QMath::rotate(Vec3(-1, 0, 0), orientation));
	normals.push_back(QMath::rotate(Vec3(-1, 0, 0), orientation));


	vertices.push_back(bottomRightFront5);
	vertices.push_back(bottomRightBack1);
	vertices.push_back(bottomLeftBack0);
	normals.push_back(QMath::rotate(Vec3(-1, 0, 0), orientation));
	normals.push_back(QMath::rotate(Vec3(-1, 0, 0), orientation));
	normals.push_back(QMath::rotate(Vec3(-1, 0, 0), orientation));

	////FACE6 rightNormal
	vertices.push_back(topLeftBack3);
	vertices.push_back(topRighBack2);
	vertices.push_back(topRightFront6);
	normals.push_back(QMath::rotate(Vec3(1, 0, 0), orientation));
	normals.push_back(QMath::rotate(Vec3(1, 0, 0), orientation));
	normals.push_back(QMath::rotate(Vec3(1, 0, 0), orientation));


	vertices.push_back(topRightFront6);
	vertices.push_back(topLeftFront7);
	vertices.push_back(topLeftBack3);
	normals.push_back(QMath::rotate(Vec3(1, 0, 0), orientation));
	normals.push_back(QMath::rotate(Vec3(1, 0, 0), orientation));
	normals.push_back(QMath::rotate(Vec3(1, 0, 0), orientation));



	// TODO Assignment 1
	// Figure out triangle 2 for the front face
	// And do this for all the other faces

	// Send this to Scott and render in triangle mode
	StoreMeshData(GL_TRIANGLES);
}



RayIntersectionInfo GEOMETRY::Box::rayIntersectionInfo(const Ray& ray) const
{
	RayIntersectionInfo rayInfo;

#pragma region Slabs
	// Imagine a box is just made up of three infinite slabs
	Slab slabX;
	slabX.normal = Vec3(1.0f, 0.0f, 0.0f);
	slabX.distNear = -halfExtents.x;
	slabX.distFar = halfExtents.x;

	Slab slabY;
	slabY.normal = Vec3(0.0f, 1.0f, 0.0f);
	slabY.distNear = -halfExtents.y;
	slabY.distFar = halfExtents.y;

	Slab slabZ;
	slabZ.normal = Vec3(0.0f, 0.0f, 1.0f);
	slabZ.distNear = -halfExtents.z;
	slabZ.distFar = halfExtents.z;

	std::array<Slab, 3> slabs = { slabX, slabY, slabZ };

#pragma endregion

	float tMin = 0.0f;
	float tMax = FLT_MAX;

	for (int i = 0; i < slabs.size(); i++)
	{
		float t1 = (slabs[i].distNear - ray.start[i]) / ray.dir[i];
		float t2 = (slabs[i].distFar - ray.start[i]) / ray.dir[i];
		//float t2 = (slabs[i].distFar ............; (-ray.start[i]) / ray.dir[i])

		if (t1 > t2)
			std::swap(t1, t2);

		tMin = std::max(tMin, t1);
		tMax = std::min(tMax, t2);

		if (tMin > tMax)
			return rayInfo;
	}

	rayInfo.isIntersected = true;
	rayInfo.t = tMin;
	rayInfo.intersectionPoint = ray.currentPosition(tMin);

	return rayInfo;
}
