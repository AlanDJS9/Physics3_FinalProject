#include "ShapeComponent.h"
using namespace GEOMETRY;
using namespace MATH;
ShapeComponent::ShapeComponent(Component* parent_, const Sphere& sphere_) :Component(parent_)
{
	shapeType = ShapeType::sphere;
	// We are sure at this point that shape should be a sphere
	// So we can safely make a shared pointer of type sphere with the constructor Sphere(MATH::Vec3 centre, float r)
	shape = std::make_shared<Sphere>(Vec3(sphere_.x, sphere_.y, sphere_.z), sphere_.r);
}

ShapeComponent::ShapeComponent(Component* parent_, const Cylinder& cylinder_) : Component(parent_)
{
	shapeType = ShapeType::cylinder;
	shape = std::make_shared<Cylinder>(cylinder_.r, cylinder_.capCentrePosA, cylinder_.capCentrePosB);
}

ShapeComponent::ShapeComponent(Component* parent_, const GEOMETRY::Capsule& capsule_) : Component(parent_)
{
	shapeType = ShapeType::capsule;
	shape = std::make_shared<Capsule>(capsule_.r, capsule_.sphereCentrePosA, capsule_.sphereCentrePosB);
}

ShapeComponent::ShapeComponent(Component* parent_, const GEOMETRY::Box& box_) : Component(parent_)
{
	shapeType = ShapeType::box;
	shape = std::make_shared<Box>(box_.centre, box_.halfExtents, box_.orientation);
}

// TODO for Assignment 1:
// The other constructors that take in Cylinder, Capsule, or Box 

ShapeComponent::~ShapeComponent()
{
}

bool ShapeComponent::OnCreate()
{
	return true;
}

void ShapeComponent::OnDestroy()
{
}

void ShapeComponent::Update(const float deltaTime_)
{
}

void ShapeComponent::Render() const
{
	shape->debugDraw();
}
