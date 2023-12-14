#include "Physics.h"
#include "TransformComponent.h"
#include <QMath.h>

// TODO for assignment 3
// Fill out all these methods
void PHYSICS::ApplyForce(Ref<PhysicsComponent> body, const MATH::Vec3& force)
{
	body->accel = force / body->mass;
}

void PHYSICS::UpdatePos(Ref<PhysicsComponent> body, float deltaTime)
{
	// Skip the 1/2 a t^2 for now. Our constraint motion does not need it
	body->pos += body->vel * deltaTime;
}

void PHYSICS::UpdateVel(Ref<PhysicsComponent> body, float deltaTime)
{
	body->vel += body->accel * deltaTime;
}

void PHYSICS::UpdateOrientation(Ref<PhysicsComponent> body, float deltaTime)
{
	float changeInAngle = VMath::mag(body->angularVel) * deltaTime * 100;
	Vec3 axisOfRotation = VMath::normalize(body->angularVel);
	Quaternion rotation = QMath::angleAxisRotation(changeInAngle, axisOfRotation);
	body->orientation = rotation * body->orientation;
}

void PHYSICS::UpdateTransform(Ref<Actor> actor)
{
	Ref<TransformComponent> transform = actor->GetComponent<TransformComponent>();
	Ref<PhysicsComponent> body = actor->GetComponent<PhysicsComponent>();
	// Make sure the transfor has the position and orientation from physics
	transform->pos = body->pos;
	transform->orientation = body->orientation;
}
void PHYSICS::StraightLineConstraint(Ref<PhysicsComponent> body, float deltaTime, float slope, float yIntercept)
{
	float positionConstraint = body->pos.y - slope * body->pos.x - yIntercept;
	float baumgarteStabilization = 0.15f; // you can tune this
	if (deltaTime < VERY_SMALL) {
		deltaTime = 1.0f / 60.0f; // helps me when I'm debugging
	}
	// The sign on the bias can be negative or positive
	// depending on the direction of the constraint
	float bias = baumgarteStabilization * positionConstraint / deltaTime;
	// We derived JV, which is the velocity constraint, on the board
	float JV = body->vel.y - slope * body->vel.x;
	// We calculated J * M^-1 * J^T on the board
	float inverseMeffective = (slope * slope + 1.0f) / body->mass;

	float langrangian = (-JV - bias) / inverseMeffective;
	// JT is the Jacobian transposed. So it's a column vector
	Vec3 JT = Vec3(-slope, 1.0f, 0.0f);
	Vec3 changeInVelocity = (1.0f / body->mass) * JT * langrangian;
	body->vel += changeInVelocity;

}

void PHYSICS::PlaneConstraint(Ref<PhysicsComponent> body, float deltaTime, const Vec3& planeNormal, float planeD)
{
	// Me and Scott like planes to follow the equation ax + by + cz - d = 0
	float positionConstraint = VMath::dot(body->pos, planeNormal) - planeD;

	float baumgarteStabilization = 1.01f; // you can tune this
	if (deltaTime < VERY_SMALL) {
		deltaTime = 1.0f / 60.0f; // helps me when I'm debugging
	}
	// The sign on the bias can be negative or positive
	// depending on the direction of the constraint
	float bias = baumgarteStabilization * positionConstraint / deltaTime;

	// We derived JV, which is the velocity constraint, on the board
	float JV = VMath::dot(planeNormal, body->vel);

	// We calculated J * M^-1 * J^T on the board
	float inverseMeffective = VMath::dot(planeNormal, planeNormal) / body->mass;

	float langrangian = (-JV - bias) / inverseMeffective;
	// JT is the Jacobian transposed. So it's a column vector
	Vec3 JT = planeNormal;
	Vec3 changeInVelocity = (1.0f / body->mass) * JT * langrangian;
	body->vel += changeInVelocity;
}

void PHYSICS::MouseConstraint(Ref<PhysicsComponent> body, float deltaTime, const Vec3& intersectionPoint)
{
	// Assume the position in world space is the centre of mass
	MATH::Vec3 body_centreOfMass = body->pos;
	// r is the vector from the centre of mass to the attachment point
	MATH::Vec3 r = intersectionPoint - body_centreOfMass;
	MATH::Matrix3 m_effective(
		1 + r.z * r.z + r.y * r.y, -r.x * r.y, -r.x * r.z,  // first column
		-r.x * r.y, 1 + r.z * r.z + r.x * r.x, -r.y * r.z,   // second column
		-r.x * r.z, -r.y * r.z, 1 + r.y * r.y + r.x * r.x); // third column

	MATH::Matrix3 m_effective_inversed = MATH::MMath::inverse(m_effective);
	// Now calculate lambda = m_effective_inversed *(-JV -b)
	// lambda = m_effective_inversed * (beta/deltatime) * d - m_effective_inversed * JV
	float beta = 0.2f;
	MATH::Vec3 d = (body_centreOfMass + r - intersectionPoint);
	// at the start of the program deltaTime == zero! Set to 1/60 instead
	if (deltaTime < VERY_SMALL) { deltaTime = 0.01667f; }
	MATH::Vec3 negative_b = (beta / deltaTime) * d;
	// for a static anchor point I wrote out the JV vector on paper
	MATH::Vec3 JV = body->vel + MATH::VMath::cross(body->angularVel, r);;

	MATH::Vec3 lambda = m_effective_inversed * negative_b + m_effective_inversed * JV;
	// I wrote out on paper what the change in vel & angularVel should be
	MATH::Vec3 deltaAngularVel = -MATH::VMath::cross(lambda, r);

	body->vel -= lambda;
	body->angularVel -= deltaAngularVel;

}
