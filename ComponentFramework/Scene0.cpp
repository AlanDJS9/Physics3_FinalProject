#include <glew.h>
#include <iostream>
#include <SDL.h>
#include "Debug.h"
#include "Scene0.h"
#include <MMath.h>
#include "Debug.h"
#include "TransformComponent.h"
#include "MaterialComponent.h"
#include <QMath.h>
#include "XMLAssetManager.h"
#include "ShaderComponent.h"
#include "MeshComponent.h"
#include "ShapeComponent.h"
#include "QuadraticSolver.h"
#include "Physics.h"
#include "PhysicsComponent.h"

bool Scene0::OnCreate()
{
	XMLAssetManager assetManager;
	// Make sure these names match the stuff in your xml file:
	std::vector<std::string> names{ 
		"ActorGameBoard" , "ActorChecker1", "ActorChecker2", 
		"ActorSkull", "ActorCube", "ActorMario"
	};
	for (const auto& name : names) {
		auto asset = assetManager.xmlAssets.find(name);
		actors[name] = asset->second;
	}
	camera = std::dynamic_pointer_cast<CameraActor>(assetManager.xmlAssets.find("Camera1")->second);
	light = std::dynamic_pointer_cast<LightActor>(assetManager.xmlAssets.find("Light1")->second);
	return true;
}

void Scene0::OnDestroy()
{
	actors.clear();
}


void Scene0::HandleEvents(const SDL_Event& sdlEvent)
{
	Ref<TransformComponent> cameraTransform = camera->GetComponent <TransformComponent>();
	switch (sdlEvent.type) {
	case SDL_KEYDOWN:
		if (sdlEvent.key.keysym.scancode == SDL_SCANCODE_LEFT) {

		}
		else if (sdlEvent.key.keysym.scancode == SDL_SCANCODE_RIGHT) {

		}
		else if (sdlEvent.key.keysym.scancode == SDL_SCANCODE_UP) {

		}
		else if (sdlEvent.key.keysym.scancode == SDL_SCANCODE_DOWN) {

		}
		else if (sdlEvent.key.keysym.scancode == SDL_SCANCODE_D) {
			cameraTransform->SetTransform(cameraTransform->pos, cameraTransform->GetOrientation() * QMath::angleAxisRotation(2.0f, Vec3(0.0f, 1.0f, 0.0f)));
			camera->UpdateViewMatrix();
		}
		else if (sdlEvent.key.keysym.scancode == SDL_SCANCODE_A) {
			cameraTransform->SetTransform(cameraTransform->pos, cameraTransform->GetOrientation() * QMath::angleAxisRotation(-2.0f, Vec3(0.0f, 1.0f, 0.0f)));
			camera->UpdateViewMatrix();

		}
		else if (sdlEvent.key.keysym.scancode == SDL_SCANCODE_W) {
			cameraTransform->SetTransform(cameraTransform->pos, cameraTransform->GetOrientation() * QMath::angleAxisRotation(2.0f, Vec3(1.0f, 0.0f, 0.0f)));
			camera->UpdateViewMatrix();

		}
		else if (sdlEvent.key.keysym.scancode == SDL_SCANCODE_S) {
			cameraTransform->SetTransform(cameraTransform->pos, cameraTransform->GetOrientation() * QMath::angleAxisRotation(-2.0f, Vec3(1.0f, 0.0f, 0.0f)));
			camera->UpdateViewMatrix();

		}
		else if (sdlEvent.key.keysym.scancode == SDL_SCANCODE_M) {
			renderMeshes = !renderMeshes;
		}
		else if (sdlEvent.key.keysym.scancode == SDL_SCANCODE_C) {
			renderCollisionShapes = !renderCollisionShapes;
		}

		break;
		
	case SDL_MOUSEBUTTONDOWN:
		if (sdlEvent.button.button == SDL_BUTTON_LEFT) {
			Vec4 mouseCoords(static_cast<float>(sdlEvent.button.x), static_cast<float>(sdlEvent.button.y), 0.0f, 1.0f);
			/*std::cout << "*****************************\n";
			std::cout << "*****************************\n";
			std::cout << "*****************************\n";
			mouseCoords.print("Pixel space coords");*/

			Matrix4 ndcToPixelSpace = MMath::viewportNDC(1280, 720);
			Vec4 mouseNdcCoords = MMath::inverse(ndcToPixelSpace) * mouseCoords;
			// Set the z value to be the front plane of the NDC box
			mouseNdcCoords.z = -1.0f;
			mouseNdcCoords.print("NDC space coords");

			Matrix4 perspectiveToNdc = camera->GetProjectionMatrix();
			Vec4 mousePerspectiveCoords = MMath::inverse(perspectiveToNdc) * mouseNdcCoords;
			//mousePerspectiveCoords.print("Camera space coords before dividing by w");
			mousePerspectiveCoords /= mousePerspectiveCoords.w;
			//mousePerspectiveCoords.print("Camera space coords after dividing by w");

			Matrix4 worldToPerspective = camera->GetViewMatrix();
			Vec4 mouseWorldCoords = MMath::inverse(worldToPerspective) * mousePerspectiveCoords;
			//mouseWorldCoords.print("World space coords");

			// Right now we are getting the position at the front clipping plane
			// Sorry Scott, ray starts at the camera world position
			Vec3 rayStart = camera->GetComponent<TransformComponent>()->pos;

			// Ray direction is from the camera position to the front clipping plane
			Vec3 rayDir = mouseWorldCoords - rayStart;
			rayDir.print("ray direction");
			rayDir = VMath::normalize(rayDir);

			// TODO for Assignment 2: 
			// Get a ray pointing into the world, We have the x, y pixel coordinates
			// Need to convert this into world space to build our ray

			// Loop through all the actors and check if the ray has collided with them
			// Pick the one with the smallest positive t value
			for (auto it = actors.begin(); it != actors.end(); ++it) {
				Ref<Actor> actor = std::dynamic_pointer_cast<Actor>(it->second);
				Ref<TransformComponent> transformComponent = actor->GetComponent <TransformComponent>();
				Ref<ShapeComponent> shapeComponent = actor->GetComponent <ShapeComponent>();
				// Transform the ray into the local space of the object and check if a collision occured
				// All our ray-intersections actually happen in Paul Neale space (also known as object space)
				Matrix4 worldToPaulNealeSpace = MMath::inverse(actor->GetModelMatrix());

				Vec4 rayStartPaulNealeSpace = worldToPaulNealeSpace * Vec4(rayStart, 1.0f);
				Vec4 rayDirPaulNealeSpace = worldToPaulNealeSpace * Vec4(rayDir, 0.0f);
				GEOMETRY::Ray rayPaulNealeSpace(rayStartPaulNealeSpace, rayDirPaulNealeSpace);

				 rayInfo = shapeComponent->shape->rayIntersectionInfo(rayPaulNealeSpace);
				if (rayInfo.isIntersected) {
					std::cout << "You picked: " << it->first << '\n';
					intersectionPoint = actor->GetModelMatrix() * rayInfo.intersectionPoint;

					pickedActor = actor; // make a member variable called pickedActor. Will come in handy later…
					haveClickedOnSomething = true; // make this a member variable too. Set it to false before we loop over each actor
				}
			}
		}
		break;

	default:
		break;
	}

}

void Scene0::Update(const float deltaTime)
{
	if (haveClickedOnSomething) {
		Ref<PhysicsComponent> body = pickedActor->GetComponent<PhysicsComponent>();
		// Set up gravity and drag forces
		Vec3 gravityForce(0.0f, -0.8f * body->mass, 0.0f);
		float dragCoeff = 0.25f;
		Vec3 dragForce = body->vel * (-dragCoeff);
		Vec3 netForce = gravityForce + dragForce;
		PHYSICS::ApplyForce(body, netForce);
		// We are going to update the position and velocity in separate steps
		// This will make our constrained motion a bit easier later on
		// Calculates a first approximation of the velocity
		PHYSICS::UpdateVel(body, deltaTime);
		//Constraints frontier
	
		//// Use constraint to correct velocity errors
		//PHYSICS::MouseConstraint(body, deltaTime, intersectionPoint);
		//
		//// Update position using corrected velocities
		//body->pos += body->vel * deltaTime;
		//// We can rotate too with the mouse constraint, so update orientation too
		//Quaternion angularVelQuaternion(0.0f, body->angularVel);
		//body->orientation = body->orientation + angularVelQuaternion * body->orientation * 0.5f * deltaTime;
		//body->orientation = QMath::normalize(body->orientation); 


		//
		{
			// Correct the velocity to follow the constraint
			float slope = 1.0f;
			float yIntercept = 5.0f;
			//		PHYSICS::StraightLineConstraint(body, deltaTime, slope, yIntercept);
					// Roberts wants a plane with the normal along the diagonal
			Vec3 planeNormal = VMath::normalize(Vec3(0.0f, 1.0f, 1.0f));
			float planeD = 5.0f;
			// Let's code up the d value of the plane associated with the game board
			Ref<Actor> gameBoard = std::dynamic_pointer_cast<Actor>(actors["ActorGameBoard"]);
			Vec3 pointOnPlane = gameBoard->GetComponent<TransformComponent>()->pos;
			planeD = VMath::dot(planeNormal, pointOnPlane);
			// Add the radius of the skull
			Ref<ShapeComponent> skullShape = pickedActor->GetComponent<ShapeComponent>();
			// Rylers says what if its not a sphere?
			if (skullShape->shapeType == ShapeType::sphere) {
				// Cast the shape into a sphere
				float r = std::dynamic_pointer_cast<GEOMETRY::Sphere>(skullShape->shape)->r;
				planeD += r;
				PHYSICS::PlaneConstraint(body, deltaTime, planeNormal, planeD);
				// Calculate the angular velocity using the velocity and radius of skull
				float angularVelMag = VMath::mag(body->vel) / r;
				// Vlad thinks we don't need to multiply by the radius below
				// as we normalize the cross product anyways. I think he's right
				Vec3 rVector = r * planeNormal;
				Vec3 axisOfRotation = VMath::normalize(VMath::cross(rVector, body->vel));
				// There seems to be a bug in angularVelMag
				body->angularVel = angularVelMag * axisOfRotation;
			}


		}


		//Constraints end
		// Update position using the corrected velocity
		PHYSICS::UpdatePos(body, deltaTime);
		PHYSICS::UpdateOrientation(body, deltaTime);
		// Ensure the actor’s transform component matches the physics component
		PHYSICS::UpdateTransform(pickedActor);
	}
}

void Scene0::Render() const
{
	glEnable(GL_DEPTH_TEST);
	glClearColor(0.5f, 0.5f, 0.5f, 0.0f);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glBindBuffer(GL_UNIFORM_BUFFER, camera->GetMatricesID());
	glBindBuffer(GL_UNIFORM_BUFFER, light->GetLightID());
	// Let it go
	glBindTexture(GL_TEXTURE_2D, 0);

	for (auto it = actors.begin(); it != actors.end(); ++it) {
		Ref<Actor> actor = std::dynamic_pointer_cast<Actor>(it->second);
		glUseProgram(actor->GetComponent<ShaderComponent>()->GetProgram());
		glUniformMatrix4fv(actor->GetComponent<ShaderComponent>()->GetUniformID("modelMatrix"), 1, GL_FALSE, actor->GetModelMatrix());
		glBindTexture(GL_TEXTURE_2D, actor->GetComponent<MaterialComponent>()->getTextureID());
		if (renderMeshes) {
			actor->GetComponent<MeshComponent>()->Render(GL_TRIANGLES);
		}
		if (renderCollisionShapes) {
			// Drawing the primitive geometry associated with the mesh to help debug ray intersects, culling, and collision detection
			actor->GetComponent<ShapeComponent>()->Render();
		}
	}
}

