#pragma once

#include "RigidBody.h"
#include "RigidBodySystem.h"

#include <Eigen/Dense>

class Scenarios
{
public:
    // Box filled with balls.
    //
    static void createMarbleBox(RigidBodySystem& rigidBodySystem)
    {
        rigidBodySystem.clear();
        polyscope::removeAllStructures();

        std::cout << "Loading marble box scenario" << std::endl;

        // Create two layers of "marbles", in a grid layout
        //
        for(int i = 0; i < 9; ++i)
        {
            for(int j = 0; j < 9; ++j)
            {
                RigidBody* body1 = new RigidBody(1.0f, new Sphere(0.5f), "resources/sphere.obj");
                body1->x.x() = -4.0f + (float)i*1.0f;
                body1->x.z() = -4.0f + (float)j*1.0f;
                body1->x.y() = 2.0f;
                rigidBodySystem.addBody(body1);
                body1->mesh->setSurfaceColor({ 1.0f, 0.1f, 0.1f });
                body1->mesh->setTransparency(0.8f);
                RigidBody* body2 = new RigidBody(1.0f, new Sphere(0.5f), "resources/sphere.obj");
                body2->x.x() = -4.0f + (float)i * 1.0f;
                body2->x.z() = -4.0f + (float)j * 1.0f;
                body2->x.y() = 3.0f;
                rigidBodySystem.addBody(body2);
                body2->mesh->setSurfaceColor({ 1.0f, 0.1f, 0.1f });
                body2->mesh->setTransparency(0.8f);
            }
        }

        RigidBody* body0 = new RigidBody(1.0f, new Box(Eigen::Vector3f(0.4f, 4.0f, 10.0f)), "resources/box_side.obj");
        RigidBody* body1 = new RigidBody(1.0f, new Box(Eigen::Vector3f(0.4f, 4.0f, 10.0f)), "resources/box_side.obj");
        RigidBody* body2 = new RigidBody(1.0f, new Box(Eigen::Vector3f(0.4f, 4.0f, 10.0f)), "resources/box_side.obj");
        RigidBody* body3 = new RigidBody(1.0f, new Box(Eigen::Vector3f(0.4f, 4.0f, 10.4f)), "resources/box_side.obj");
        RigidBody* body4 = new RigidBody(1.0f, new Box(Eigen::Vector3f(10.0f, 0.4f, 10.0f)), "resources/box_bot.obj");
        body0->fixed = true;
        body1->fixed = true;
        body2->fixed = true;
        body3->fixed = true;
        body4->fixed = true;
        body0->mesh->setSurfaceColor({ 0.6f, 0.6f, 0.6f });
        body1->mesh->setSurfaceColor({ 0.6f, 0.6f, 0.6f });
        body2->mesh->setSurfaceColor({ 0.6f, 0.6f, 0.6f });
        body3->mesh->setSurfaceColor({ 0.6f, 0.6f, 0.6f });
        body4->mesh->setSurfaceColor({ 0.6f, 0.6f, 0.6f });
        body0->x.x() = 4.75f;
        body1->x.x() = -4.75f;
        body2->x.z() = 4.75f;
        body2->q = Eigen::AngleAxisf(1.57, Eigen::Vector3f(0, 1, 0));
        body3->x.z() = -4.75f;
        body3->q = Eigen::AngleAxisf(1.57, Eigen::Vector3f(0, 1, 0));
        body4->x.y() = -2.0f;

        rigidBodySystem.addBody(body0);
        rigidBodySystem.addBody(body1);
        rigidBodySystem.addBody(body2);
        rigidBodySystem.addBody(body3);
        rigidBodySystem.addBody(body4);
    }

    // Simple sphere falling on a box.
    //
	static void createSphereOnBox(RigidBodySystem& rigidBodySystem)
	{
		rigidBodySystem.clear();
		polyscope::removeAllStructures();

    std::cout << "Loading sphere-on-box scenario." << std::endl;

		// Create a sphere.
		RigidBody* bodySphere = new RigidBody(1.0f, new Sphere(0.5f), "resources/sphere.obj");
		bodySphere->x.y() = 4.0f;
    bodySphere->omega = Eigen::Vector3f(10.0f, 0.0f, 0.0f);
    bodySphere->mesh->setTransparency(0.8f);

		RigidBody* bodyBox = new RigidBody(1.0f, new Box(Eigen::Vector3f(10.0f, 0.4f, 10.0f)), "resources/box_bot.obj");
		bodyBox->fixed = true;

		rigidBodySystem.addBody(bodySphere);
		rigidBodySystem.addBody(bodyBox);

    bodySphere->mesh->setSurfaceColor({ 0.1f, 1.0f, 0.2f });
    bodyBox->mesh->setSurfaceColor({ 0.2f, 0.2f, 0.2f });
	}

};
