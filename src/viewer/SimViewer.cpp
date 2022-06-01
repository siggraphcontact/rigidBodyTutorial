#include "viewer/SimViewer.h"

#include "polyscope/polyscope.h"
#include "polyscope/surface_mesh.h"
#include "polyscope/point_cloud.h"
#include "polyscope/pick.h"
#include "polyscope/view.h"
#include "imgui.h"

#include <iostream>
#include <functional>

#include "contact/Contact.h"
#include "rigidbody/RigidBodySystem.h"
#include "rigidbody/Scenarios.h"

using namespace std;

namespace
{
    Eigen::Vector3f sImpulseDirs[8] = {
        Eigen::Vector3f(1, 0, 0),
        Eigen::Vector3f(0, 0, -1),
        Eigen::Vector3f(-1, 0, 0),
        Eigen::Vector3f(0, 0, 1),
        Eigen::Vector3f(0.71f, 0, 0.71f),
        Eigen::Vector3f(0.71f, 0, -0.71f),
        Eigen::Vector3f(-0.71f, 0, 0.71f),
        Eigen::Vector3f(-0.71f, 0, -0.71f)
    };

    static void updateRigidBodyMeshes(RigidBodySystem& _rigidBodySystem)
    {
        auto& bodies = _rigidBodySystem.getBodies();
        for(unsigned int i = 0; i < bodies.size(); ++i)
        { 
            Eigen::Isometry3f tm = Eigen::Isometry3f::Identity();
            tm.linear() = bodies[i]->q.toRotationMatrix();
            tm.translation() = bodies[i]->x;
            bodies[i]->mesh->setTransform( glm::make_mat4x4(tm.data()) );
        }
    }

    static void updateContactPoints(RigidBodySystem& _rigidBodySystem)
    {
        const auto& contacts = _rigidBodySystem.getContacts();
        const unsigned int numContacts = contacts.size();
        Eigen::MatrixXf contactP(numContacts, 3);
        Eigen::MatrixXf contactN(numContacts, 3);

        for (unsigned int i = 0; i < numContacts; ++i)
        {
            contactP.row(i) = contacts[i]->p.transpose();
            contactN.row(i) = contacts[i]->n.transpose();
        }

        auto pointCloud = polyscope::registerPointCloud("contacts", contactP);
        pointCloud->setPointColor({ 1.0f, 0.0f, 0.0f });
        pointCloud->setPointRadius(0.005);
        pointCloud->addVectorQuantity("normal", contactN)->setVectorColor({ 1.0f, 1.0f, 0.0f })->setVectorLengthScale(0.05f)->setEnabled(true);

    }

}

SimViewer::SimViewer() : 
    m_dt(0.0167f), m_subSteps(1),
    m_paused(true), m_stepOnce(false),
    m_rigidBodySystem()
{
}

SimViewer::~SimViewer()
{
}

void SimViewer::start()
{
    // Create the rigid body system and renderer
    m_rigidBodySystem.reset(new RigidBodySystem);

    // Setup Polyscope
    polyscope::options::programName = "Rigid Body Tutorial";
    polyscope::options::verbosity = 0;
    polyscope::options::usePrefsFile = false;
    polyscope::options::alwaysRedraw = true;
    polyscope::options::openImGuiWindowForUserCallback = true;
    polyscope::options::buildGui = false;
    polyscope::options::maxFPS = 60;
    polyscope::options::groundPlaneEnabled = false;

    // initialize
    polyscope::init();

    // Specify the update callback
    polyscope::state::userCallback = std::bind(&SimViewer::draw, this);

    // Add pre-step hook.
    m_rigidBodySystem->setPreStepFunc(std::bind(&SimViewer::preStep, this, std::placeholders::_1));

    createSphereOnBox();

    // Show the window
    polyscope::show();

}

void SimViewer::drawGUI()
{
    ImGui::Text("Simulation:");
    ImGui::Checkbox("Pause", &m_paused);
    if (ImGui::Button("Step once"))
    {
        m_stepOnce = true;
    }
    ImGui::PushItemWidth(100);
    ImGui::SliderFloat("Time step", &m_dt, 0.0f, 0.1f, "%.3f");
    ImGui::SliderInt("Num. sub-steps", &m_subSteps, 1, 20, "%u");
    ImGui::SliderInt("Solver iters.", &(m_rigidBodySystem->solverIter), 1, 100, "%u");
    ImGui::SliderFloat("Friction coeff.", &(m_rigidBodySystem->mu), 0.0f, 2.0f, "%.2f");
    ImGui::PopItemWidth();

    if (ImGui::Button("Sphere on box")) {
        createSphereOnBox();
    }
    if (ImGui::Button("Marble box")) {
        createMarbleBox();
    }
}

void SimViewer::draw()
{
    drawGUI();

    if( !m_paused || m_stepOnce )
    {
        // Step the simulation.
        // The time step dt is divided by the number of sub-steps.
        //
        const float dt = m_dt / (float)m_subSteps;
        for(int i = 0; i < m_subSteps; ++i)
        {
            m_rigidBodySystem->step(dt);
        }

        updateRigidBodyMeshes(*m_rigidBodySystem);
        updateContactPoints(*m_rigidBodySystem);
       

        // Clear step-once flag.
        m_stepOnce = false;
    }
}

void SimViewer::createMarbleBox()
{
    Scenarios::createMarbleBox(*m_rigidBodySystem);
    updateRigidBodyMeshes(*m_rigidBodySystem);
    polyscope::view::resetCameraToHomeView();
}

void SimViewer::createSphereOnBox()
{
    Scenarios::createSphereOnBox(*m_rigidBodySystem);
    updateRigidBodyMeshes(*m_rigidBodySystem);
    polyscope::view::resetCameraToHomeView();
}

void SimViewer::preStep(std::vector<RigidBody*>& _bodies)
{

}
