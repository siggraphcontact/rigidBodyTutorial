#pragma once

/**
 * @file SimViewer.h
 *
 * @brief Viewer for a cloth simulation application.
 *
 */

#include <Eigen/Dense>
#include <vector>

namespace polyscope
{
    class SurfaceMesh;
    class PointCloud;
}

class Contact;
class RigidBodySystem;
class RigidBody;

class SimViewer 
{
public:
    SimViewer();
    virtual ~SimViewer();

    void start();

private:
    void setTimestep(float);
    void setSubsteps(int);
    void setMaxIterations(int);
    void setFrictionCoefficient(float);
    void setContactStiffness(float);
    void setContactDamping(float);
    void setPaused(bool);
    void stepOnce();

    void createMarbleBox();
    void createSphereOnBox();

    void draw();
    void drawGUI();

    void preStep(std::vector<RigidBody*>&);

private:

    // Simulation parameters
    float m_dt;                         //< Time step parameter.
    int m_subSteps;
    bool m_paused;                      //< Pause the simulation.
    bool m_stepOnce;                    //< Advance the simulation by one frame and then stop.

    std::unique_ptr<RigidBodySystem> m_rigidBodySystem;
};
