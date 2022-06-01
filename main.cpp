#include "viewer/SimViewer.h"
#include <iostream>

int main(int argc, char *argv[])
{
    SimViewer app;

    std::cout << "Launching Rigid Body Simulation tutorial" << std::endl;

    app.start();
    return 0;
}
