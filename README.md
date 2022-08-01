# SIGGRAPH'22 Course on Contact and Friction Simulation for Computer Graphics

## Tutorial: Rgid Body Simulation with Contact and Friction


![Rigid body sim](https://siggraphcontact.github.io/assets/images/rigidbodytut.png "Rigid body sim")

A tutorial on implementing a simple C++ rigid body simulator.

The tutorial follows Appendix A from the [SIGGRAPH'22 Course](https://siggraphcontact.github.io/) on contact and friction simulation for computer graphics.

CMake is used to build the main application.  To build and run the code, first create a sub-directory: 

```    
> mkdir build
```

Then, configure the project for the specific build target:

```
> cd build
> cmake -DCMAKE_BUILD_TYPE=Debug ..
```

Finally, compile using your favorite build tool or IDE, e.g.
```
> make
```

The code has been compiled and tested on Windows (MS Visual C++ 2019), Ubuntu Linux (g++ 9.3 with VS Code), and MacOS (Clang)


## Dependencies

The following dependencies are included and compiled automatically (see the **3rdParty** directory):

 * [Eigen](https://eigen.tuxfamily.org/)
 * [Polyscope](https://github.com/nmwsharp/polyscope)

## Source code

### solvers/

Implementations of a matrix-free Projected Gauss-Seidel (PGS) solver for boxed LCPs (see **SolverBoxPGS.h/.cpp**)

### collision/

The **CollisionDetect.h** class handles collision detection and contact generation for all pairs of simulation bodies. For this tutorial, the following geometries are supported:
 * Spheres
 * Boxes

And you will implemented collision handling for the following geometry pairs:
  * Sphere-sphere
  * Sphere-box
  
### contact/

Implementation of the box friction cone model (see **Contact.h/.cpp**).
