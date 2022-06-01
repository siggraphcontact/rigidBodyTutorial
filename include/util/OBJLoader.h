#pragma once

#include <Eigen/Dense>

// File loading for OBJ meshes.
//
class OBJLoader
{
public:

    // Loads the mesh from the OBJ file  path @a filename and return the mesh vertices @a meshV and faces @a meshF.
    // 
    //  Note: Use the MeshAssetRegistry class in MeshAssets.h to load meshes rather than this function, 
    //    since previously loaded mesh files will be loaded from the cache, which is much faster.
    //
    static bool load(const std::string& filename, Eigen::MatrixXf& meshV, Eigen::MatrixXi& meshF);
    
};
