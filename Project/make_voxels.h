#include <set>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include "list3d.h"

class Voxel {
public:
    Eigen::Vector3d center;
    double distanceToMesh; // Distance to the closest face on the mesh
    int xIdx, yIdx, zIdx; // Location of the voxel within the grid
    std::set<int> cornerIndices; // Indices to corner vertices of the voxel
    bool isFilled; // For carving algorithm
    bool isInner; // Is the voxel inside the given mesh

    Voxel() 
        : center(Eigen::Vector3d::Zero()), distanceToMesh(0), 
          xIdx(-1), yIdx(-1), zIdx(-1), 
          isFilled(true), isInner(false) {}
};

class InnerVoidMesh {
private:
    List3d<Voxel> innerMesh;
    Eigen::MatrixXd planeV; // Plane vertices
    Eigen::MatrixXi planeF; // Plane faces
    Eigen::Vector3d gravity;
    Eigen::Vector3d balancePoint;
    Eigen::Vector3i dimensions; // Dimensions in voxels
    Eigen::Vector3d voxelSize;
    Eigen::MatrixXd corners; // Corners of all voxels
    int voxelsDisplaying;
    Eigen::Vector3d targetCOM; // Optimal center of mass for the object to balance
    Eigen::Vector3d currCOM; // Center of mass after carving the object
    double mass;
    int iterations;

public:
    // Constructor
    InnerVoidMesh(
        const Eigen::MatrixXd &V, const Eigen::MatrixXi &F, 
        const Eigen::MatrixXd &planeV, const Eigen::MatrixXi &planeF, 
        const Eigen::Vector3d &gravity, 
        const Eigen::Vector3d &balancePoint)
        : planeV(planeV), planeF(planeF), gravity(gravity), 
          balancePoint(balancePoint), iterations(0), mass(0), 
          voxelsDisplaying(0)

    void convertToMesh(Eigen::MatrixXd &V, Eigen::MatrixXi &F) {
    }

    void carveInnerMeshStep(
        Eigen::MatrixXd &carvePlaneV, 
        Eigen::MatrixXi &carvePlaneF, 
        Eigen::Vector3d &com) 

    double getCoMDistance() {
        return (currCOM - targetCOM).norm();
    }

};
