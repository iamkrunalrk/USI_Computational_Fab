#include <igl/bounding_box.h>
#include <igl/voxel_grid.h>
#include <igl/grid.h>
#include <igl/signed_distance.h>
#include <igl/rotation_matrix_from_directions.h>
#include "utils.h"

const int GRID_LEN = 50; // Number of voxels on each side of the grid

void createAlignedBox(const Eigen::MatrixXd &V, Eigen::AlignedBox3d &box) {
    for (int i = 0; i < V.rows(); i++) {
        box.extend(V.row(i).transpose());
    }
}

void transformVertices(const Eigen::MatrixXd &V, const Eigen::AlignedBox3d &newOrientation, bool keepSize, Eigen::MatrixXd &Vout) {
    Vout.resize(V.rows(), 3);

    Eigen::Vector3d corner1 = newOrientation.corner(Eigen::AlignedBox3d::BottomLeftFloor);
    Eigen::Vector3d corner2 = newOrientation.corner(Eigen::AlignedBox3d::TopRightCeil);

    // Compute diagonals of input and output grids
    Eigen::AlignedBox3d inBox;
    createAlignedBox(V, inBox);
    Eigen::Vector3d inDiag = inBox.diagonal();
    Eigen::Vector3d outDiag = corner2 - corner1;

    // Scale V to desired size
    Eigen::Matrix3d scale = Eigen::Matrix3d::Identity();
    if (!keepSize) {
        scale.diagonal() << outDiag.cwiseQuotient(inDiag).array();
    }
    Vout = V * scale;

    // Rotate
    Eigen::Matrix3d R = igl::rotation_matrix_from_directions(inDiag.normalized(), outDiag.normalized());
    Vout *= R.transpose();

    // Translate
    Eigen::Vector3d translate = corner1 - R * (scale * inBox.corner(Eigen::AlignedBox3d::BottomLeftFloor));
    Vout.rowwise() += translate.transpose();
}

void transformVertices(const Eigen::MatrixXd &V, const Eigen::AlignedBox3d &newOrientation, Eigen::MatrixXd &Vout) {
    transformVertices(V, newOrientation, false, Vout);
}

void createVoxelGrid(const Eigen::MatrixXd &V, Eigen::MatrixXd &centers, Eigen::MatrixXd &corners, Eigen::Vector3i &dimensions, Eigen::Vector3d &voxelSize) {
    // Compute bounding box
    Eigen::MatrixXd BV, BF;
    igl::bounding_box(V, BV, BF);
    Eigen::AlignedBox3d boundBox;
    createAlignedBox(V, boundBox);

    // Create voxel grid (centers of voxels)
    Eigen::RowVector3i dimOut;
    igl::voxel_grid(boundBox, GRID_LEN, 0, centers, dimOut);
    dimensions = dimOut.transpose();
    Eigen::AlignedBox3d centersBox;
    createAlignedBox(centers, centersBox);

    voxelSize = centersBox.sizes().cwiseQuotient((dimensions.cast<double>().array() - 1).matrix());

    // Compute corner vertices of voxel grid
    Eigen::MatrixXd defaultGrid;
    Eigen::Vector3d dimCorners = (dimensions.cast<double>().array() + 1).matrix();
    igl::grid(dimCorners, defaultGrid);

    // Scale default grid to voxel grid
    Eigen::Vector3d outBLF = centersBox.corner(Eigen::AlignedBox3d::BottomLeftFloor) - (voxelSize / 2.0);
    Eigen::Vector3d outTRC = centersBox.corner(Eigen::AlignedBox3d::TopRightCeil) + (voxelSize / 2.0);
    Eigen::AlignedBox3d outBox(outBLF, outTRC);
    transformVertices(defaultGrid, outBox, corners);
}

void alignToAxis(const Eigen::MatrixXd &V, Eigen::MatrixXd &alignedV) {
    Eigen::AlignedBox3d inputBox;
    createAlignedBox(V, inputBox);

    Eigen::MatrixXd alignedGrid;
    Eigen::Vector3d dim = Eigen::Vector3d::Constant(10.0);
    igl::grid(dim, alignedGrid);
    Eigen::AlignedBox3d alignedBox;
    createAlignedBox(alignedGrid, alignedBox);

    transformVertices(V, alignedBox, true, alignedV);
}
