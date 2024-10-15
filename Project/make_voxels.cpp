#include <stdio.h>
#include <iostream>
#include <vector>
#include <set>
#include <tuple>
#include <cmath>
#include <algorithm>
#include <cassert>
#include <limits>

#include <igl/bounding_box.h>
#include <igl/voxel_grid.h>
#include <igl/grid.h>
#include <igl/signed_distance.h>
#include <igl/per_face_normals.h>
#include <igl/centroid.h>

#include "make_voxels.h"
#include "utils.h"

const int MAX_ITER = 20;
const int MAX_CARVE = 1000; 
const double MAX_EMPTY = 0.80;

int getBinIdx(double min, double max, double binSize, double location) {
    assert(location >= min && location <= max);
    return std::floor((location - min) / binSize);
}

void getNeighborsIdx(double min, double max, double binSize, double border, double epsilon, std::vector<int>& out) {
    assert(border >= min - epsilon && border <= max + epsilon);
    int borderIdx = std::round((border - min) / binSize);

    assert(std::abs(border - (min + (borderIdx) * binSize)) < epsilon);
    int maxIdx = std::round((max - min) / binSize);
    if (borderIdx == maxIdx) {
        out[0] = borderIdx - 1;
        out[1] = -1;
    } else if (borderIdx == 0) {
        out[0] = 0;
        out[1] = -1;
    } else {
        out[0] = borderIdx - 1;
        out[1] = borderIdx;
    }
}

class InnerVoidMesh {
public:
    InnerVoidMesh(const Eigen::MatrixXd& V, const Eigen::MatrixXi& F, 
                  const Eigen::MatrixXd& planeV, const Eigen::MatrixXi& planeF, 
                  const Eigen::Vector3d& gravity, const Eigen::Vector3d& balancePoint);

    void convertToMesh(Eigen::MatrixXd& V, Eigen::MatrixXi& F);
    void carveInnerMeshStep(Eigen::MatrixXd& carvePlaneV, Eigen::MatrixXi& carvePlaneF, Eigen::Vector3d& com);
    void carveInnerMesh();

    bool isOptimized();
    double getCoMDistance();

private:
    Eigen::MatrixXd planeV, corners;
    Eigen::MatrixXi planeF;
    Eigen::Vector3d gravity, targetCOM, balancePoint, currCOM;
    List3d<Voxel> innerMesh;
    Eigen::Vector3d voxelSize;
    Eigen::Vector3i dimensions;
    double mass;
    int voxelsDisplaying, iterations;

    bool shouldDisplayVoxel(const Voxel& voxel);
    void appendCommonSide(const Voxel& voxel1, const Voxel& voxel2, const Eigen::MatrixXd& V, Eigen::MatrixXi& outF, bool onlyNecessary = true);
    void updateCenterOfMass(const Voxel& insertVoxel, const Voxel& neighbor, const Eigen::MatrixXd& V, double mass, Eigen::Vector3d& globalCom);
};

InnerVoidMesh::InnerVoidMesh(const Eigen::MatrixXd& V, const Eigen::MatrixXi& F, 
                             const Eigen::MatrixXd& planeV, const Eigen::MatrixXi& planeF, 
                             const Eigen::Vector3d& gravity, const Eigen::Vector3d& balancePoint)
    : planeV(planeV), planeF(planeF), gravity(gravity), 
      targetCOM(balancePoint.transpose()), balancePoint(balancePoint), 
      innerMesh(List3d<Voxel>()), voxelsDisplaying(0), iterations(0) {

    mass = center_of_mass(V, F, currCOM);
    Eigen::MatrixXd centers;
    createVoxelGrid(V, centers, corners, dimensions, voxelSize);

    Eigen::AlignedBox3d gridBounds;
    createAlignedBox(corners, gridBounds);

    double minX = corners.col(0).minCoeff();
    double maxX = corners.col(0).maxCoeff();
    double minY = corners.col(1).minCoeff();
    double maxY = corners.col(1).maxCoeff();
    double minZ = corners.col(2).minCoeff();
    double maxZ = corners.col(2).maxCoeff();

    Eigen::VectorXd distances, closestFaces;
    Eigen::MatrixXd closestPoints, closestNormals;
    igl::signed_distance(centers, V, F, igl::SIGNED_DISTANCE_TYPE_FAST_WINDING_NUMBER, 
                         std::numeric_limits<double>::min(), std::numeric_limits<double>::max(), 
                         distances, closestFaces, closestPoints, closestNormals);

    innerMesh.resize(dimensions(0), dimensions(1), dimensions(2));

    for (int i = 0; i < centers.rows(); i++) {
        Eigen::RowVector3d currCenter = centers.row(i);
        int xIdx = getBinIdx(minX, maxX, voxelSize(0), currCenter(0));
        int yIdx = getBinIdx(minY, maxY, voxelSize(1), currCenter(1));
        int zIdx = getBinIdx(minZ, maxZ, voxelSize(2), currCenter(2));

        innerMesh(xIdx, yIdx, zIdx) = {currCenter, xIdx, yIdx, zIdx, distances(i)};

        if (distances(i) < 0.0 && std::abs(distances(i)) >= voxelSize(0)) {
            if (xIdx > 0 && xIdx < dimensions(0) - 1 && yIdx > 0 && yIdx < dimensions(1) - 1 && zIdx > 0 && zIdx < dimensions(2) - 1) {
                innerMesh(xIdx, yIdx, zIdx).isInner = true;
            }
        }
    }

    double epsilon = voxelSize(0) / 10.0;
    std::vector<int> xNeighs(2), yNeighs(2), zNeighs(2);
    for (int rowIdx = 0; rowIdx < corners.rows(); rowIdx++) {
        Eigen::RowVector3d currCorner = corners.row(rowIdx);
        getNeighborsIdx(minX, maxX, voxelSize(0), currCorner(0), epsilon, xNeighs);
        getNeighborsIdx(minY, maxY, voxelSize(1), currCorner(1), epsilon, yNeighs);
        getNeighborsIdx(minZ, maxZ, voxelSize(2), currCorner(2), epsilon, zNeighs);

        for (int i = 0; i < 2; i++) {
            for (int j = 0; j < 2; j++) {
                for (int k = 0; k < 2; k++) {
                    if (xNeighs[i] != -1 && yNeighs[j] != -1 && zNeighs[k] != -1) {
                        innerMesh(xNeighs[i], yNeighs[j], zNeighs[k]).cornerIndices.insert(rowIdx);
                    }
                }
            }
        }
    }
}

bool InnerVoidMesh::shouldDisplayVoxel(const Voxel& voxel) {
    return voxel.isInner && !voxel.isFilled;
}

void InnerVoidMesh::appendCommonSide(const Voxel& voxel1, const Voxel& voxel2, const Eigen::MatrixXd& V, Eigen::MatrixXi& outF, bool onlyNecessary) {
    if (onlyNecessary && !shouldDisplayVoxel(voxel1) && !shouldDisplayVoxel(voxel2)) return;

    Voxel insideVoxel = shouldDisplayVoxel(voxel1) ? voxel1 : voxel2;

    std::vector<int> intersect(4);
    std::set_intersection(voxel1.cornerIndices.begin(), voxel1.cornerIndices.end(), 
                          voxel2.cornerIndices.begin(), voxel2.cornerIndices.end(), 
                          intersect.begin());

    assert(intersect.size() == 4);

    Eigen::MatrixXi sideF(2, 3);
    sideF << intersect[0], intersect[1], intersect[2], intersect[2], intersect[1], intersect[3];

    Eigen::MatrixXd N;
    igl::per_face_normals(V, sideF, N);
    assert(N.row(0).dot(N.row(1)) > 0);

    Eigen::RowVector3d vectToSide = V.row(sideF(0, 0)) - insideVoxel.center.transpose();
    if (N.row(0).dot(vectToSide) > 0) {
        sideF << intersect[2], intersect[1], intersect[0], intersect[3], intersect[1], intersect[2];
    }

    outF.conservativeResize(outF.rows() + 2, outF.cols());
    outF.bottomRows(2) = sideF;
}

void InnerVoidMesh::convertToMesh(Eigen::MatrixXd& V, Eigen::MatrixXi& F) {
    V = corners;

    F.resize(0, 3);
    for (int i = 1; i < dimensions(0) - 1; ++i) {
        for (int j = 1; j < dimensions(1) - 1; ++j) {
            for (int k = 1; k < dimensions(2) - 1; ++k) {
                Voxel currVoxel = innerMesh(i, j, k);
                if (!shouldDisplayVoxel(currVoxel)) continue;

                for (int iNeigh = i - 1; iNeigh <= i + 1; iNeigh += 2) {
                    appendCommonSide(innerMesh(iNeigh, j, k), currVoxel, V, F, false);
                }
                for (int jNeigh = j - 1; jNeigh <= j + 1; jNeigh += 2) {
                    appendCommonSide(innerMesh(i, jNeigh, k), currVoxel, V, F, false);
                }
                for (int kNeigh = k - 1; kNeigh <= k + 1; kNeigh += 2) {
                    appendCommonSide(innerMesh(i, j, kNeigh), currVoxel, V, F, false);
                }
            }
        }
    }
}

void InnerVoidMesh::updateCenterOfMass(const Voxel& insertVoxel, const Voxel& neighbor, const Eigen::MatrixXd& V, double mass, Eigen::Vector3d& globalCom) {
    double boxMass = insertVoxel.volume() * std::abs(insertVoxel.signedDistance);
    globalCom = (globalCom * mass - insertVoxel.signedDistance * boxMass * insertVoxel.center) / (mass - boxMass);
    mass -= boxMass;

    if (mass < 0) {
        throw std::invalid_argument("Mass has become negative.");
    }
}

void InnerVoidMesh::carveInnerMeshStep(Eigen::MatrixXd& carvePlaneV, Eigen::MatrixXi& carvePlaneF, Eigen::Vector3d& com) {
    Eigen::Vector3d oldCOM = com;
    bool wasCarved = false;
    for (int i = 1; i < dimensions(0) - 1; ++i) {
        for (int j = 1; j < dimensions(1) - 1; ++j) {
            for (int k = 1; k < dimensions(2) - 1; ++k) {
                Voxel& currVoxel = innerMesh(i, j, k);
                if (currVoxel.isInner && !currVoxel.isFilled) {
                    wasCarved = true;
                    currVoxel.isFilled = true;
                    ++voxelsDisplaying;
                    updateCenterOfMass(currVoxel, currVoxel, carvePlaneV, mass, com);
                }
            }
        }
    }
    if (!wasCarved) throw std::runtime_error("No voxels to carve.");

    std::vector<int> updatedVoxelIndices;
    for (int i = 1; i < dimensions(0) - 1; ++i) {
        for (int j = 1; j < dimensions(1) - 1; ++j) {
            for (int k = 1; k < dimensions(2) - 1; ++k) {
                if (shouldDisplayVoxel(innerMesh(i, j, k))) {
                    updatedVoxelIndices.push_back(i);
                    updatedVoxelIndices.push_back(j);
                    updatedVoxelIndices.push_back(k);
                }
            }
        }
    }
}

void InnerVoidMesh::carveInnerMesh() {
    iterations = 0;
    while (!isOptimized() && iterations < MAX_ITER) {
        ++iterations;
        Eigen::MatrixXd carvePlaneV;
        Eigen::MatrixXi carvePlaneF;
        carveInnerMeshStep(carvePlaneV, carvePlaneF, currCOM);

        if (voxelsDisplaying > MAX_CARVE) {
            break;
        }

        if (static_cast<double>(voxelsDisplaying) / (dimensions(0) * dimensions(1) * dimensions(2)) > MAX_EMPTY) {
            break;
        }
    }
}

bool InnerVoidMesh::isOptimized() {
    return (currCOM - targetCOM).norm() < 1e-6;
}

double InnerVoidMesh::getCoMDistance() {
    return (currCOM - targetCOM).norm();
}
