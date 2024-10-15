#include "utils.h"

double center_of_mass(
    const Eigen::MatrixXd& V,
    const Eigen::MatrixXi& F,
    Eigen::Vector3d& center) {

    int numVertices = V.rows();
    int numFaces = F.rows();
    double totalMass = 0;
    center.setZero();

    for (int i = 0; i < numFaces; ++i) {
        Eigen::RowVector3i triangle = F.row(i);

        Eigen::RowVector3d X0 = V.row(triangle(0));
        Eigen::RowVector3d X1 = V.row(triangle(1));
        Eigen::RowVector3d X2 = V.row(triangle(2));

        Eigen::RowVector3d A = X0 - X1;
        Eigen::RowVector3d B = X0 - X2;

        double a = A.norm();
        double b = B.norm();
        double c = (X1 - X2).norm();
        double s = (a + b + c) / 2.0;
        double area = std::sqrt(s * (s - a) * (s - b) * (s - c));

        Eigen::RowVector3d N = A.cross(B).normalized();

        totalMass += 2 * area * N(0) * (X0(0) + X1(0) + X2(0)) / 6.0;
    }

    center_of_mass(V, F, totalMass, center);
    return totalMass;
}

void center_of_mass(
    const Eigen::MatrixXd& V,
    const Eigen::MatrixXi& F,
    double mass,
    Eigen::Vector3d& center) {

    int numFaces = F.rows();
    center.setZero();

    for (int i = 0; i < numFaces; ++i) {
        Eigen::RowVector3i triangle = F.row(i);

        Eigen::RowVector3d X0 = V.row(triangle(0));
        Eigen::RowVector3d X1 = V.row(triangle(1));
        Eigen::RowVector3d X2 = V.row(triangle(2));

        Eigen::RowVector3d A = X0 - X1;
        Eigen::RowVector3d B = X0 - X2;

        double a = A.norm();
        double b = B.norm();
        double c = (X1 - X2).norm();
        double s = (a + b + c) / 2.0;
        double area = std::sqrt(s * (s - a) * (s - b) * (s - c));

        Eigen::RowVector3d N = A.cross(B).normalized();

        double N1 = N(0);
        double N2 = N(1);
        double N3 = N(2);

        center(0) += area * N1 * (X0(0) * (X0(0) + X1(0) + X2(0)) + X1(0) * (X0(0) + X1(0) + X2(0)) + X2(0) * (X0(0) + X1(0) + X2(0))) / 24.0;
        center(1) += area * N2 * (X0(1) * (X0(1) + X1(1) + X2(1)) + X1(1) * (X0(1) + X1(1) + X2(1)) + X2(1) * (X0(1) + X1(1) + X2(1))) / 24.0;
        center(2) += area * N3 * (X0(2) * (X0(2) + X1(2) + X2(2)) + X1(2) * (X0(2) + X1(2) + X2(2)) + X2(2) * (X0(2) + X1(2) + X2(2))) / 24.0;
    }

    center /= mass;
}
