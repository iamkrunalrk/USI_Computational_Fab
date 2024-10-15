#include <iostream>
#include <vector>
#include <string>
#include <math.h>

#include <Eigen/Geometry>

#include <imgui/imgui.h>
#include <igl/opengl/glfw/Viewer.h>
#include <igl/unproject_onto_mesh.h>
#include <igl/opengl/glfw/imgui/ImGuiMenu.h>
#include <igl/opengl/glfw/imgui/ImGuiHelpers.h>
#include <igl/bounding_box.h>
#include <igl/grid.h>

#include "utils.h"
#include "make_voxels.h"


#define PI 3.14159265
const bool DEBUG = false;
const std::string DEFAULT_MESH_FILE = "objects/spheres.stl";

// Viewer data indices
int mesh_data_id, inner_data_mesh_id, plane_data_id, gravity_data_id, carve_plane_data_id;

// Default gravity settings
const Eigen::Vector3d DEFAULT_GRAVITY(0.0, -1.0, 0.0);
const float DEFAULT_GRAVITY_YAW = 0.0;
const float DEFAULT_GRAVITY_PITCH = 0.0;
const float DEFAULT_GRAVITY_ROLL = 180.0;

// Predefined colors
const Eigen::RowVector3d orange(1.0, 0.7, 0.2);
const Eigen::RowVector3d yellow(1.0, 0.9, 0.2);
const Eigen::RowVector3d blue(0.2, 0.3, 0.8);
const Eigen::RowVector3d green(0.2, 0.6, 0.3);
const Eigen::RowVector3d black(0.0, 0.0, 0.0);

// GUI objects
igl::opengl::glfw::Viewer viewer;
igl::opengl::glfw::imgui::ImGuiMenu menu;

// Undeformed objects
Eigen::MatrixXd origV;
Eigen::MatrixXi origF;

struct State {
    Eigen::MatrixXd V, innerV, planeV;
    Eigen::MatrixXi F, innerF, planeF;
    Eigen::RowVector3d planeCenter;
    Eigen::Vector3d gravity;
    float yaw = DEFAULT_GRAVITY_YAW;
    float pitch = DEFAULT_GRAVITY_PITCH;
    float roll = DEFAULT_GRAVITY_ROLL;
    int balancePointIdx;
    bool selectBalancePoint = true;
    bool selectOrientation = false;
    bool isCarving = false;
    InnerVoidMesh* innerMesh = nullptr;
} state;

double sin_deg(double angle) { return std::sin(angle * PI / 180.0); }
double cos_deg(double angle) { return std::cos(angle * PI / 180.0); }

Eigen::RowVector3d getBalancePoint() {
    return state.V.row(state.balancePointIdx);
}

void updateBalancePoint() {
    viewer.data(mesh_data_id).clear_points();
    viewer.data(mesh_data_id).set_points(getBalancePoint(), state.selectBalancePoint ? yellow : blue);
}

void updateGravityVector() {
    state.gravity(0) = -cos_deg(state.yaw) * sin_deg(state.pitch) * sin_deg(state.roll) - sin_deg(state.yaw) * cos_deg(state.roll);
    state.gravity(1) = -sin_deg(state.yaw) * sin_deg(state.pitch) * sin_deg(state.roll) + cos_deg(state.yaw) * cos_deg(state.roll);
    state.gravity(2) = cos_deg(state.pitch) * sin_deg(state.roll);
}

void updateGravity() {
    updateGravityVector();
    if (DEBUG) {
        viewer.data(gravity_data_id).clear_points();
        viewer.data(gravity_data_id).add_points(Eigen::RowVector3d(0.0, 0.0, 0.0), black);
        viewer.data(gravity_data_id).add_points(state.gravity.transpose() / 10.0, black);
    }
}

void updateMesh() {
    Eigen::Matrix3d R, rollR, pitchR, yawR;
    rollR <<
        1, 0, 0,
        0, cos_deg(state.roll - DEFAULT_GRAVITY_ROLL), -sin_deg(state.roll - DEFAULT_GRAVITY_ROLL),
        0, sin_deg(state.roll - DEFAULT_GRAVITY_ROLL), cos_deg(state.roll - DEFAULT_GRAVITY_ROLL);
    pitchR <<
        cos_deg(state.pitch), 0, -sin_deg(state.pitch),
        0, 1, 0,
        sin_deg(state.pitch), 0, cos_deg(state.pitch);
    yawR <<
        cos_deg(state.yaw), -sin_deg(state.yaw), 0,
        sin_deg(state.yaw), cos_deg(state.yaw), 0,
        0, 0, 1;

    state.V = origV * rollR.transpose() * pitchR.transpose() * yawR.transpose();
    Eigen::RowVector3d translate = state.planeCenter - getBalancePoint();
    state.V.rowwise() += translate;

    viewer.data(mesh_data_id).set_vertices(state.V);
    updateBalancePoint();
}

void updateInnerMesh() {
    viewer.data(inner_data_mesh_id).clear();
    viewer.data(inner_data_mesh_id).set_mesh(state.innerV, state.innerF);
}

void clearInnerMesh() {
    state.innerV.resize(0, 3);
    state.innerF.resize(0, 3);
    viewer.data(inner_data_mesh_id).clear();
    if (DEBUG) {
        viewer.data(carve_plane_data_id).clear_points();
    }
}

bool findLowestPointIdx(const Eigen::MatrixXd& V, int& lowestIdx) {
    int heightCol = 1;
    double minZ = V.col(heightCol).minCoeff();
    lowestIdx = -1;
    for (int i = 0; i < V.rows(); ++i) {
        if (V(i, heightCol) == minZ) {
            lowestIdx = i;
            break;
        }
    }
    return (lowestIdx != -1);
}

void initState(int argc, char* argv[]) {
    igl::read_triangle_mesh((argc > 1) ? argv[1] : DEFAULT_MESH_FILE, origV, origF);
    Eigen::MatrixXd unalignedV = origV;
    alignVerticesToAxis(unalignedV, origV);

    state.V = origV;
    state.F = origF;

    Eigen::AlignedBox3d boundingBox;
    createAlignedBox(origV, boundingBox);
    state.planeV.resize(4, 3);
    state.planeV <<
        boundingBox.corner(boundingBox.BottomLeftCeil).transpose(),
        boundingBox.corner(boundingBox.BottomRightCeil).transpose(),
        boundingBox.corner(boundingBox.BottomLeftFloor).transpose(),
        boundingBox.corner(boundingBox.BottomRightFloor).transpose();
    state.planeF.resize(2, 3);
    state.planeF <<
        0, 1, 2,
        2, 1, 3;

    state.planeCenter = state.planeV.colwise().sum() / 4.0;
    updateGravityVector();

    if (!findLowestPointIdx(origV, state.balancePointIdx)) {
        std::cerr << "Error: Failed to find lowest point";
        exit(-1);
    }
}

// == Callback Functions ==
bool mouse_down(igl::opengl::glfw::Viewer& viewer, int x, int y) {
    Eigen::RowVector3f last_mouse(viewer.current_mouse_x, viewer.core().viewport(3) - viewer.current_mouse_y, 0);
    if (state.selectBalancePoint) {
        int fid;
        Eigen::Vector3f bary;
        if (igl::unproject_onto_mesh(last_mouse.head(2), viewer.core().view, viewer.core().proj, viewer.core().viewport, state.V, state.F, fid, bary)) {
            long c;
            bary.maxCoeff(&c);
            state.balancePointIdx = state.F(fid, c);
            updateBalancePoint();
            return true;
        }
    }
    return false;
}

void draw_viewer_menu() {
    menu.draw_viewer_menu();
}

void draw_workflow_control_window() {
    ImGui::SetNextWindowPos(ImVec2(180.f * menu.menu_scaling(), 10), ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSize(ImVec2(250, 360), ImGuiCond_FirstUseEver);
    ImGui::Begin("CF - MakeItStand", nullptr, ImGuiWindowFlags_NoSavedSettings);

    ImGui::Text("COM Point");
    if (ImGui::Checkbox("Select COM point", &(state.selectBalancePoint))) {
        if (state.selectBalancePoint) {
            state.selectOrientation = false;
            state.isCarving = false;
            updateBalancePoint();
            clearInnerMesh();
            viewer.data(mesh_data_id).show_faces = true;
        }
    }

    if (state.selectBalancePoint && ImGui::Button("New Plane", ImVec2(-1, 0))) {
        updateMesh();
    }

    ImGui::Separator();
    ImGui::Text("Orientation of Object");
    if (ImGui::Checkbox("Change orientation", &(state.selectOrientation))) {
        if (state.selectOrientation) {
            state.selectBalancePoint = false;
            state.isCarving = false;
            clearInnerMesh();
            viewer.data(mesh_data_id).show_faces = true;
            updateMesh();
        }
    }
    if (state.selectOrientation) {
        ImGui::SliderFloat("Yaw", &(state.yaw), 0.0, 360.0);
        if (ImGui::IsItemActive()) {
            updateGravity();
            updateMesh();
        }
        ImGui::SliderFloat("Pitch", &(state.pitch), 0.0, 360.0);
        if (ImGui::IsItemActive()) {
            updateGravity();
            updateMesh();
        }
        ImGui::SliderFloat("Roll", &(state.roll), 0.0, 360.0);
        if (ImGui::IsItemActive()) {
            updateGravity();
            updateMesh();
        }
    }

    ImGui::Separator();
    ImGui::Text("Voxels");
    if (ImGui::Checkbox("start voxalization", &(state.isCarving))) {
        if (state.isCarving) {
            state.selectBalancePoint = false;
            state.selectOrientation = false;
            updateMesh();
            viewer.data(mesh_data_id).show_faces = false;
            if (state.innerMesh != nullptr) {
                delete state.innerMesh;
            }
            state.innerMesh = new InnerVoidMesh(state.V, state.F, state.planeV, state.planeF, Eigen::Vector3d(0, -1, 0), getBalancePoint());
            state.innerMesh->convertToMesh(state.innerV, state.innerF);
            updateInnerMesh();
        }
    }
    if (state.isCarving) {
        if (ImGui::Button("Start Meshing", ImVec2(-1, 0))) {
            if (!state.innerMesh->isOptimized()) {
                Eigen::MatrixXd carvePlaneV;
                Eigen::MatrixXi carvePlaneF;
                Eigen::Vector3d com;
                state.innerMesh->carveInnerMeshStep(carvePlaneV, carvePlaneF, com);
                state.innerMesh->convertToMesh(state.innerV, state.innerF);
                updateInnerMesh();

                if (DEBUG) {
                    viewer.data(carve_plane_data_id).clear();
                    viewer.data(carve_plane_data_id).set_mesh(carvePlaneV, carvePlaneF);
                    viewer.data(carve_plane_data_id).add_points(com.transpose(), orange);
                }
            }
        }
        if (ImGui::Button("Start Balancing", ImVec2(-1, 0))) {
            state.innerMesh->carveInnerMesh();
            state.innerMesh->convertToMesh(state.innerV, state.innerF);
            updateInnerMesh();
        }
        if (state.innerMesh->isOptimized()) {
            ImGui::Text("Model Balanced");
        }
    }

    ImGui::End();
}

int main(int argc, char* argv[]) {
    initState(argc, argv);

    viewer.plugins.push_back(&menu);
    menu.callback_draw_viewer_menu = draw_viewer_menu;
    menu.callback_draw_custom_window = draw_workflow_control_window;

    viewer.callback_mouse_down = mouse_down;

    mesh_data_id = viewer.data().id;
    viewer.data(mesh_data_id).set_mesh(state.V, state.F);
    updateMesh();

    viewer.append_mesh();
    inner_data_mesh_id = viewer.data().id;

    viewer.append_mesh();
    plane_data_id = viewer.data().id;
    viewer.data(plane_data_id).set_mesh(state.planeV, state.planeF);

    if (DEBUG) {
        viewer.append_mesh();
        gravity_data_id = viewer.data().id;
        updateGravity();

        viewer.append_mesh();
        carve_plane_data_id = viewer.data().id;
    }

    viewer.selected_data_index = viewer.mesh_index(mesh_data_id);
    viewer.core().align_camera_center(state.planeV, state.planeF);

    viewer.launch();
}
