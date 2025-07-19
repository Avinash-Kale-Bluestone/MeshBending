#pragma once
#define _USE_MATH_DEFINES
// === Libraries ===
#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <imgui.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl3.h>
#include <assimp/Importer.hpp>
#include <assimp/scene.h>
#include <assimp/postprocess.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>


// === Standard Libraries ===
#include <iostream>.
#include <fstream>
#include <vector>
#include <set>
#include <unordered_set>
#include <map>
#include <limits>
#include <cmath>

// ===  Eigen + libigl Poisson solve ===
#include <Eigen/Core>
#include <Eigen/Sparse>
#include <igl/grad.h>
//#include <igl/div.h>
#include <igl/cotmatrix.h>
#include <igl/min_quad_with_fixed.h>
#include <igl/connected_components.h>


//CGAL
#include <CGAL/Simple_cartesian.h>
#include <CGAL/Polyhedron_3.h>
#include <CGAL/Polyhedron_items_with_id_3.h>
#include <CGAL/Surface_mesh_deformation.h>
#include <CGAL/IO/STL/STL_reader.h>
#include <CGAL/Polygon_mesh_processing/smooth_shape.h>
#include <CGAL/Polygon_mesh_processing/IO/polygon_mesh_io.h>
#include <CGAL/IO/STL.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/boost/graph/graph_traits_Surface_mesh.h>
#include <CGAL/subdivision_method_3.h>
#include <CGAL/Timer.h>

#include <boost/lexical_cast.hpp>



//// struct & Enum
struct Vertex {
    float x, y, z;
};



class CQuaternion
{
public:
    double w;
    double x;
    double y;
    double z;

public:
    CQuaternion();
    //~CQuaternion();

    CQuaternion(double a, double b, double c, double d);
    CQuaternion  operator +(const CQuaternion&);
    CQuaternion  operator -(const CQuaternion&);
    CQuaternion  operator *(const double&);
    CQuaternion  operator /(const double&);
    CQuaternion  operator *(const CQuaternion&);
    void R2Q(const double&, const double&, const double&, const double&);
    void Q2R(Eigen::Matrix4d& q_r);
    int Sign(double x) { return (x > 0) ? 1 : -1; };
    void GetInverse();
    double GetNorm();
    void Normalize();
    double DotProduct(const CQuaternion& q);
    void RotationMatrix2Qua(const Eigen::Matrix4d& mat);
    CQuaternion& Slerp(CQuaternion& qua1, CQuaternion& qua2, double t);

};














//Global Variables
//Model & Deformation Data
std::set<int> handle_indices, fixed_indices, smooth_indices;

//Selection State
bool selecting_handle = false, selecting_fixed = false, selecting_smooth = false, smooth_select_region = false, selection_done = false;
bool handle_select_vertices = false, handle_select_region = false;// Nested options
bool fixed_select_vertices = false, fixed_select_region = false;
bool show_handles = true, show_fixed = true, show_region = true;

//
std::vector<ImVec2> region_polygon; // Region polygon (2D screen space)
std::vector<glm::vec2> selectionPolygon;
bool drawing_region = false, region_selection_done = false, selecting_handle_region = false, selecting_fixed_region = false;// Selection state

// Handle Manipulation
bool mouseControlledDeform = false;
bool dragging_handle = false;
int dragged_handle = -1;
float tx = 0.0f, ty = 0.0f, tz = 0.0f;


//rotation
bool rotation_enabled = false;
bool boundingBoxVisible = false;
glm::vec3 bboxCenter(0.0f);
glm::vec3 bboxMin, bboxMax;
float rotationAngleY = 0.0f;
float rotationAngleX = 0.0f;
float rotationAngleZ = 0.0f;


//smooth
bool smoothSelectedMeshArea = false;


//Camera
enum CameraProjectionMode { PERSPECTIVE, ORTHOGRAPHIC };
enum CameraControlMode { ORBIT, FREE, MODEL_VIEW };
enum RenderMode { SHADED, WIREFRAME, BOTH, VERTICES };
float modelScale = 1.0f;
glm::vec3 modelOffset(0.0f);
float modelRotationAngle = 0.0f;
glm::vec3 modelRotationAxis(0.0f, 1.0f, 0.0f);
float cameraDistance = 5.0f;
float cameraYaw = 0.0f, cameraPitch = 0.0f;
float lastX = 0.0f, lastY = 0.0f;
bool rotating = false;
glm::vec3 cameraTarget(0.0f);
CameraProjectionMode cameraMode = PERSPECTIVE;
CameraControlMode controlMode = ORBIT;
RenderMode renderMode;
static bool showCameraWindow = false;


// mainwindow
GLFWwindow* window = nullptr;
bool loaded = false;


std::vector<std::array<int, 3>> triangleIndices;  // Indices for each triangle in the mesh
bool is_dragging = false;
bool mouse_released = false;
glm::vec3 drag_start_pos;

//cgal
typedef CGAL::Simple_cartesian<double> Kernel;
typedef Kernel::Point_3 Point;
typedef CGAL::Polyhedron_3<Kernel, CGAL::Polyhedron_items_with_id_3> Polyhedron;
typedef boost::graph_traits<Polyhedron>::vertex_descriptor vertex_descriptor;
typedef CGAL::Surface_mesh_deformation<Polyhedron, CGAL::Default, CGAL::Default, CGAL::SRE_ARAP> Surface_mesh_deformation;
Polyhedron mesh,demesh;
std::unique_ptr<Surface_mesh_deformation> deform, deformed_model;
std::unordered_map<int, Kernel::Point_3> original_positions;
using Vertex_iterator = Polyhedron::Vertex_iterator;
typedef Polyhedron::Facet_iterator Facet_iterator;
typedef Polyhedron::Halfedge_around_facet_circulator Halfedge_facet_circulator;
typedef Polyhedron::Facet_const_iterator Facet_const_iterator;
typedef Polyhedron::Halfedge_around_facet_const_circulator HFCC;
typedef Polyhedron::Halfedge_around_vertex_circulator      HV_circulator;


//////////////////////////////////////////////////////////////////////////////////Utility functions/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//glm::mat4 getProjectionMatrix(int width, int height) {
//    float aspect = (float)width / height;
//    if (cameraMode == PERSPECTIVE) {
//        if (height == 0) height = 1;
//
//        return glm::perspective(glm::radians(45.0f), aspect, 0.1f, 100.0f);
//    }
//    else {
//        float orthoScale = 2.0f;
//        return glm::ortho(-orthoScale * aspect, orthoScale * aspect, -orthoScale, orthoScale, 0.1f, 100.0f);
//    }
//}
//
//glm::mat4 getViewMatrix() {
//    if (controlMode == ORBIT) {
//        float yawRad = glm::radians(cameraYaw);
//        float pitchRad = glm::radians(cameraPitch);
//        glm::vec3 direction = {
//            cos(yawRad) * cos(pitchRad),
//            sin(pitchRad),
//            sin(yawRad) * cos(pitchRad)
//        };
//        glm::vec3 cameraPos = cameraTarget - direction * cameraDistance;
//        return glm::lookAt(cameraPos, cameraTarget, glm::vec3(0, 1, 0));
//    }
//    else if (controlMode == FREE) {
//        return glm::lookAt(glm::vec3(0, 0, cameraDistance), glm::vec3(0, 0, 0), glm::vec3(0, 1, 0));
//    }
//    else if (controlMode == MODEL_VIEW) {
//        // Fixed camera; move model instead
//        return glm::lookAt(glm::vec3(0, 0, cameraDistance), glm::vec3(0, 0, 0), glm::vec3(0, 1, 0));
//    }
//    return glm::mat4(1.0f);
//}

bool pointInPolygon(const glm::vec2& pt, const std::vector<glm::vec2>& poly) {
    int count = 0;
    for (size_t i = 0; i < poly.size(); ++i) {
        glm::vec2 a = poly[i];
        glm::vec2 b = poly[(i + 1) % poly.size()];
        if (((a.y > pt.y) != (b.y > pt.y)) &&
            (pt.x < (b.x - a.x) * (pt.y - a.y) / ((b.y - a.y) + 1e-5f) + a.x)) {
            count++;
        }
    }
    return (count % 2) == 1;
}

std::set<int> getDeformRegion(int totalVertices) {// Compute region as all vertices not in handle or fixed
    std::set<int> region;
    for (int i = 0; i < totalVertices; ++i) {
        if (handle_indices.count(i) == 0 && fixed_indices.count(i) == 0)
            region.insert(i);
    }
    return region;
}

void loadSTLMesh(const std::string& path) {
    if (!mesh.empty()) {
        
        deform->reset();
        mesh.clear();
        handle_indices.clear();
        fixed_indices.clear();
    }

    std::ifstream input(path, std::ios::binary);
    if (!input || !CGAL::IO::read_STL(input, mesh)) {
        std::cerr << "Error: Cannot read STL file: " << path << std::endl;

    }
    input.close();
   
    for (auto v = mesh.vertices_begin(); v != mesh.vertices_end(); ++v) {
        original_positions[v->id()] = v->point();
    }
    
    //deformation arap
    // Initialize IDs
    CGAL::set_halfedgeds_items_id(mesh);
    deform = std::make_unique<Surface_mesh_deformation>(mesh);
    deform->set_sre_arap_alpha(10);
   
    
    std::cout << "Loaded mesh: " << mesh.size_of_vertices() << " vertices, " << mesh.size_of_facets() << " triangles\n";

}
//////////////////////////////////////////////////////////////////////////////////Utility functions/////////////////////////////////////////////////////////////////////////////////////////////////////////////////



/////////////////////////////////////////////////////////////////////////////////forward decl functions//////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//model view
void differentViewsModes(RenderMode mode);
void drawSelectionPloygon();
void drawSelectedRegion();
void drawMouseControlledDeform();
void drawModel(RenderMode mode);

//mouse movement utility
glm::mat4 getProjectionMatrix(GLFWwindow* window);
glm::mat4 getViewMatrix();
glm::vec2 worldToScreen(const glm::vec3& worldPos, const glm::mat4& projection, const glm::mat4& view, int width, int height);

//mouse movement
void key_callback(GLFWwindow* window, int key, int scancode, int action, int mods);
void mouse_button_callback(GLFWwindow* window, int button, int action, int mods);
void cursor_position_callback(GLFWwindow* window, double xpos, double ypos);
void scroll_callback(GLFWwindow* window, double xoffset, double yoffset);
void findDraggedVertex(Vertex_iterator& dragged_v);
void mouse_drag_handle(GLFWwindow* window, const glm::mat4& view, const glm::mat4& proj, int width, int height);


//rendering main loop
int framebufferWidth = 1280;
int framebufferHeight = 720;
void updateCameraMovement();
void prepareFramebuffer(glm::mat4& view, glm::mat4& projection);
void renderMainMenu();
void renderSTLViewerWindow();
void renderCameraControlsWindow();
void renderSelectionControls();
void renderMainLoop();

//mainwindow
int mainwindow();

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

int mainwindow() {
    if (!glfwInit()) return -1;
    window = glfwCreateWindow(1280, 960, "STL Viewer", NULL, NULL);
    if (!window) return -1;
    glfwMakeContextCurrent(window);
    gladLoadGLLoader((GLADloadproc)glfwGetProcAddress);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_COLOR_MATERIAL);
    glEnable(GL_LIGHTING);
    glEnable(GL_LIGHT0);
    GLfloat light_pos[] = { 0.0f, 5.0f, 5.0f, 1.0f };
    glLightfv(GL_LIGHT0, GL_POSITION, light_pos);

    //mouse movement 
    glfwSetMouseButtonCallback(window, mouse_button_callback);
    glfwSetCursorPosCallback(window, cursor_position_callback);
    glfwSetScrollCallback(window, scroll_callback);
    glfwSetKeyCallback(window, key_callback);

    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL3_Init("#version 130");

    renderMode = SHADED;

    renderMainLoop();

    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();
    glfwDestroyWindow(window);
    glfwTerminate();

    return 0;
}
//////////////////////////////////////////////////////////////////////////////////Model rendering/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void differentViewsModes(RenderMode mode) {
    glPushMatrix();
    if (controlMode == MODEL_VIEW) {
        glScalef(modelScale, modelScale, modelScale);
        glTranslatef(modelOffset.x, modelOffset.y, modelOffset.z);
        glRotatef(modelRotationAngle, modelRotationAxis.x, modelRotationAxis.y, modelRotationAxis.z);
    }

    if (mode == SHADED || mode == BOTH) {
        glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
        glEnable(GL_LIGHTING);
        glColor3f(0.8f, 0.8f, 0.8f);

        glBegin(GL_TRIANGLES);
        for (auto f = mesh.facets_begin(); f != mesh.facets_end(); ++f) {
            auto h = f->facet_begin();
            CGAL_assertion(CGAL::circulator_size(h) == 3); // STL is triangulated

            do {
                const auto& p = h->vertex()->point();
                glVertex3f(p.x(), p.y(), p.z());
            } while (++h != f->facet_begin());
        }
        glEnd();
    }

    if (mode == WIREFRAME || mode == BOTH) {
        glDisable(GL_LIGHTING);
        glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
        glColor3f(0.1f, 1.0f, 0.1f);

        glBegin(GL_TRIANGLES);
        for (auto f = mesh.facets_begin(); f != mesh.facets_end(); ++f) {
            auto h = f->facet_begin();
            do {
                const auto& p = h->vertex()->point();
                glVertex3f(p.x(), p.y(), p.z());
            } while (++h != f->facet_begin());
        }
        glEnd();

        glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
    }

    if (mode == VERTICES) {
        glDisable(GL_LIGHTING);
        glColor3f(1.0f, 1.0f, 1.0f);
        glPointSize(5.0f);

        glBegin(GL_POINTS);
        for (auto v = mesh.vertices_begin(); v != mesh.vertices_end(); ++v) {
            const auto& p = v->point();
            glVertex3f(p.x(), p.y(), p.z());
        }
        glEnd();
    }
    glPopMatrix();
}
//void differentViewsModes(RenderMode mode) {
//    glPushMatrix();
//
//    if (controlMode == MODEL_VIEW) {
//        glScalef(modelScale, modelScale, modelScale);
//        glTranslatef(modelOffset.x, modelOffset.y, modelOffset.z);
//        glRotatef(modelRotationAngle, modelRotationAxis.x, modelRotationAxis.y, modelRotationAxis.z);
//    }
//
//    bool drawShaded = (mode == SHADED || mode == BOTH);
//    bool drawWire = (mode == WIREFRAME || mode == BOTH);
//    bool drawVertices = (mode == VERTICES);
//
//    if (drawShaded || drawWire) {
//        if (drawShaded) {
//            glEnable(GL_LIGHTING);
//            glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
//            glColor3f(0.8f, 0.8f, 0.8f);
//        }
//        else {
//            glDisable(GL_LIGHTING);
//            glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
//            glColor3f(0.1f, 1.0f, 0.1f);
//        }
//
//        glBegin(GL_TRIANGLES);
//        for (auto f = mesh.facets_begin(); f != mesh.facets_end(); ++f) {
//            auto h = f->facet_begin();
//            CGAL_assertion(CGAL::circulator_size(h) == 3);
//            do {
//                const auto& p = h->vertex()->point();
//                glVertex3f(p.x(), p.y(), p.z());
//            } while (++h != f->facet_begin());
//        }
//        glEnd();
//
//        // Reset fill mode if wireframe was drawn
//        if (drawWire) {
//            glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
//        }
//    }
//
//    if (drawVertices) {
//        glDisable(GL_LIGHTING);
//        glColor3f(1.0f, 1.0f, 1.0f);
//        glPointSize(5.0f);
//
//        glBegin(GL_POINTS);
//        for (auto v = mesh.vertices_begin(); v != mesh.vertices_end(); ++v) {
//            const auto& p = v->point();
//            glVertex3f(p.x(), p.y(), p.z());
//        }
//        glEnd();
//    }
//
//    glPopMatrix();
//}

//void drawSelectedRegion() {
//    glPushMatrix();
//    if ((show_handles || show_fixed) && (!handle_indices.empty() || !fixed_indices.empty())) {
//        glDisable(GL_LIGHTING);
//
//        glPointSize(5.0f);
//
//        glBegin(GL_POINTS);
//
//        for (auto v = mesh.vertices_begin(); v != mesh.vertices_end(); ++v) {
//            const auto& p = v->point();
//
//            if (show_handles && !handle_indices.empty() && handle_indices.count(v->id())) {
//
//                glColor3f(1.0f, 1.0f, 0.0f);
//                glVertex3f(p.x(), p.y(), p.z());
//
//            }
//
//            if (show_fixed && !fixed_indices.empty() && fixed_indices.count(v->id())) {
//
//                glColor3f(0.0f, 1.0f, 1.0f);
//                glVertex3f(p.x(), p.y(), p.z());
//
//            }
//        }
//        glEnd();
//    }
//    glPopMatrix();
//}
void drawSelectedRegion() {
    glPushMatrix();

    glDisable(GL_LIGHTING);
    glPointSize(5.0f);

    // Draw handle and fixed points
    glBegin(GL_POINTS);

    glm::vec3 minPt(FLT_MAX), maxPt(-FLT_MAX);

    for (auto v = mesh.vertices_begin(); v != mesh.vertices_end(); ++v) {
        const auto& pt = v->point();

        if ((show_handles || show_fixed) && (!handle_indices.empty() || !fixed_indices.empty())) {
            if (show_handles && handle_indices.count(v->id())) {
                glColor3f(1.0f, 1.0f, 0.0f);  // Yellow for handles
                glVertex3f(pt.x(), pt.y(), pt.z());
            }

            if (show_fixed && fixed_indices.count(v->id())) {
                glColor3f(0.0f, 1.0f, 1.0f);  // Cyan for fixed
                glVertex3f(pt.x(), pt.y(), pt.z());
            }
        }

        // Update bounding box only for handle points
        if (handle_indices.count(v->id())) {
            glm::vec3 p(pt.x(), pt.y(), pt.z());
            minPt = glm::min(minPt, p);
            maxPt = glm::max(maxPt, p);
        }
    }

    glEnd();  // End GL_POINTS

    // Draw bounding box if conditions are met
    if (mouseControlledDeform && boundingBoxVisible && minPt != glm::vec3(FLT_MAX)) {
        boundingBoxVisible = true;
        bboxCenter = 0.5f * (minPt + maxPt);

        glLineWidth(2.0f);
        glColor3f(1.0f, 0.0f, 0.0f);  // Red bounding box
        glBegin(GL_LINES);

        glm::vec3 min = minPt;
        glm::vec3 max = maxPt;

        // Define the 8 corners of the box
        glm::vec3 v0(min.x, min.y, min.z);
        glm::vec3 v1(max.x, min.y, min.z);
        glm::vec3 v2(max.x, max.y, min.z);
        glm::vec3 v3(min.x, max.y, min.z);
        glm::vec3 v4(min.x, min.y, max.z);
        glm::vec3 v5(max.x, min.y, max.z);
        glm::vec3 v6(max.x, max.y, max.z);
        glm::vec3 v7(min.x, max.y, max.z);

        // Bottom face
        glVertex3fv(glm::value_ptr(v0)); glVertex3fv(glm::value_ptr(v1));
        glVertex3fv(glm::value_ptr(v1)); glVertex3fv(glm::value_ptr(v2));
        glVertex3fv(glm::value_ptr(v2)); glVertex3fv(glm::value_ptr(v3));
        glVertex3fv(glm::value_ptr(v3)); glVertex3fv(glm::value_ptr(v0));

        // Top face
        glVertex3fv(glm::value_ptr(v4)); glVertex3fv(glm::value_ptr(v5));
        glVertex3fv(glm::value_ptr(v5)); glVertex3fv(glm::value_ptr(v6));
        glVertex3fv(glm::value_ptr(v6)); glVertex3fv(glm::value_ptr(v7));
        glVertex3fv(glm::value_ptr(v7)); glVertex3fv(glm::value_ptr(v4));

        // Vertical edges
        glVertex3fv(glm::value_ptr(v0)); glVertex3fv(glm::value_ptr(v4));
        glVertex3fv(glm::value_ptr(v1)); glVertex3fv(glm::value_ptr(v5));
        glVertex3fv(glm::value_ptr(v2)); glVertex3fv(glm::value_ptr(v6));
        glVertex3fv(glm::value_ptr(v3)); glVertex3fv(glm::value_ptr(v7));

        glEnd();
    }

    glEnable(GL_LIGHTING);
    glPopMatrix();
}
void drawSelectionPloygon() {
    glPushMatrix();

    // Draw selection polygon in 2D overlay
    if (!selectionPolygon.empty()) {
        glDisable(GL_LIGHTING);
        int width, height;
        glfwGetFramebufferSize(glfwGetCurrentContext(), &width, &height);

        glMatrixMode(GL_PROJECTION);
        glPushMatrix();
        glLoadIdentity();
        glOrtho(0, width, height, 0, -1, 1);

        glMatrixMode(GL_MODELVIEW);
        glPushMatrix();
        glLoadIdentity();

        glColor3f(1.0f, 1.0f, 0.0f);
        glBegin(GL_LINE_LOOP);
        for (const auto& pt : selectionPolygon) {
            glVertex2f(pt.x, pt.y);
        }
        glEnd();

        glPopMatrix();
        glMatrixMode(GL_PROJECTION);
        glPopMatrix();
        glMatrixMode(GL_MODELVIEW);
    }
    glPopMatrix();
}
void drawMouseControlledDeform() {
    glPushMatrix();
    if (mouseControlledDeform) {
        glm::vec3 minPt(FLT_MAX), maxPt(-FLT_MAX);

        for (auto v = mesh.vertices_begin(); v != mesh.vertices_end(); ++v) {
            if (handle_indices.count(v->id())) {
                const auto& pt = v->point();
                glm::vec3 p(pt.x(), pt.y(), pt.z());
                minPt = glm::min(minPt, p);
                maxPt = glm::max(maxPt, p);
            }
        }

        bboxCenter = 0.5f * (minPt + maxPt);
        boundingBoxVisible = true;
        glm::vec3 bboxMin = minPt;
        glm::vec3 bboxMax = maxPt;

        if (boundingBoxVisible) {
            glDisable(GL_LIGHTING);
            glLineWidth(2.0f);
            glColor3f(1.0f, 0.0f, 0.0f);  // Red bounding box

            glBegin(GL_LINES);

            glm::vec3 min = bboxMin;
            glm::vec3 max = bboxMax;

            // 8 corners
            glm::vec3 v0(min.x, min.y, min.z);
            glm::vec3 v1(max.x, min.y, min.z);
            glm::vec3 v2(max.x, max.y, min.z);
            glm::vec3 v3(min.x, max.y, min.z);
            glm::vec3 v4(min.x, min.y, max.z);
            glm::vec3 v5(max.x, min.y, max.z);
            glm::vec3 v6(max.x, max.y, max.z);
            glm::vec3 v7(min.x, max.y, max.z);

            // Bottom square
            glVertex3fv(glm::value_ptr(v0)); glVertex3fv(glm::value_ptr(v1));
            glVertex3fv(glm::value_ptr(v1)); glVertex3fv(glm::value_ptr(v2));
            glVertex3fv(glm::value_ptr(v2)); glVertex3fv(glm::value_ptr(v3));
            glVertex3fv(glm::value_ptr(v3)); glVertex3fv(glm::value_ptr(v0));

            // Top square
            glVertex3fv(glm::value_ptr(v4)); glVertex3fv(glm::value_ptr(v5));
            glVertex3fv(glm::value_ptr(v5)); glVertex3fv(glm::value_ptr(v6));
            glVertex3fv(glm::value_ptr(v6)); glVertex3fv(glm::value_ptr(v7));
            glVertex3fv(glm::value_ptr(v7)); glVertex3fv(glm::value_ptr(v4));

            // Vertical lines
            glVertex3fv(glm::value_ptr(v0)); glVertex3fv(glm::value_ptr(v4));
            glVertex3fv(glm::value_ptr(v1)); glVertex3fv(glm::value_ptr(v5));
            glVertex3fv(glm::value_ptr(v2)); glVertex3fv(glm::value_ptr(v6));
            glVertex3fv(glm::value_ptr(v3)); glVertex3fv(glm::value_ptr(v7));

            glEnd();
            glEnable(GL_LIGHTING);
        }

    }
    glPopMatrix();
}
void drawModel(RenderMode mode)  {
    differentViewsModes(mode);
    drawSelectionPloygon();
    drawSelectedRegion();
   // drawMouseControlledDeform();
}
//////////////////////////////////////////////////////////////////////////////////Model rendering/////////////////////////////////////////////////////////////////////////////////////////////////////////////////


//////////////////////////////////////////////////////////////////////////////////Mesh smooth/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool is_border_vertex(Polyhedron::Vertex& v) {
    HV_circulator h = v.vertex_begin();
    HV_circulator end = h;

    if (h == NULL) return false; // isolated vertex

    do {
        if (h->is_border()) {
            return true;
        }
        ++h;
    } while (h != end);

    return false;
}
void laplacian_smooth(Polyhedron& mesh, const std::set<int>& fixed_vertex_ids, int iterations = 10) {// Laplacian smoothing with fixed vertex 
    for (int iter = 0; iter < iterations; ++iter) {
        std::map<Vertex_iterator, Point> new_positions;

        for (Vertex_iterator v = mesh.vertices_begin(); v != mesh.vertices_end(); ++v) {
            int vid = v->id();

            // skip border or fixed vertices
            //if (fixed_vertex_ids.count(vid) || is_border_vertex(*v))
            //    continue;
            if (is_border_vertex(*v))
                continue;

            HV_circulator h = v->vertex_begin();
            HV_circulator end = h;
            int count = 0;
            Kernel::Vector_3 avg(0, 0, 0);

            CGAL_For_all(h, end) {
                Point neighbor = h->opposite()->vertex()->point();
                avg = avg + (neighbor - CGAL::ORIGIN);
                ++count;
            }

            if (count > 0) {
                Point smoothed = CGAL::ORIGIN + (avg / count);
                new_positions[v] = smoothed;
            }
        }

        // apply updated positions
        for (const auto& pair : new_positions) {
            pair.first->point() = pair.second;
        }
    }
}
void smooth_after_bend(Polyhedron& mesh, const std::set<int>& fixed_vertex_ids, int subdivision_level = 1, int smoothing_iters = 5) {
    //  subdivision to increase resolution
    std::cout << "before Subdivision method3 Loop_subdivision has : " << mesh.size_of_vertices() << " vertices, " << mesh.size_of_facets() << " triangles\n";
   CGAL::Subdivision_method_3::Loop_subdivision(mesh, subdivision_level);

    std::cout << "after Subdivision method3 Loop_subdivision has : " << mesh.size_of_vertices() << " vertices, " << mesh.size_of_facets() << " triangles\n";
    // CGAL::Subdivision_method_3::CatmullClark_subdivision(mesh, CGAL::parameters::number_of_iterations(3));
    mesh.normalize_border();
    
    //  Laplacian smoothing
    laplacian_smooth(mesh, fixed_vertex_ids, smoothing_iters);
}
//////////////////////////////////////////////////////////////////////////////////Mesh smooth/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
