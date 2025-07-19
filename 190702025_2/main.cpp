#include "Header.h"
int main() {mainwindow();return 0;}


void updateCameraMovement() {
    const float panSpeed = 0.05f;
    if (glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS) cameraTarget.z -= panSpeed;
    if (glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS) cameraTarget.z += panSpeed;
    if (glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS) cameraTarget.x -= panSpeed;
    if (glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS) cameraTarget.x += panSpeed;
}
void prepareFramebuffer(glm::mat4& view, glm::mat4& projection) {
    glClearColor(0.1f, 0.3f, 0.2f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    glfwGetFramebufferSize(window, &framebufferWidth, &framebufferHeight);
    framebufferHeight = (framebufferHeight == 0) ? 1 : framebufferHeight;
    glViewport(0, 0, framebufferWidth, framebufferHeight);

    float aspect = static_cast<float>(framebufferWidth) / framebufferHeight;
    projection = glm::perspective(glm::radians(45.0f), aspect, 0.1f, 100.0f);

    float yawRad = glm::radians(cameraYaw), pitchRad = glm::radians(cameraPitch);
    glm::vec3 direction = {
        cos(yawRad) * cos(pitchRad),
        sin(pitchRad),
        sin(yawRad) * cos(pitchRad)
    };
    glm::vec3 cameraPos = cameraTarget - direction * cameraDistance;
    view = glm::lookAt(cameraPos, cameraTarget, glm::vec3(0, 1, 0));

    glMatrixMode(GL_PROJECTION);
    glLoadMatrixf(glm::value_ptr(projection));
    glMatrixMode(GL_MODELVIEW);
    glLoadMatrixf(glm::value_ptr(view));
}
void renderMainMenu() {
    if (ImGui::BeginMainMenuBar()) {
        if (ImGui::BeginMenu("File")) {
            if (ImGui::MenuItem("Load STL")) {
                loadSTLMesh("C:\\Users\\SW6\\Downloads\\prong.stl");
                deformed_model.reset();
                handle_indices.clear();
                fixed_indices.clear();
                loaded = true;
            }
            if (ImGui::MenuItem("Exit")) {
                glfwSetWindowShouldClose(window, true);
            }
            ImGui::EndMenu();
        }

        if (ImGui::BeginMenu("Camera")) {
            if (ImGui::MenuItem("Camera parameters")) {
                showCameraWindow = true;
            }
            ImGui::EndMenu();
        }

        ImGui::EndMainMenuBar();
    }

    // Camera Controls window
    if (showCameraWindow) {
        ImGui::Begin("Camera Controls", &showCameraWindow);  // Pass pointer to allow window closing
        ImGui::Text("Projection:");
        if (ImGui::RadioButton("Perspective", cameraMode == PERSPECTIVE)) cameraMode = PERSPECTIVE;
        if (ImGui::RadioButton("Orthographic", cameraMode == ORTHOGRAPHIC)) cameraMode = ORTHOGRAPHIC;

        ImGui::Separator();
        ImGui::Text("Control:");
        if (ImGui::RadioButton("Orbit", controlMode == ORBIT)) controlMode = ORBIT;
        if (ImGui::RadioButton("Free", controlMode == FREE)) controlMode = FREE;
        if (ImGui::RadioButton("Move Model", controlMode == MODEL_VIEW)) controlMode = MODEL_VIEW;
        ImGui::End();
    }
}
void renderSTLViewerWindow() {
    ImGui::Begin("STL Viewer");
    ImGui::Text("%s", loaded ? "Model loaded" : "No model loaded");

    if (loaded) {
        ImGui::Text("Vertices: %zu", mesh.size_of_vertices());
        ImGui::Combo("Render Mode", (int*)&renderMode, "Shaded\0Wireframe\0Both\0Vertices\0");
    }

    renderSelectionControls();

    ImGui::Separator();
    if (ImGui::Checkbox("Mouse controlled deform", &mouseControlledDeform)) {
       
        if (!mouseControlledDeform) boundingBoxVisible = false;
    }
    if (mouseControlledDeform) {
        ImGui::Indent();

        if (ImGui::Checkbox("Rotation", &boundingBoxVisible)) {

        }
        ImGui::Unindent();
    }

    if (ImGui::Checkbox("Smooth area", &smoothSelectedMeshArea)) {

       // if (!mouseControlledDeform) boundingBoxVisible = false;
        selecting_smooth = true;
        drawing_region = true;
        region_polygon.clear();
        region_selection_done = false;
    }
    if (smoothSelectedMeshArea) {
        ImGui::Indent();

        if (ImGui::Button("Smooth")) {
           // laplacian_smooth(mesh, 20);

            
          //  Subdivision_method_3::CatmullClark_subdivision(mesh, CGAL::parameters::number_of_iterations(3));
          //  laplacian_smooth(mesh,fixed_indices, 10);
            smooth_after_bend(mesh, fixed_indices);


        }
        ImGui::Unindent();
    }

    ImGui::Checkbox("Show Handle Region", &show_handles);
    ImGui::Checkbox("Show Fixed Region", &show_fixed);
    ImGui::Checkbox("Show Deform Region", &show_region);

    ImGui::Text("Handles: %d", (int)handle_indices.size());
    ImGui::Text("Fixed: %d", (int)fixed_indices.size());
    ImGui::Text("Camera Target: (%.2f, %.2f, %.2f)", cameraTarget.x, cameraTarget.y, cameraTarget.z);
    ImGui::End();
}
void renderCameraControlsWindow() {
    ImGui::Begin("Camera Controls");
    ImGui::Text("Projection:");
    if (ImGui::RadioButton("Perspective", cameraMode == PERSPECTIVE)) cameraMode = PERSPECTIVE;
    if (ImGui::RadioButton("Orthographic", cameraMode == ORTHOGRAPHIC)) cameraMode = ORTHOGRAPHIC;

    ImGui::Separator();
    ImGui::Text("Control:");
    if (ImGui::RadioButton("Orbit", controlMode == ORBIT)) controlMode = ORBIT;
    if (ImGui::RadioButton("Free", controlMode == FREE)) controlMode = FREE;
    if (ImGui::RadioButton("Move Model", controlMode == MODEL_VIEW)) controlMode = MODEL_VIEW;
    ImGui::End();
}
void renderSelectionControls() {
    if (selection_done) {
        ImGui::Text("Selection complete.");
        if (ImGui::Button("Clear Selection")) {
            handle_indices.clear();
            fixed_indices.clear();
            selection_done = false;
        }
        return;
    }

    ImGui::Text("Selection Mode:");

    if (ImGui::Checkbox("Select Handle", &selecting_handle)) {
        selecting_fixed = false;
        if (!selecting_handle) handle_select_vertices = handle_select_region = false;
    }
    if (selecting_handle) {
        ImGui::Indent();
        ImGui::Checkbox("Select Vertices (Handle)", &handle_select_vertices);
        if (ImGui::Checkbox("Select Region (Handle)", &handle_select_region)) {
            selecting_handle_region = handle_select_region;
            if (handle_select_region) {
                drawing_region = true;
                region_polygon.clear();
                region_selection_done = false;
            }
        }
        ImGui::Unindent();
    }

    if (ImGui::Checkbox("Select Fixed", &selecting_fixed)) {
        selecting_handle = false;
        if (!selecting_fixed) fixed_select_vertices = fixed_select_region = false;
    }
    if (selecting_fixed) {
        ImGui::Indent();
        ImGui::Checkbox("Select Vertices (Fixed)", &fixed_select_vertices);
        if (ImGui::Checkbox("Select Region (Fixed)", &fixed_select_region)) {
            selecting_fixed_region = fixed_select_region;
            if (fixed_select_region) {
                drawing_region = true;
                region_polygon.clear();
                region_selection_done = false;
            }
        }
        ImGui::Unindent();
    }

    if (ImGui::Checkbox("Selection Done", &selection_done)) {
        if (selection_done) {
            selecting_handle = selecting_fixed = false;
            handle_select_vertices = handle_select_region = false;
            fixed_select_vertices = fixed_select_region = false;
        }
    }
}
void renderMainLoop() {
    glm::mat4 view, projection;

    while (!glfwWindowShouldClose(window)) {
        glfwPollEvents();

        // --- Camera Movement ---
        updateCameraMovement();

        // --- ImGui Frame Setup ---
        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        renderMainMenu();
        renderSTLViewerWindow();
      //  renderCameraControlsWindow();

        ImGui::Render();

        // --- Clear and Prepare Framebuffer ---
        prepareFramebuffer(view, projection);

        // --- Mesh Interaction & Rendering ---
        if (mouseControlledDeform && !ImGui::GetIO().WantCaptureMouse) {
            mouse_drag_handle(window, view, projection, framebufferWidth, framebufferHeight);
        }

        if (loaded) {
            drawModel(renderMode);
        }

        // --- ImGui Render Call ---
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
        glfwSwapBuffers(window);
    }
}

glm::mat4 getProjectionMatrix(GLFWwindow* window) {
    int width, height;
    glfwGetFramebufferSize(window, &width, &height);
    if (height == 0) height = 1;
    return glm::perspective(glm::radians(45.0f), (float)width / height, 0.1f, 100.0f);
}
glm::mat4 getViewMatrix() {
    float yawRad = glm::radians(cameraYaw);
    float pitchRad = glm::radians(cameraPitch);
    glm::vec3 direction = {
        cos(yawRad) * cos(pitchRad),
        sin(pitchRad),
        sin(yawRad) * cos(pitchRad)
    };
    glm::vec3 cameraPos = cameraTarget - direction * cameraDistance;
    return glm::lookAt(cameraPos, cameraTarget, glm::vec3(0, 1, 0));
}
glm::vec2 worldToScreen(const glm::vec3& worldPos, const glm::mat4& projection, const glm::mat4& view, int width, int height) {
    glm::vec4 clipSpace = projection * view * glm::vec4(worldPos, 1.0f);
    if (clipSpace.w == 0.0f) return glm::vec2(-1.0f); // invalid
    glm::vec3 ndc = glm::vec3(clipSpace) / clipSpace.w;
    return {
        (ndc.x * 0.5f + 0.5f) * width,
        (1.0f - (ndc.y * 0.5f + 0.5f)) * height
    };
}

void key_callback(GLFWwindow* window, int key, int scancode, int action, int mods) {
    if (key == GLFW_KEY_ENTER && action == GLFW_PRESS) {
        if (!selectionPolygon.empty()) {
            auto projection = getProjectionMatrix(window);
            auto view = getViewMatrix();

            int width, height;
            glfwGetFramebufferSize(window, &width, &height);

            for (auto v = mesh.vertices_begin(); v != mesh.vertices_end(); ++v) {
                glm::vec3 worldPos = { v->point().x(), v->point().y(), v->point().z() };
                glm::vec2 screenPos = worldToScreen(worldPos, projection, view, width, height);
                if (screenPos.x < 0.0f) continue;

                if (pointInPolygon(screenPos, selectionPolygon)) {
                    int id = v->id();
                    if (selecting_handle && selecting_handle_region)
                        handle_indices.insert(id);
                    else if (selecting_fixed && selecting_fixed_region)
                        fixed_indices.insert(id);
                    else if (smoothSelectedMeshArea)
                        smooth_indices.insert(id);
                }
            }

            selectionPolygon.clear();
            drawing_region = false;
            region_selection_done = true;
            selecting_handle_region = false;
            selecting_fixed_region = false;
            handle_select_region = false;
            fixed_select_region = false;
            selecting_smooth = false;
        }

    }

    if (key == GLFW_KEY_ENTER && action == GLFW_PRESS) {
        if (boundingBoxVisible) {
            boundingBoxVisible = false;
        }
        else {
            mouseControlledDeform = false;
        }
    }

}
void mouse_button_callback(GLFWwindow* window, int button, int action, int mods) {
    double xpos, ypos;
    glfwGetCursorPos(window, &xpos, &ypos);

    int width, height;
    glfwGetFramebufferSize(window, &width, &height);

    if (button == GLFW_MOUSE_BUTTON_RIGHT) {
        if (action == GLFW_PRESS && mouseControlledDeform && !dragging_handle && !handle_indices.empty()) {
            rotation_enabled = true;
            glm::vec3 minPt(FLT_MAX), maxPt(-FLT_MAX);
            for (auto v = mesh.vertices_begin(); v != mesh.vertices_end(); ++v) {
                if (handle_indices.count(v->id())) {
                    glm::vec3 p = { v->point().x(), v->point().y(), v->point().z() };
                    minPt = glm::min(minPt, p);
                    maxPt = glm::max(maxPt, p);
                }
            }
            bboxCenter = 0.5f * (minPt + maxPt);
            boundingBoxVisible = true;
        }
        if (action == GLFW_RELEASE) {
            rotation_enabled = false;
        }
    }

    if (button == GLFW_MOUSE_BUTTON_LEFT) {
        if (action == GLFW_PRESS) {
            if ((handle_select_region || fixed_select_region || smoothSelectedMeshArea) && drawing_region && !region_selection_done) {
                selectionPolygon.emplace_back((float)xpos, (float)ypos);
                return;
            }

            auto projection = getProjectionMatrix(window);
            auto view = getViewMatrix();

            float minDist = 1e10f;
            int closestId = -1;
            Vertex_iterator closestVertex;

            for (auto v = mesh.vertices_begin(); v != mesh.vertices_end(); ++v) {
                glm::vec3 worldPos = { v->point().x(), v->point().y(), v->point().z() };
                glm::vec2 screenPos = worldToScreen(worldPos, projection, view, width, height);
                if (screenPos.x < 0.0f) continue;

                float dx = xpos - screenPos.x;
                float dy = ypos - screenPos.y;
                float dist2 = dx * dx + dy * dy;

                if (dist2 < minDist) {
                    minDist = dist2;
                    closestId = v->id();
                    closestVertex = v;
                }
            }

            if (closestId != -1 && closestVertex != Vertex_iterator()) {
                if (selecting_handle && selecting_handle_region) handle_indices.insert(closestId);//smooth_indices
                if (selecting_fixed && selecting_fixed_region) fixed_indices.insert(closestId);
                if (selecting_smooth ) smooth_indices.insert(closestId);

                if (handle_indices.count(closestId)) {
                    dragging_handle = true;
                    dragged_handle = closestId;

                    const auto& p = closestVertex->point();
                    drag_start_pos = glm::vec3(p.x(), p.y(), p.z());
                }
            }

            rotating = !dragging_handle;
            lastX = (float)xpos;
            lastY = (float)ypos;
        }
        else if (action == GLFW_RELEASE) {
            if (dragging_handle) mouse_released = true;
            dragging_handle = false;
            dragged_handle = -1;
            rotating = false;
        }
    }
}
void cursor_position_callback(GLFWwindow* window, double xpos, double ypos) {
    if (dragging_handle) return; // Block camera movement during handle drag
    if (rotating) {
        cameraYaw += (float)(xpos - lastX) * 0.3f;
        cameraPitch += (float)(ypos - lastY) * 0.3f;
        cameraPitch = glm::clamp(cameraPitch, -89.0f, 89.0f);
        lastX = (float)xpos;
        lastY = (float)ypos;
    }
}
void scroll_callback(GLFWwindow* window, double xoffset, double yoffset) {
    // Get modifier keys
    int ctrlPressed = glfwGetKey(window, GLFW_KEY_LEFT_CONTROL) == GLFW_PRESS;
    int shiftPressed = glfwGetKey(window, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS;

    if (rotation_enabled && mouseControlledDeform && boundingBoxVisible) {

        static bool controlsPrinted = false;

        if (!controlsPrinted) {
            std::cout << "Rotation Controls:" << std::endl;
            std::cout << "Scroll:\t\t\tRotate around Y-axis" << std::endl;
            std::cout << "Ctrl + Scroll:\t\tRotate around X-axis" << std::endl;
            std::cout << "Shift + Scroll:\t\tRotate around Z-axis" << std::endl;
            std::cout << "Right-click + Scroll:\tZoom in/out (camera)" << std::endl;

            controlsPrinted = true; // prevent further prints
        }


        if (ctrlPressed) {
            // Rotate around X-axis
            rotationAngleX += static_cast<float>(yoffset) * 5.0f;
        }
        else if (shiftPressed) {
            // Rotate around Z-axis
            rotationAngleZ += static_cast<float>(yoffset) * 5.0f;
        }
        else {
            // Default: Rotate around Y-axis
            rotationAngleY += static_cast<float>(yoffset) * 5.0f;
        }
    }

    // Camera zoom (when rotation is not enabled)
    if (!rotation_enabled && (1 - GLFW_MOUSE_BUTTON_RIGHT == 0)) {
        cameraDistance -= yoffset * 0.5f;
        cameraDistance = std::max(0.5f, cameraDistance);
    }


}

void  compute_face_gradients(const Polyhedron& mesh, std::vector<Eigen::Matrix3d>& gradients) {
   

    for (Facet_const_iterator f = mesh.facets_begin(); f != mesh.facets_end(); ++f) {
        HFCC h = f->facet_begin();

        std::vector<Point> pts;

        do {
            pts.push_back(h->vertex()->point());
        } while (++h != f->facet_begin());

        if (pts.size() != 3) continue; // skip non-triangles

        Eigen::Vector3d p0(pts[0].x(), pts[0].y(), pts[0].z());
        Eigen::Vector3d p1(pts[1].x(), pts[1].y(), pts[1].z());
        Eigen::Vector3d p2(pts[2].x(), pts[2].y(), pts[2].z());

        // 2D parametrization of triangle
        Eigen::Matrix2d Dm;
        Dm.col(0) = (p1 - p0).head<2>();
        Dm.col(1) = (p2 - p0).head<2>();

        double area = 0.5 * ((p1 - p0).cross(p2 - p0)).norm();
        if (area < 1e-8) area = 1e-8;

        Eigen::Matrix2d Dm_inv = Dm.inverse();

        // 3D edge matrix
        Eigen::Matrix<double, 3, 2> Ds;
        Ds.col(0) = p1 - p0;
        Ds.col(1) = p2 - p0;

        Eigen::Matrix3d grad = Eigen::Matrix3d::Zero();
        grad.block<3, 2>(0, 0) = Ds * Dm_inv;

        gradients.push_back(grad);
    }

   
}
void apply_transformation_to_gradients(std::vector<Eigen::Matrix3d>& gradients, const Eigen::Matrix3d& M_T) {
    for (auto& JT : gradients) {
        JT = M_T * JT;
    }
}
void compute_divergence(const std::vector<Eigen::Matrix3d>& gradients, const Eigen::MatrixXi& F,size_t n_vertices,Eigen::VectorXd& div_x, Eigen::VectorXd& div_y, Eigen::VectorXd& div_z)
{
    div_x.setZero(n_vertices);
    div_y.setZero(n_vertices);
    div_z.setZero(n_vertices);

    for (int i = 0; i < F.rows(); ++i) {
        const Eigen::Matrix3d& grad = gradients[i];

        int i0 = F(i, 0);
        int i1 = F(i, 1);
        int i2 = F(i, 2);

        Eigen::Vector3d area_normal = (grad.col(0).cross(grad.col(1))) / 2.0;
        double area = area_normal.norm();
        if (area < 1e-8) area = 1e-8;

        // Distribute divergence equally to each vertex (simplified version)
        for (int j = 0; j < 3; ++j) {
            int v = F(i, j);
            if (v >= 0 && v < n_vertices) {
                div_x[v] += grad(0, j) * area / 3.0;
                div_y[v] += grad(1, j) * area / 3.0;
                div_z[v] += grad(2, j) * area / 3.0;
            }
        }
    }
}
void poisson_deformation(const Eigen::MatrixXd& V, const Eigen::MatrixXi& F, const Eigen::VectorXd& div_x, const Eigen::VectorXd& div_y, const Eigen::VectorXd& div_z, const std::set<int>& fixed,  Eigen::MatrixXd& V_new)
{
    using namespace Eigen;
    using namespace igl;

    const int n = V.rows();

    // Step 1: Build cotangent Laplacian
    SparseMatrix<double> L;
    cotmatrix(V, F, L); // cotangent Laplacian (symmetric negative-definite)

    // Step 2: Convert fixed set to vector
    VectorXi b(fixed.size());
    {
        int i = 0;
        for (int idx : fixed) b[i++] = idx;
    }

    // Step 3: Build known values at fixed points (set to original positions)
    MatrixXd bc(fixed.size(), 3);
    for (int i = 0; i < b.size(); ++i)
        bc.row(i) = V.row(b[i]);

    // Step 4: Solve three Poisson systems (X, Y, Z)
    MatrixXd result(n, 3);

    min_quad_with_fixed_data<double> data;
    Eigen::SparseMatrix<double> Q = -L;  // make positive definite
    VectorXd x_sol, y_sol, z_sol;

    std::cout << "[Debug] Q size: " << Q.rows() << " x " << Q.cols() << std::endl;
    std::cout << "[Debug] Q nonzeros: " << Q.nonZeros() << std::endl;
    if (b.size() == 0) {
        std::cerr << "[ERROR] No fixed vertex constraints — you must fix at least one vertex." << std::endl;
        return;
    }
    for (int i = 0; i < b.size(); ++i) {
        if (b[i] < 0 || b[i] >= V.rows()) {
            std::cerr << "[ERROR] Invalid fixed index: " << b[i] << std::endl;
            return;
        }
    }
    bool valid = true;
    for (int i = 0; i < b.size(); ++i) {
        if (b[i] < 0 || b[i] >= V.rows()) {
            std::cerr << "[ERROR] Invalid constraint index b[" << i << "] = " << b[i]
                << " (V.rows() = " << V.rows() << ")" << std::endl;
            valid = false;
        }
    }
    if (!valid) return; // Abort if any index is invalid

    if (!min_quad_with_fixed_precompute(Q, b, SparseMatrix<double>(), true, data)) {
        std::cerr << "Poisson precompute failed!" << std::endl;
        return;
    }

    if (!min_quad_with_fixed_solve(data, div_x, bc.col(0), VectorXd(), x_sol) ||
        !min_quad_with_fixed_solve(data, div_y, bc.col(1), VectorXd(), y_sol) ||
        !min_quad_with_fixed_solve(data, div_z, bc.col(2), VectorXd(), z_sol)) {
        std::cerr << "Poisson solve failed!" << std::endl;
        return;
    }

    // Step 5: Combine results
    result.col(0) = x_sol;
    result.col(1) = y_sol;
    result.col(2) = z_sol;

    V_new = result;  // output
}


void findDraggedVertex(Vertex_iterator& dragged_v) {

    for (auto v = mesh.vertices_begin(); v != mesh.vertices_end(); ++v) {
        if (v->id() == dragged_handle) {
            dragged_v = v;
            break;
        }
    }
    if (dragged_v == Vertex_iterator()) return;
}

void arapDeform(GLFWwindow* window, const glm::mat4& view, const glm::mat4& proj, int width, int height) {
    double xpos, ypos;
    glfwGetCursorPos(window, &xpos, &ypos);

    // Find dragged vertex
    Vertex_iterator dragged_v = Vertex_iterator();
    //for (auto v = mesh.vertices_begin(); v != mesh.vertices_end(); ++v) {
    //    if (v->id() == dragged_handle) {
    //        dragged_v = v;
    //        break;
    //    }
    //}
    //if (dragged_v == Vertex_iterator()) return;


    int target_id = dragged_handle;
    dragged_v = std::find_if(mesh.vertices_begin(), mesh.vertices_end(),
        [target_id](const Polyhedron::Vertex& v) {
            return v.id() == dragged_handle;
        });


    //findDraggedVertex(dragged_v);


    const auto& p = dragged_v->point();
    glm::vec4 worldPos(p.x(), p.y(), p.z(), 1.0f);
    glm::vec4 screenSpace = proj * view * worldPos;
    screenSpace /= screenSpace.w;

    float ndc_x = (float)xpos / width * 2.0f - 1.0f;
    float ndc_y = 1.0f - (float)ypos / height * 2.0f;
    glm::vec4 new_ndc(ndc_x, ndc_y, screenSpace.z, 1.0f);

    glm::mat4 invVP = glm::inverse(proj * view);
    glm::vec4 new_world = invVP * new_ndc;
    new_world /= new_world.w;



    glm::vec3 drag_from(p.x(), p.y(), p.z());
    glm::vec3 drag_to(new_world.x, new_world.y, new_world.z);
    glm::vec3 displacement = drag_to - drag_from;

    // Apply rotation
    float angleRad = glm::radians(rotationAngleY);
    glm::mat4 T = glm::translate(glm::mat4(1.0f), bboxCenter);
    glm::mat4 R = glm::rotate(glm::mat4(1.0f), angleRad, glm::vec3(0, 1, 0));
    glm::mat4 rotationMatrix = T * R * glm::inverse(T);

    // Precompute control/ROI insertion
   // deform->clear_roi_vertices();
   // deform->clear_control_vertices();
    for (auto v = mesh.vertices_begin(); v != mesh.vertices_end(); ++v) {
        int vid = v->id();
        if (fixed_indices.count(vid)) continue;
        deform->insert_roi_vertex(v);
        if (handle_indices.count(vid)) {
            deform->insert_control_vertex(v);
        }
    }

    // Set new target positions for control vertices
    for (auto v = mesh.vertices_begin(); v != mesh.vertices_end(); ++v) {
        if (!handle_indices.count(v->id())) continue;

        const auto& pt = v->point();
        glm::vec4 origPos(pt.x(), pt.y(), pt.z(), 1.0f);
        glm::vec4 rotatedPos = rotationMatrix * origPos;

        rotatedPos.x += displacement.x;
        rotatedPos.y += displacement.y;
        rotatedPos.z += displacement.z;

        deform->set_target_position(v, Kernel::Point_3(rotatedPos.x, rotatedPos.y, rotatedPos.z));
    }

    deform->deform();

    rotationAngleY = 0.0f;  // Reset rotation
}

void mouse_drag_handle(GLFWwindow* window, const glm::mat4& view, const glm::mat4& proj, int width, int height) {
    if (!dragging_handle || dragged_handle == -1) return;

    //if (mouseControlledDeform) {
    //    arapDeform(window, view, proj, width, height);
    //}

   /* if (mouseControlledDeform) {
        if (mouse_released && dragged_handle >= 0) {
            mouse_released = false; // reset flag
            std::vector<std::array<int, 3>> cleaned_faces;
            clean_degenerate_faces(triangleIndices, cleaned_faces);
            Eigen::VectorXi component_ids;




            // int num_components = igl::components(F, component_ids);
           //  std::cout << "[Info] Mesh has " << num_components << " connected components." << std::endl;

             // === Step 1: Build mesh and gradients
             //Eigen::MatrixXi F(triangleIndices.size(), 3);
             //for (size_t i = 0; i < triangleIndices.size(); ++i) {
             //    F(i, 0) = triangleIndices[i][0];
             //    F(i, 1) = triangleIndices[i][1];
             //    F(i, 2) = triangleIndices[i][2];
             //}

             // === Step 1: Build mesh and gradients

             //Eigen::MatrixXi F(cleaned_faces.size(), 3);
             //for (size_t i = 0; i < cleaned_faces.size(); ++i) {
             //    F(i, 0) = cleaned_faces[i][0];
             //    F(i, 1) = cleaned_faces[i][1];
             //    F(i, 2) = cleaned_faces[i][2];
             //}
            std::vector<std::array<int, 3>> face_indices;

            for (auto f = mesh.facets_begin(); f != mesh.facets_end(); ++f) {
                auto h = f->facet_begin();
                std::vector<int> ids;
                do {
                    ids.push_back(h->vertex()->id());
                } while (++h != f->facet_begin());

                if (ids.size() == 3) {
                    face_indices.push_back({ ids[0], ids[1], ids[2] });
                }
            }

            // Now convert to Eigen::MatrixXi
            Eigen::MatrixXi F(face_indices.size(), 3);
            for (size_t i = 0; i < face_indices.size(); ++i) {
                F(i, 0) = face_indices[i][0];
                F(i, 1) = face_indices[i][1];
                F(i, 2) = face_indices[i][2];
            }





            Eigen::MatrixXd V(mesh.size_of_vertices(), 3);
            int i = 0;
            for (auto f = mesh.facets_begin(); f != mesh.facets_end(); ++f) {
                auto h = f->facet_begin();
                do {
                    const auto& p = h->vertex()->point();
                    V.row(i) = Eigen::Vector3d(h->vertex()->point().x(), h->vertex()->point().y(), h->vertex()->point().z());
                    ++i;
                    if (i == mesh.size_of_vertices())break;
                } while (++h != f->facet_begin());
            }



            //for (int i = 0; i < model.size(); ++i) {
            //    V.row(i) = Eigen::Vector3d(model[i].x, model[i].y, model[i].z);
            //}


            //Eigen::VectorXi C;
            //igl::connected_components(V,F, C);

            //int num_components = *std::max_element(C.begin(), C.end()) + 1;

            //if (num_components == 1) {
            //    std::cout << "Mesh is fully connected." << std::endl;
            //}
            //else {
            //    std::cout << "Mesh is not fully connected. Number of components: " << num_components << std::endl;
            //}



            std::vector<Eigen::Matrix3d> gradients;
            compute_face_gradients(mesh, gradients);
            // compute_gradients( F, gradients);

             // === Step 2: Compute handle drag world position
            double xpos, ypos;
            glfwGetCursorPos(window, &xpos, &ypos);

            int width, height;
            glfwGetFramebufferSize(window, &width, &height);

            glm::vec4 worldPos(model[dragged_handle].x, model[dragged_handle].y, model[dragged_handle].z, 1.0f);
            glm::vec4 screenSpace = proj * view * worldPos;
            screenSpace /= screenSpace.w;

            float ndc_x = (float)xpos / width * 2.0f - 1.0f;
            float ndc_y = 1.0f - (float)ypos / height * 2.0f;

            glm::vec4 new_ndc(ndc_x, ndc_y, screenSpace.z, 1.0f);
            glm::mat4 invVP = glm::inverse(proj * view);
            glm::vec4 new_world = invVP * new_ndc;
            new_world /= new_world.w;

            glm::vec3 new_pos(new_world.x, new_world.y, new_world.z);
            glm::vec3 translation = new_pos - drag_start_pos;

            // === Step 3: Build transformation matrix
            Eigen::Matrix3d M_T = Eigen::Matrix3d::Identity();
            if (glm::length(translation) > 1e-5f) {
                glm::vec3 direction = glm::normalize(translation);
                float magnitude = glm::length(translation);

                Eigen::Matrix3d scale_dir = Eigen::Matrix3d::Identity();
                Eigen::Vector3d dir(direction.x, direction.y, direction.z);
                scale_dir += magnitude * dir * dir.transpose();

                M_T = scale_dir;
            }

            // === Step 4: Transform gradients
            apply_transformation_to_gradients(gradients, M_T);

            // === Step 5: Compute divergence and solve
            Eigen::VectorXd div_x, div_y, div_z;
            compute_divergence(gradients, F, mesh.size_of_vertices(), div_x, div_y, div_z);

            Eigen::MatrixXd V_new;
            //solve_poisson_system(model, F, div_x, div_y, div_z,  V_new);





            //Eigen::MatrixXd V(4, 3);
            //V << 0, 0, 0,
            //    1, 0, 0,
            //    1, 1, 0,
            //    0, 1, 0;

            //Eigen::MatrixXi F(2, 3);
            //F << 0, 1, 2,
            //    0, 2, 3;

            //Eigen::VectorXd div_x = Eigen::VectorXd::Zero(4);
            //Eigen::VectorXd div_y = Eigen::VectorXd::Zero(4);
            //Eigen::VectorXd div_z = Eigen::VectorXd::Zero(4);

            //std::set<int> fixed = { 0, 3 };

            //Eigen::MatrixXd V_new;




            poisson_deformation(V, F, div_x, div_y, div_z, fixed_indices, V_new);


            // === Step 6: Apply to deformed model






            // 
            //deformed_model = model;

            //

            //for (int i = 0; i < V_new.rows(); ++i) {
            //    if (fixed_indices.count(i)) continue;
            //    deformed_model[i].x = V_new(i, 0);
            //    deformed_model[i].y = V_new(i, 1);
            //    deformed_model[i].z = V_new(i, 2);
            //}


            for (auto v = mesh.vertices_begin(); v != mesh.vertices_end(); ++v) {
                int vid = v->id();
                if (fixed_indices.count(vid)) continue;  // Skip fixed vertices

                v->point() = Kernel::Point_3(
                    V_new(vid, 0),
                    V_new(vid, 1),
                    V_new(vid, 2)
                );
            }



            // Optional: update model directly if you want permanent change
            // model = deformed_model;
        }
    }*/
    

if (mouseControlledDeform) {
    if (mouse_released && dragged_handle >= 0) {
        mouse_released = false; // reset

        std::vector<std::array<int, 3>> face_indices;
        for (auto f = mesh.facets_begin(); f != mesh.facets_end(); ++f) {
            auto h = f->facet_begin(); std::vector<int> ids;
            do { ids.push_back(h->vertex()->id()); } while (++h != f->facet_begin());
            if (ids.size() == 3) face_indices.push_back({ ids[0], ids[1], ids[2] });
        }

        Eigen::MatrixXi F(face_indices.size(), 3);
        for (size_t i = 0; i < face_indices.size(); ++i) {
            F(i, 0) = face_indices[i][0];
            F(i, 1) = face_indices[i][1];
            F(i, 2) = face_indices[i][2];
        }

        Eigen::MatrixXd V(mesh.size_of_vertices(), 3);
        for (auto v = mesh.vertices_begin(); v != mesh.vertices_end(); ++v) {
            int id = v->id();
            const auto& p = v->point();
            V.row(id) = Eigen::Vector3d(p.x(), p.y(), p.z());
        }

        std::vector<Eigen::Matrix3d> gradients;


        compute_face_gradients(mesh, gradients);

        double xpos, ypos;
        glfwGetCursorPos(window, &xpos, &ypos);
        int width, height;
        glfwGetFramebufferSize(window, &width, &height);

       // glm::vec4 worldPos(model[dragged_handle].x, model[dragged_handle].y, model[dragged_handle].z, 1.0f);

        Vertex_iterator dragged_v;
        for (auto v = mesh.vertices_begin(); v != mesh.vertices_end(); ++v) {
            if (v->id() == dragged_handle) {
                dragged_v = v;
                break;
            }
        }

        const auto& pt = dragged_v->point();
        glm::vec4 worldPos(pt.x(), pt.y(), pt.z(), 1.0f);





        glm::vec4 screenSpace = proj * view * worldPos;
        screenSpace /= screenSpace.w;

        float ndc_x = (float)xpos / width * 2.0f - 1.0f;
        float ndc_y = 1.0f - (float)ypos / height * 2.0f;

        glm::vec4 new_ndc(ndc_x, ndc_y, screenSpace.z, 1.0f);
        glm::mat4 invVP = glm::inverse(proj * view);
        glm::vec4 new_world = invVP * new_ndc;
        new_world /= new_world.w;

        glm::vec3 new_pos(new_world.x, new_world.y, new_world.z);
        glm::vec3 translation = new_pos - drag_start_pos;

        Eigen::Matrix3d M_T = Eigen::Matrix3d::Identity();
        if (glm::length(translation) > 1e-5f) {
            glm::vec3 dir = glm::normalize(translation);
            float mag = glm::length(translation);
            Eigen::Vector3d e_dir(dir.x, dir.y, dir.z);
            M_T += mag * e_dir * e_dir.transpose();
        }

        apply_transformation_to_gradients(gradients, M_T);

        Eigen::VectorXd div_x, div_y, div_z;
        compute_divergence(gradients, F, mesh.size_of_vertices(), div_x, div_y, div_z);

        Eigen::MatrixXd V_new;
        poisson_deformation(V, F, div_x, div_y, div_z, fixed_indices, V_new);





        // === Step 6: Apply to deformed model





      //  demesh= mesh;
        for (auto v = mesh.vertices_begin(); v != mesh.vertices_end(); ++v) {
            int vid = v->id();
            if (vid >= V_new.rows()) continue;
            if (fixed_indices.count(vid)) continue;

            v->point() = Kernel::Point_3(
                V_new(vid, 0),
                V_new(vid, 1),
                V_new(vid, 2)
            );
        }

// 
//deformed_model = model;

//

//for (int i = 0; i < V_new.rows(); ++i) {
//    if (fixed_indices.count(i)) continue;
//    deformed_model[i].x = V_new(i, 0);
//    deformed_model[i].y = V_new(i, 1);
//    deformed_model[i].z = V_new(i, 2);
//}


        //for (auto v = mesh.vertices_begin(); v != mesh.vertices_end(); ++v) {
        //    int vid = v->id();
        //    if (fixed_indices.count(vid)) continue;  // Skip fixed vertices
        //
        //    v->point() = Kernel::Point_3(
        //        V_new(vid, 0),
        //        V_new(vid, 1),
        //        V_new(vid, 2)
        //    );
        //}



        // Optional: update model directly if you want permanent change
        // model = deformed_model;





        //glDisable(GL_LIGHTING);
        //glColor3f(1.0f, 0.0f, 0.0f);
        //glPointSize(5.0f);

        //glBegin(GL_POINTS);
        //for (auto v = mesh.vertices_begin(); v != mesh.vertices_end(); ++v) {
        //    int vid = v->id();
        //    if (fixed_indices.count(vid)) continue;
        //
        //    v->point() = Kernel::Point_3(V_new(vid, 0), V_new(vid, 1), V_new(vid, 2));
        //  
        //    glVertex3f(V_new(vid, 0), V_new(vid, 1), V_new(vid, 2));
        //    
        //    
        //}
       //
        //glEnd();
    }
}

}

