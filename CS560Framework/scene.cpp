#include "math.h"
#include <iostream>
#include <stdlib.h>

#include <glbinding/gl/gl.h>
#pragma warning(disable: 4251)
#include <glbinding/Binding.h>
#pragma warning(disable: 4251)
using namespace gl;

#pragma warning(disable: 4190)
#include <glu.h>

#define GLM_FORCE_RADIANS
#define GLM_SWIZZLE
#include <glm/glm.hpp>
#include <glm/ext.hpp>          

#include "framework.h"
#include "texture.h"
#include "transform.h"

#include "RigidBody.h"

const float PI = 3.14159f;
const float rad = PI/180.0f;    

glm::mat4 Identity;

#define CHECKERROR {GLenum err = glGetError(); if (err != GL_NO_ERROR) { fprintf(stderr, "OpenGL error (at line scene.cpp:%d): %s\n", __LINE__, gluErrorString(err)); exit(-1);} }


RigidBody Scene::gCube;

/**
 * @brief Retrieve the world position of a spring end.
 *
 * @param e SpringEnd object (RigidPoint or Anchor)
 * @param body Reference to the RigidBody
 * @return glm::vec3 World-space position of the spring end
 */
static glm::vec3 getEndWorldPos(const SpringEnd& e, const RigidBody& body)
{
    if (e.type == SpringEndType::RigidPoint)
        return body.pointWorld(e.index);
    return e.worldPos; // Anchor
}


/**
 * @brief Update the VBO storing spring line vertices for rendering.
 *
 * @param springVBO OpenGL buffer ID for spring vertices
 * @param springVertexCount Output number of vertices
 * @param springs Vector of springs to render
 * @param body RigidBody to which springs are attached
 */
void Scene::UpdateSpringBuffer(GLuint springVBO, GLsizei& springVertexCount,
    const std::vector<Spring>& springs, const RigidBody& body)
{
    std::vector<glm::vec3> verts;
    verts.reserve(springs.size() * 2);

    for (const Spring& s : springs)
    {
        glm::vec3 a = getEndWorldPos(s.a, body);
        glm::vec3 b = getEndWorldPos(s.b, body);
        verts.push_back(a);
        verts.push_back(b);
    }

    springVertexCount = (GLsizei)verts.size();

    glBindBuffer(GL_ARRAY_BUFFER, springVBO);
    glBufferData(GL_ARRAY_BUFFER, verts.size() * sizeof(glm::vec3), verts.data(), GL_DYNAMIC_DRAW);
}


/**
 * @brief Compute a linear index for a 4x4x4 3D lattice.
 *
 * This function maps 3D coordinates (i, j, k) in a 4x4x4 grid to a single
 * linear index suitable for accessing a flat array of 64 elements.
 *
 * @param i X-coordinate in the range [0, 3].
 * @param j Y-coordinate in the range [0, 3].
 * @param k Z-coordinate in the range [0, 3].
 * @return int Linear index corresponding to the 3D coordinates.
 */
static inline int idx4(int i, int j, int k) { return i + 4 * j + 16 * k; }

/**
 * @brief Add a spring connecting two rigid points of a RigidBody.
 *
 * This helper function creates a spring connecting two points on a rigid body,
 * sets its spring constant, damping coefficient, and rest length based on the
 * initial distance between the points, and appends it to the given spring vector.
 *
 * @param springs Reference to the vector of springs to which the new spring will be added.
 * @param body Reference to the RigidBody containing the points.
 * @param a Index of the first rigid point in the RigidBody.
 * @param b Index of the second rigid point in the RigidBody.
 * @param k Spring constant (stiffness) of the new spring.
 * @param c Damping coefficient of the new spring.
 */
static void addRigidRigidSpring(std::vector<Spring>& springs, RigidBody& body, int a, int b, float k, float c) {
    Spring s;
    s.a = { SpringEndType::RigidPoint, a };
    s.b = { SpringEndType::RigidPoint, b };
    s.k = k;
    s.c = c;
    s.L0 = glm::length(body.pointWorld(a) - body.pointWorld(b));
    springs.push_back(s);
}



/**
 * @brief Initialize camera parameters (position, orientation, frustum, etc.)
 */
void Scene::InitCamera()
{
    spin = 0.0;
    tilt = 15.0;
    eye = glm::vec3(0.0, -5.0, 5.0);
    speed = 300.0 / 30.0;
    last_time = (float)glfwGetTime();
    tr = glm::vec3(0.0, 0.0, 5.0);

    ry = 0.4f;
    front = 0.5;
    back = 5000.0;
}

/**
 * @brief Initialize the scene, including rigid body, springs, textures, shaders, and buffers.
 *
 * Builds a 4x4x4 cube lattice, sets up spring connections, world anchors, initial spin,
 * gBuffer and shaders, VAO/VBO for springs, and other scene-wide parameters.
 */
void Scene::InitializeScene()
{

    gCube.buildLattice4x4x4(1.0f);            // cube side length = 1 (adjust)
    gCube.computeMassAndInertiaFromPoints(1.0f); // total mass = 1 kg (adjust)

    gCube.x = glm::vec3(0.0f);
    gCube.R = glm::mat3(1.0f);
    gCube.P = glm::vec3(0.0f);
    gCube.L = glm::vec3(0.0f);

    springs.clear();

    // World anchors
    glm::vec3 anchor1(-0.5f, 1.5f, 0.0f);
    glm::vec3 anchor2(0.5f, 1.5f, 0.0f);

    int p1 = 60;  // top-layer cube points
    int p2 = 63;

    Spring s1;
    s1.a = { SpringEndType::RigidPoint, p1 };
    s1.b = { SpringEndType::Anchor, -1, anchor1 };
    s1.k = 50.0f;
    s1.c = 1.0f;
    s1.L0 = glm::length(gCube.pointWorld(p1) - anchor1);

    Spring s2 = s1;
    s2.a.index = p2;
    s2.b.worldPos = anchor2;

    springs.push_back(s1);
    springs.push_back(s2);

    // internal springs

    const float k_struct = 200.0f;
    const float c_struct = 2.0f;

    const float k_diag = 200.0f;
    const float c_diag = 2.0f;

    // Structural springs (x, y, z neighbors)
    for (int k = 0; k < 4; ++k) {
        for (int j = 0; j < 4; ++j) {
            for (int i = 0; i < 4; ++i)
            {
                const int p = idx4(i, j, k);

                if (i + 1 < 4) addRigidRigidSpring(springs, gCube, p, idx4(i + 1, j, k), k_struct, c_struct);
                if (j + 1 < 4) addRigidRigidSpring(springs, gCube, p, idx4(i, j + 1, k), k_struct, c_struct);
                if (k + 1 < 4) addRigidRigidSpring(springs, gCube, p, idx4(i, j, k + 1), k_struct, c_struct);
            }
        }
    }

    // Face diagonal springs: XY faces
    for (int k = 0; k < 4; ++k) {
        for (int j = 0; j < 3; ++j) {
            for (int i = 0; i < 3; ++i) {
                int p00 = idx4(i, j, k);
                int p11 = idx4(i + 1, j + 1, k);
                int p10 = idx4(i + 1, j, k);
                int p01 = idx4(i, j + 1, k);

                addRigidRigidSpring(springs, gCube, p00, p11, k_diag, c_diag);
                addRigidRigidSpring(springs, gCube, p10, p01, k_diag, c_diag);
            }
        }
    }

    // Face diagonal springs: XZ faces
    for (int j = 0; j < 4; ++j) {
        for (int k = 0; k < 3; ++k) {
            for (int i = 0; i < 3; ++i)
            {
                int p00 = idx4(i, j, k);
                int p11 = idx4(i + 1, j, k + 1);
                int p10 = idx4(i + 1, j, k);
                int p01 = idx4(i, j, k + 1);

                addRigidRigidSpring(springs, gCube, p00, p11, k_diag, c_diag);
                addRigidRigidSpring(springs, gCube, p10, p01, k_diag, c_diag);
            }
        }
    }

    // Face diagonal springs: YZ faces
    for (int i = 0; i < 4; ++i){
        for (int k = 0; k < 3; ++k){
            for (int j = 0; j < 3; ++j)
            {
                int p00 = idx4(i, j, k);
                int p11 = idx4(i, j + 1, k + 1);
                int p10 = idx4(i, j + 1, k);
                int p01 = idx4(i, j, k + 1);

                addRigidRigidSpring(springs, gCube, p00, p11, k_diag, c_diag);
                addRigidRigidSpring(springs, gCube, p10, p01, k_diag, c_diag);
            }
        }
    }

    /**/
    gCube.x = glm::vec3(0.0f, 0.0f, 0.0f);
    gCube.R = glm::mat3(1.0f);

    // Give it an initial spin so you can see it move (temporary sanity check)
    gCube.P = glm::vec3(0.0f);
    gCube.L = glm::vec3(0.0f, 0.5f, 0.0f); // angular momentum about Y



    boxPosition = glm::vec3(0.0f, 0.0f, 5.0f);
    boxMoveSpeed = 2.0f;

    moveLeft = false;
    moveRight = false;
    moveUp = false;
    moveDown = false;
    moveForward = false;
    moveBackward = false;

    std::cout << "INIT\n";
    glEnable(GL_DEPTH_TEST);
    CHECKERROR;

    glfwGetFramebufferSize(window, &width, &height);

    lightSpin = 90.0;
    lightTilt = -45.0;
    lightDist = 50.0;
    lightCol = { 3.0f, 3.0f, 3.0f };

    key = 0;
    InitCamera();
 
    glGenBuffers(1, &id);
    bindpoint = 1;
    glBindBufferBase(GL_UNIFORM_BUFFER, bindpoint, id);
    CHECKERROR;

    glEnable(GL_DEPTH_TEST);

    //FBO
    gBuffer.Init(width, height);
    CHECKERROR;

    CharTexture = new Texture("fbx/textures/Gray.jpg");

    lightingProgram = new ShaderProgram();
    lightingProgram->AddShader("shaders/lightingPhong.vert", GL_VERTEX_SHADER);
    lightingProgram->AddShader("shaders/lightingPhong.frag", GL_FRAGMENT_SHADER);

    glBindAttribLocation(lightingProgram->programId, 0, "vertex");
    glBindAttribLocation(lightingProgram->programId, 1, "vertexNormal");
    glBindAttribLocation(lightingProgram->programId, 2, "vertexTexture");
    glBindAttribLocation(lightingProgram->programId, 3, "vertexTangent");
    lightingProgram->LinkProgram();
    lightingProgram->UseShader();
    CharTexture->BindTexture(0, lightingProgram->programId, "texture1");//
    

    gBufferProgram = new ShaderProgram();
    gBufferProgram->AddShader("shaders/gBuffer.vert", GL_VERTEX_SHADER);
    gBufferProgram->AddShader("shaders/gBuffer.frag", GL_FRAGMENT_SHADER);

    glBindAttribLocation(gBufferProgram->programId, 0, "vertex");
    glBindAttribLocation(gBufferProgram->programId, 1, "vertexNormal");
    glBindAttribLocation(gBufferProgram->programId, 2, "vertexTexture");
    glBindAttribLocation(gBufferProgram->programId, 3, "vertexTangent");
    glBindAttribLocation(gBufferProgram->programId, 4, "boneIds");
    glBindAttribLocation(gBufferProgram->programId, 5, "weight");
    gBufferProgram->LinkProgram();
    
    quad = new Quad();

    //model = new Mesh();
    //model->LoadMesh(meshPat); //moved the loading animations into this
    //model->LoadAnimation();
	//model->PrintValues();//to get the bones structure

    boxMesh = new Mesh();
	boxMesh->SetRigibody(gCube);
	boxMesh->LoadMesh(boxPath);

    //
    // ---------------- Layer 4: debug spring rendering ----------------
    debugProgram = new ShaderProgram();
    debugProgram->AddShader("shaders/debugLines.vert", GL_VERTEX_SHADER);
    debugProgram->AddShader("shaders/debugLines.frag", GL_FRAGMENT_SHADER);

    // match your attribute binding style
    glBindAttribLocation(debugProgram->programId, 0, "vertex");
    debugProgram->LinkProgram();

    // VAO/VBO for dynamic line vertices
    glGenVertexArrays(1, &springVAO);
    glGenBuffers(1, &springVBO);

    glBindVertexArray(springVAO);
    glBindBuffer(GL_ARRAY_BUFFER, springVBO);

    // allocate some space; we'll overwrite every frame
    glBufferData(GL_ARRAY_BUFFER, sizeof(glm::vec3) * 2 * springs.size(), nullptr, GL_DYNAMIC_DRAW);

    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(glm::vec3), (void*)0);

    glBindVertexArray(0);


}



/**
 * @brief Draw the UI menu using ImGui.
 *
 * Provides controls for physics simulation, spring display, anchors, and cube manipulation.
 */
void Scene::DrawMenu()
{
    ImGui_ImplOpenGL3_NewFrame();
    ImGui_ImplGlfw_NewFrame();
    ImGui::NewFrame();

    ImGui::Begin("UI Box");
    ImGui::Separator();
    ImGui::Text("Physics Controls");
    ImGui::Separator();

    ImGui::Checkbox("Enable Gravity", &enableGravity);
    ImGui::DragFloat3("Gravity", &gravity[0], 0.1f);
    
    
    ImGui::Text("Anchors");

    bool a1Changed = ImGui::DragFloat3("Anchor 1", &anchor1[0], 0.01f);
    bool a2Changed = ImGui::DragFloat3("Anchor 2", &anchor2[0], 0.01f);

    if (a1Changed || a2Changed)
        ApplyAnchorsToSprings();

    if (ImGui::Button("Push Cube"))
        PushCube();

    ImGui::SameLine();

    if (ImGui::Button("Reset"))
        ResetSim();

    ImGui::End();

    ImGui::Render();
    ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

}

/**
 * @brief Build the WorldView and WorldProj matrices based on camera movement and input.
 *
 * Handles keyboard input to move or rotate the camera and zoom in/out.
 */
void Scene::BuildTransforms()
{
    float now = (float)glfwGetTime();
    float dist = (now-last_time)*speed;
    last_time = now;
    if (key == GLFW_KEY_KP_8)
        tr[1] -= dist;
    if (key == GLFW_KEY_KP_2)
        tr[1] += dist;
    if (key == GLFW_KEY_KP_6)
        tr[0] -= dist;
    if (key == GLFW_KEY_KP_4)
        tr[0] += dist;
    if (key == GLFW_KEY_KP_5)
        InitCamera();
    if (key == GLFW_KEY_KP_0)
        spin += dist*20.f;
    if (key == GLFW_KEY_KP_ADD)
        tr[2] = pow(tr[2], 1.0f / 1.01f);
    if (key == GLFW_KEY_KP_SUBTRACT)
        tr[2] = pow(tr[2], 1.01f);


 //   CHECKERROR;
    WorldView = Translate(tr[0], tr[1], -tr[2]) * Rotate(0, tilt - 90) * Rotate(2, spin);
    WorldProj = Perspective((ry*width)/height, ry, front, (mode==0) ? 1000 : back);

}


/**
 * @brief Draw the entire scene.
 *
 * Executes fixed timestep physics simulation, updates spring buffers, draws meshes to g-buffer,
 * applies Phong lighting, renders springs (optional), and handles wireframe/fill mode.
 */
void Scene::DrawScene()
{
    glfwGetFramebufferSize(window, &width, &height);
    glViewport(0, 0, width, height);

    BuildTransforms();
    WorldInverse = glm::inverse(WorldView);

    int loc, programId;

    //
    //charTexture->BindTexture(1, basicShader->programId, "texture1");
    CharTexture->ActivateTexture(0);//Activating Texture


    // --- Layer 5: fixed timestep physics ---
    static double lastTime = glfwGetTime();
    double now = glfwGetTime();
    double frameDt = now - lastTime;
    lastTime = now;

    // Clamp dt so pausing / dragging window won't explode
    if (frameDt > maxFrameDt) frameDt = maxFrameDt;

    physicsAccumulator += (float)frameDt;

    int steps = 0;
    while (physicsAccumulator >= fixedDt && steps < maxSubSteps)
    {
        StepPhysics(fixedDt);
        physicsAccumulator -= fixedDt;
        
        steps++;
    }

    // update spring line buffer once per frame (after physics substeps)
    UpdateSpringBuffer(springVBO, springVertexCount, springs, gCube);


    ////////////G-BUFFER////////////////////
    //
    //Draw the mesh to the g buffer
    gBufferProgram->UseShader();
    programId = gBufferProgram->programId;

    //charTexture->BindTexture(0, basicShader->programId, "texture1");

    gBuffer.BindFBO();
    GLenum DrawBuffers[] = { GL_COLOR_ATTACHMENT0, GL_COLOR_ATTACHMENT1, GL_COLOR_ATTACHMENT2, GL_COLOR_ATTACHMENT3 };
    glDrawBuffers(4, DrawBuffers);

    glViewport(0, 0, width, height);
    //glClearColor(0.8f, 0.8f, 0.8f, 1.f);
    glClearColor(0.2f, 0.2f, 0.2f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    loc = glGetUniformLocation(programId, "WorldProj");
    glUniformMatrix4fv(loc, 1, GL_FALSE, Pntr(WorldProj));
    loc = glGetUniformLocation(programId, "WorldView");
    glUniformMatrix4fv(loc, 1, GL_FALSE, Pntr(WorldView));
    

    float deltaTime = (float)frameDt;
  	UpdateSphereMovement((float)deltaTime);
	boxMesh->Draw(gBufferProgram->programId, deltaTime);


    gBuffer.UnbindFBO();
    gBufferProgram->UnuseShader();
    
      
    glViewport(0, 0, width, height);
    //glClearColor(0.8f, 0.8f, 0.8f, 1.f);
    glClearColor(0.2f, 0.2f, 0.2f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    ////////////PHONG LIGHTING////////////////////
    lightingProgram->UseShader();
    programId = lightingProgram->programId;
    
    loc = glGetUniformLocation(programId, "WorldProj");
    glUniformMatrix4fv(loc, 1, GL_FALSE, Pntr(WorldProj));
    loc = glGetUniformLocation(programId, "WorldView");
    glUniformMatrix4fv(loc, 1, GL_FALSE, Pntr(WorldView));
    loc = glGetUniformLocation(programId, "WorldInverse");
    glUniformMatrix4fv(loc, 1, GL_FALSE, Pntr(WorldInverse));

    loc = glGetUniformLocation(programId, "lightPos");
    glUniform3fv(loc, 1, &(lightPos[0]));   
    loc = glGetUniformLocation(programId, "lightCol");
    glUniform3fv(loc, 1, &(lightCol[0]));

    loc = glGetUniformLocation(programId, "width");
    glUniform1i(loc, width);
    loc = glGetUniformLocation(programId, "height");
    glUniform1i(loc, height);
    loc = glGetUniformLocation(programId, "eyePos");
    glUniform3fv(loc, 1, &(eye[0]));
    
    CHECKERROR;

    gBuffer.BindTexture(1, lightingProgram->programId, "gBufferWorldPos", 0);
    gBuffer.BindTexture(2, lightingProgram->programId, "gBufferNormal", 1);
    gBuffer.BindTexture(3, lightingProgram->programId, "gBufferDiffuse", 2);
    gBuffer.BindTexture(4, lightingProgram->programId, "gBufferSpecular", 3);
    CHECKERROR;

    quad->DrawVAO();
    CHECKERROR; 

    lightingProgram->UnuseShader();

    glClear(GL_DEPTH_BUFFER_BIT);
    glBindFramebuffer(GL_READ_FRAMEBUFFER, gBuffer.fbo);
    glBindFramebuffer(GL_DRAW_FRAMEBUFFER, 0);
    glBlitFramebuffer(0, 0, width, height,
        0, 0, width, height,
        GL_DEPTH_BUFFER_BIT,
        GL_NEAREST);
    glBindFramebuffer(GL_FRAMEBUFFER, 0);



    //Springs
    if (showSprings && debugProgram && springVertexCount > 0)
    {
        glBindFramebuffer(GL_FRAMEBUFFER, 0);

        glDisable(GL_CULL_FACE);

        glDisable(GL_DEPTH_TEST);

        glLineWidth(3.0f);

        glDepthMask(GL_FALSE);

        debugProgram->UseShader();
        int programId = debugProgram->programId;

        int loc = glGetUniformLocation(programId, "WorldProj");
        glUniformMatrix4fv(loc, 1, GL_FALSE, Pntr(WorldProj));

        loc = glGetUniformLocation(programId, "WorldView");
        glUniformMatrix4fv(loc, 1, GL_FALSE, Pntr(WorldView));

        // spring color
        loc = glGetUniformLocation(programId, "uColor");
        glm::vec3 col(1.0f, 1.0f, 0.0f);
        glUniform3fv(loc, 1, &col[0]);

        glBindVertexArray(springVAO);
        glDrawArrays(GL_LINES, 0, springVertexCount);
        glBindVertexArray(0);

        debugProgram->UnuseShader();


        glDepthMask(GL_TRUE);
        glLineWidth(1.0f);
        glEnable(GL_DEPTH_TEST);
        glEnable(GL_CULL_FACE);
    }


    // Handle wireframe mode
    if (mesh == false) {
        glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
    }
    else {
        glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
    }


}



/**
 * @brief Update the position of a sphere (or box) based on input flags.
 *
 * @param deltaTime Time step (seconds)
 */
void Scene::UpdateSphereMovement(float deltaTime)
{
    bool sphereMoved = false;
    float moveDistance = boxMoveSpeed * deltaTime;

    if (moveLeft) {
        boxPosition.x -= moveDistance;
        sphereMoved = true;
    }
    if (moveRight) {
        boxPosition.x += moveDistance;
        sphereMoved = true;
    }
    if (moveUp) {
        boxPosition.y += moveDistance;
        sphereMoved = true;
    }
    if (moveDown) {
        boxPosition.y -= moveDistance;
        sphereMoved = true;
    }
    if (moveForward) {
        boxPosition.z += moveDistance;
        sphereMoved = true;
    }
    if (moveBackward) {
        boxPosition.z -= moveDistance;
        sphereMoved = true;
    }


}



/**
 * @brief Apply the force of a spring to a RigidBody.
 *
 * Calculates spring force using Hooke's law and damping, applies to attachment points.
 *
 * @param s Spring to apply
 * @param body RigidBody the spring is attached to
 * @param dt Time step (seconds)
 */
void Scene::applySpring(const Spring& s,  RigidBody& body, float dt)
{
    // Get world positions
    glm::vec3 pa, pb;

    if (s.a.type == SpringEndType::RigidPoint)
        pa = body.pointWorld(s.a.index);
    else
        pa = s.a.worldPos;

    if (s.b.type == SpringEndType::RigidPoint)
        pb = body.pointWorld(s.b.index);
    else
        pb = s.b.worldPos;

    glm::vec3 d = pa - pb;
    float len = glm::length(d);
    if (len < 1e-6f) return;

    glm::vec3 n = d / len;

    // Velocity of attachment points
    auto pointVelocity = [&](const SpringEnd& e, const glm::vec3& p)
        {
            if (e.type == SpringEndType::RigidPoint)
                return body.v + glm::cross(body.w, p - body.x);
            else
                return glm::vec3(0.0f);
        };

    glm::vec3 va = pointVelocity(s.a, pa);
    glm::vec3 vb = pointVelocity(s.b, pb);
    float vrel = glm::dot(va - vb, n);

    float fmag = -s.k * (len - s.L0) - s.c * vrel;
    glm::vec3 F = fmag * n;

    if (s.a.type == SpringEndType::RigidPoint)
        body.applyForceAtPoint(F, pa);

    if (s.b.type == SpringEndType::RigidPoint)
        body.applyForceAtPoint(-F, pb);
}

/**
 * @brief Step the physics simulation for the rigid body.
 *
 * Applies gravity, spring forces, and integrates linear and angular momentum using
 * symplectic Euler integration.
 *
 * @param h Fixed timestep (seconds)
 */
void Scene::StepPhysics(float h)
{
    gCube.clearAccumulators();

    // Gravity
    if (enableGravity) {
        gCube.forceAccum += gCube.M * gravity;
    }

    for (const Spring& s : springs)
        applySpring(s, gCube, h);

    gCube.P += h * gCube.forceAccum;
    gCube.L += h * gCube.torqueAccum;

    gCube.integrateStateSymplecticEuler(h);
}

//
/**
 * @brief Apply updated anchor positions to the spring array.
 *
 * Assumes the first two springs in the vector are connected to the anchors.
 */
void Scene::ApplyAnchorsToSprings()
{
    // assumes springs[0] and springs[1] are the anchor springs
    if (springs.size() >= 2)
    {
        springs[0].b.worldPos = anchor1;
        springs[1].b.worldPos = anchor2;
    }
}

/**
 * @brief Apply an instantaneous push to the cube along the Z-axis.
 */
void Scene::PushCube()
{
    gCube.P += glm::vec3(0.0f, 0.0f, -0.8f);
}


/**
 * @brief Reset the simulation state.
 *
 * Resets rigid body position, rotation, momentum, anchor positions, and physics accumulator.
 */
void Scene::ResetSim()
{
    gCube.x = glm::vec3(0.0f);
    gCube.R = glm::mat3(1.0f);
    gCube.P = glm::vec3(0.0f);
    gCube.L = glm::vec3(0.0f);

    anchor1 = glm::vec3(-0.5f, 1.5f, 0.0f);
    anchor2 = glm::vec3(0.5f, 1.5f, 0.0f);

    ApplyAnchorsToSprings();
    physicsAccumulator = 0.0f;
}



