/**
 * @file scene.cpp
 * @brief Main scene implementation handling rendering, animation, UI, and camera logic.
 *
 * This file initializes and renders a deferred shading scene with:
 *  - Skinned animated character
 *  - Path-following using Bezier curves
 *  - Animation blending via AnimationController
 *  - Bone debugging
 *  - ImGui-based UI controls
 *
 */

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
#include "AnimationController.h"



const float PI = 3.14159f;
const float rad = PI/180.0f;    

glm::mat4 Identity;

#define CHECKERROR {GLenum err = glGetError(); if (err != GL_NO_ERROR) { fprintf(stderr, "OpenGL error (at line scene.cpp:%d): %s\n", __LINE__, gluErrorString(err)); exit(-1);} }

/**
 * @brief Initializes camera parameters
 *
 * Sets default position, orientation, movement speed,
 * clipping planes, and projection parameters.
 */
void Scene::InitCamera()
{
    spin = 0.0;
    tilt = 15.0;
    eye = glm::vec3(0.0, -20.0, 10.0);
    speed = 300.0 / 30.0;
    last_time = (float)glfwGetTime();
    tr = glm::vec3(0.0, -3.0, 25.0);

    ry = 0.4f;
    front = 0.5;
    back = 5000.0;
}

/**
 * @brief Initializes all scene resources
 *
 * This includes:
 *  - OpenGL states
 *  - G-buffer setup (deferred shading)
 *  - Shader programs
 *  - Model and animation clips
 *  - Bezier curve path
 *  - Path follower
 *  - Animation controller
 */
void Scene::InitializeScene()
{
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
    
    
    BoneProgram = new ShaderProgram();
    BoneProgram->AddShader("shaders/bone.vert", GL_VERTEX_SHADER);
    BoneProgram->AddShader("shaders/bone.frag", GL_FRAGMENT_SHADER);
    glBindAttribLocation(BoneProgram->programId, 0, "aPos");
    BoneProgram->LinkProgram();
    
	CurveProgram = new ShaderProgram();
	CurveProgram->AddShader("shaders/curve.vert", GL_VERTEX_SHADER);
	CurveProgram->AddShader("shaders/curve.frag", GL_FRAGMENT_SHADER);
	glBindAttribLocation(CurveProgram->programId, 0, "aPos");
	CurveProgram->LinkProgram();


    CharTexture = new Texture("fbx/textures/roman_blue_D_texture.jpg");

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

    model = new Mesh();
    model->LoadMesh(meshPat);


    /*
    * 
    Idle - 0 to 338
    Run - 338 to 362
    Death - 362 to 442
    Attack - 442 to 510
    T-Pose - 510 to 540
    540
    */

    model->DefineAnimationClip("idle", 0, 0, 338, true);
	model->SetAnimationYOffset("idle", 0.0f);
    model->DefineAnimationClip("run", 0, 339, 361, true);
	model->SetAnimationYOffset("run", 0.0f);
    model->DefineAnimationClip("death", 0, 362, 442, true);
	model->SetAnimationYOffset("death", 0.0f);
    model->DefineAnimationClip("attack", 0, 442, 510, true);
	model->SetAnimationYOffset("attack", 0.0f);
    model->DefineAnimationClip("tpose", 0, 510, 540, true);
	model->SetAnimationYOffset("tpose", 0.0f);
    
	model->AddFullTrackAsClip(1,"walk", true);
	model->SetAnimationYOffset("walk", 3.0f);

	model->SetAnimation("run");
	model->SetAnimationYOffset("run", 0.0f);

    //
    // Initialize Bezier Curve
    bezierCurve = new PathGenerator(100);
    //
	//Path Follower Initialization
	follower = new PathFollower(bezierCurve, 15.0f); // Speed = 5 units/sec

    // RECOMMENDED: Use look-ahead with banking for smooth path following
    follower->setOrientationMode(PathFollower::LOOK_AHEAD_WITH_BANKING);
    follower->setLookAheadDistance(5.0f);  // Look 5% ahead on the path
    follower->setBankingFactor(0.4f);      // Moderate banking into turns
    follower->setLooping(true);
    follower->start();

	float PathMaxSpeed = follower->getMaxSpeed();

//    float walkspeed = PathMaxSpeed * 0.8f;//0.35f;
//    float runspeed = PathMaxSpeed;

    /*
    //
    //Current Speed
    float walkspeed = PathMaxSpeed / 0.5f;//0.35f;
    float runspeed = PathMaxSpeed / 0.5f;
    */
    //New changes
    float walkspeed = 2.0f;//0.35f;
    float runspeed = 5.0f;


/*
    float walkspeed = 4.0f;
    float runspeed = 10.0f;
*/

    animController = new AnimationController(*model, *follower, walkspeed, runspeed, 0.5f);
    animController->SetMeshPrgId(gBufferProgram->programId);
    animController->setDebugOutput(true);


    //start = glfwGetTime();

}




void Scene::DrawMenu()
{
    ImGui_ImplOpenGL3_NewFrame();
    ImGui_ImplGlfw_NewFrame();
    ImGui::NewFrame();

    ImGui::Begin("Sample UI Box");
    ImGui::Text("sample text");

    ImGui::Checkbox("Draw Bone", &bone);
    ImGui::SameLine();
    ImGui::Checkbox("Draw Mesh", &mesh);

    ImGui::Separator();

    // Bezier Curve Controls
    ImGui::Text("Bezier Curve Controls");
    ImGui::Checkbox("Show Curve", &showCurve);

    if (showCurve && bezierCurve) {
        if (ImGui::CollapsingHeader("Control Points")) {
            //ImGui::Begin("Path Editor");

            if (ImGui::Button("Reset Path Curve")) {
                float radius = 15.0f;
                float k = 0.552284f; // Handle length for circle

                // Setup 9 points for 3 segments in a circle
                for (int i = 0; i < 3; i++) {
                    float angle = (i * 2.0f * 3.14159f) / 3.0f;
                    float nextAngle = ((i + 1) * 2.0f * 3.14159f) / 3.0f;

                    
                    // The Node
                    bezierCurve->controlPoints[i * 3] = glm::vec3(cos(angle) * radius, sin(angle) * radius, 0);

                    // The outgoing handle (Right)
                    glm::vec3 tangent = glm::vec3(-sin(angle), cos(angle), 0);
                    bezierCurve->controlPoints[i * 3 + 1] = bezierCurve->controlPoints[i * 3] + tangent * (radius * 0.8f);

                    // The incoming handle (Left)
                    glm::vec3 nextNode = glm::vec3(cos(nextAngle) * radius, sin(nextAngle) * radius, 0);
                    glm::vec3 nextTangent = glm::vec3(-sin(nextAngle), cos(nextAngle), 0);
                    bezierCurve->controlPoints[i * 3 + 2] = nextNode - nextTangent * (radius * 0.8f);
                }
                bezierCurve->updateCurveVertices();
            }


            for (int i = 0; i < bezierCurve->getNumPoints(); i++)
            {
                ImGui::PushID(i);  // unique stack ID

                glm::vec3 p = bezierCurve->getControlPoint(i);
                float pos[3] = { p.x, p.y, p.z };

                const char* visibleLabel = (i % 3 == 0) ? "Node" : "Handle";

                // Hidden unique ID: visibleLabel##i
                std::string internalLabel = std::string(visibleLabel) + "##" + std::to_string(i);

                if (ImGui::DragFloat3(internalLabel.c_str(), pos, 0.1f))
                {
                    bezierCurve->setControlPoint(i, glm::vec3(pos[0], pos[1], pos[2]));
                }

                ImGui::PopID();
            }


        }
    }

    ImGui::End();


    ImGui::Render();
    ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

}

/**
 * @brief Builds camera transformations
 *
 * Handles keyboard input for camera translation,
 * rotation, zoom, and projection matrix creation.
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
 * @brief Renders the entire scene
 *
 * Rendering pipeline:
 *  1. G-buffer pass (geometry)
 *  2. Lighting pass (fullscreen quad)
 *  3. Bone debug overlay
 *  4. Bezier curve visualization
 */
void Scene::DrawScene()
{
    glfwGetFramebufferSize(window, &width, &height);
    glViewport(0, 0, width, height);

    BuildTransforms();
    WorldInverse = glm::inverse(WorldView);

    int loc, programId;

    CharTexture->ActivateTexture(0);//Activating Texture


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
    glClearColor(0.8f, 0.8f, 0.8f, 1.f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    loc = glGetUniformLocation(programId, "WorldProj");
    glUniformMatrix4fv(loc, 1, GL_FALSE, Pntr(WorldProj));
    loc = glGetUniformLocation(programId, "WorldView");
    glUniformMatrix4fv(loc, 1, GL_FALSE, Pntr(WorldView));


    static long double lastFrame = glfwGetTime();
    long double currentFrame = glfwGetTime();
    long double deltaTime = currentFrame - lastFrame;
    lastFrame = currentFrame;

    // Cap delta time to prevent huge jumps
    if (deltaTime > 0.1) {
        deltaTime = 0.1;
    }

    animController->update((float)deltaTime);
    

    gBuffer.UnbindFBO();
    gBufferProgram->UnuseShader();

 
    glViewport(0, 0, width, height);
    glClearColor(0.8f, 0.8f, 0.8f, 1.f);
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


    if (bone == true)
    {
        BoneProgram->UseShader();
        glm::mat4 viewProj = WorldProj * WorldView;
        model->DebugSkeleton(viewProj, BoneProgram->programId);
        BoneProgram->UnuseShader();
    }


    // Draw Bezier Curve
    if (showCurve && bezierCurve)
    {
        CurveProgram->UseShader();
        programId = CurveProgram->programId;

        glm::mat4 viewProj = WorldProj * WorldView;
        loc = glGetUniformLocation(programId, "viewProj");
        glUniformMatrix4fv(loc, 1, GL_FALSE, Pntr(viewProj));

        glLineWidth(3.0f);
        bezierCurve->draw();
        glLineWidth(1.0f);

        CurveProgram->UnuseShader();
    }


    // Handle wireframe mode
    if (mesh == false) {
        glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
    }
    else {
        glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
    }


}




