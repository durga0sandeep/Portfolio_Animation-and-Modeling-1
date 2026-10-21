////////////////////////////////////////////////////////////////////////
// The scene class contains all the parameters needed to define and
// draw a simple scene, including:
//   * Geometry
//   * Light parameters
//   * Material properties
//   * Viewport size parameters
//   * Viewing transformation values
//   * others ...
//
// Some of these parameters are set when the scene is built, and
// others are set by the framework in response to user mouse/keyboard
// interactions.  All of them can be used to draw the scene.

#include "shapes.h"
#include "texture.h"
#include "fbo.h"
#include "gbuffer.h"
#include "Mesh.h"
#include "RigidBody.h"
#include "Spring.h"

class Shader;


class Scene
{
public:
    GLFWwindow* window = nullptr;

    std::string meshPat = "fbx/fbx/roman_D.fbx";
    const char* current_model = "Roman";

	std::string boxPath = "fbx/fbx/box.fbx";

    static RigidBody gCube;

    // Light parameters
    float lightSpin, lightTilt, lightDist;
    glm::vec3 lightPos = {-30.0f, -60.f, 90.0f};
    glm::vec3 lightCol = {1.0f, 1.0f, 1.0f};
    glm::vec3 lightDir = {1.0f, -1.0f, -1.0f};

    //buffer
    GBuffer gBuffer;
    unsigned int id, bindpoint;

    long double start, curr, dt;


    int key;
    float spin, tilt, speed, ry, front, back;
    glm::vec3 eye;
    glm::vec3 tr;
    float last_time;
    int mode; 

    
    // Viewport
    int width, height;

    // Transformations
    glm::mat4 WorldProj, WorldView, WorldInverse;
    glm::mat4 lightProj, lightView;
    glm::mat4 ResultMat;

    //Mesh* model;
    Shape* quad; //deferred shading
    Texture* CharTexture;
	Mesh* boxMesh;

    glm::vec3 boxPosition;
    float boxMoveSpeed;

    //UI
    bool play = true;
    bool bone = false;
    bool mesh = true;

    // Shader programs
    ShaderProgram* lightingProgram;
    ShaderProgram* gBufferProgram;
    ShaderProgram* basicShader;

    ShaderProgram* BoneProgram;
	ShaderProgram* CurveProgram;

    void InitCamera();
    void InitializeScene();
    void BuildTransforms();
    void DrawMenu();
    void DrawScene();
    
    void UpdateSphereMovement(float deltaTime);

    void applySpring(const Spring& s, RigidBody& body, float dt);


    void UpdateSpringBuffer(GLuint springVBO, GLsizei& springVertexCount,
        const std::vector<Spring>& springs, const RigidBody& body);

    bool moveLeft;
    bool moveRight;
    bool moveUp;
    bool moveDown;
    bool moveForward;
    bool moveBackward;
	
    glm::vec3 characterPosition;

    void StepPhysics(float h);

    void ResetSim();
    void PushCube();
    void ApplyAnchorsToSprings();


private:

    ShaderProgram* debugProgram = nullptr;
    GLuint springVAO = 0, springVBO = 0;
    GLsizei springVertexCount = 0;

    bool showCurve = true;

    std::vector<Spring> springs;

    float physicsAccumulator = 0.0f;
    float fixedDt = 1.0f / 240.0f;     // stable for springs
    float maxFrameDt = 1.0f / 30.0f;   // clamp huge dt spikes
    int maxSubSteps = 8;

    // ---------- Layer 5: Controls ----------
    float anchorMoveSpeed = 1.5f;      // units/sec
    bool showSprings = true;           // you already have this
    bool depthSprings = false;         // toggle depth test for springs
    
    bool physicsRunning = true;     // replaces play if you want

    glm::vec3 anchor1 = glm::vec3(-0.5f, 1.5f, 0.0f);
    glm::vec3 anchor2 = glm::vec3(0.5f, 1.5f, 0.0f);

    glm::vec3 gravity = glm::vec3(0.0f, -9.81f, 0.0f);
    bool enableGravity = true;

};





