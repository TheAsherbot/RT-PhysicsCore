#include "RT-PhysicsCore/rendering/Renderer.h"
#include "RT-PhysicsCore/rendering/components/MeshComponent.h"
#include "RT-PhysicsCore/core/ecs/components/TransformComponent.h"
#include "RT-PhysicsCore/utils/DebugDraw.h"
#include "RT-PhysicsCore/utils/Log.h"

// glad must be included before GLFW - GLFW pulls in system GL headers
// unless it detects a loader was already included, which causes macro
// redefinition errors if the order is reversed.
#include <glad/gl.h>
#include <GLFW/glfw3.h>

#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/gtc/type_ptr.hpp>

#include <cmath>
#include <cstddef>
#include <unordered_map>
#include <vector>

namespace RT_PhysicsCore
{
    namespace
    {
        constexpr float kPi = 3.14159265358979323846f;

        // ---- shaders -------------------------------------------------
        // Embedded as string literals rather than loaded from disk: with
        // only two small shaders, this avoids "where does this relative
        // path resolve from" entirely (the same problem the logs/ folder
        // had) at essentially no cost. Worth switching to file-based
        // loading only if shader iteration speed becomes a real need.

        const char* kMeshVertexSrc = R"(
#version 330 core
layout (location = 0) in vec3 aPos;
layout (location = 1) in vec3 aNormal;

uniform mat4 uModel;
uniform mat4 uView;
uniform mat4 uProjection;

out vec3 vNormal;
out vec3 vFragPos;

void main()
{
    vFragPos = vec3(uModel * vec4(aPos, 1.0));
    vNormal = mat3(transpose(inverse(uModel))) * aNormal;
    gl_Position = uProjection * uView * vec4(vFragPos, 1.0);
}
)";

        const char* kMeshFragmentSrc = R"(
#version 330 core
in vec3 vNormal;
in vec3 vFragPos;

uniform vec3 uColor;
uniform vec3 uLightDir;

out vec4 FragColor;

void main()
{
    vec3 normal = normalize(vNormal);
    float diff = max(dot(normal, -uLightDir), 0.0);
    vec3 ambient = 0.3 * uColor;
    vec3 diffuse = 0.7 * diff * uColor;
    FragColor = vec4(ambient + diffuse, 1.0);
}
)";

        const char* kDebugLineVertexSrc = R"(
#version 330 core
layout (location = 0) in vec3 aPos;
layout (location = 1) in vec3 aColor;

uniform mat4 uView;
uniform mat4 uProjection;

out vec3 vColor;

void main()
{
    vColor = aColor;
    gl_Position = uProjection * uView * vec4(aPos, 1.0);
}
)";

        const char* kDebugLineFragmentSrc = R"(
#version 330 core
in vec3 vColor;
out vec4 FragColor;

void main()
{
    FragColor = vec4(vColor, 1.0);
}
)";

        // ---- primitive geometry ---------------------------------------
        // Verified separately: vertex counts, bounds, unit normals, and
        // (for the cube) winding order all checked against expected values
        // before this went anywhere near GL.

        struct MeshVertex { glm::vec3 position; glm::vec3 normal; };

        std::vector<MeshVertex> GenerateCube()
        {
            std::vector<MeshVertex> v;
            auto face = [&](glm::vec3 n, glm::vec3 a, glm::vec3 b, glm::vec3 c, glm::vec3 d)
                {
                    v.push_back({ a, n }); v.push_back({ b, n }); v.push_back({ c, n });
                    v.push_back({ a, n }); v.push_back({ c, n }); v.push_back({ d, n });
                };
            face({ 0,0,1 }, { -0.5f,-0.5f,0.5f }, { 0.5f,-0.5f,0.5f }, { 0.5f,0.5f,0.5f }, { -0.5f,0.5f,0.5f });
            face({ 0,0,-1 }, { 0.5f,-0.5f,-0.5f }, { -0.5f,-0.5f,-0.5f }, { -0.5f,0.5f,-0.5f }, { 0.5f,0.5f,-0.5f });
            face({ 1,0,0 }, { 0.5f,-0.5f,0.5f }, { 0.5f,-0.5f,-0.5f }, { 0.5f,0.5f,-0.5f }, { 0.5f,0.5f,0.5f });
            face({ -1,0,0 }, { -0.5f,-0.5f,-0.5f }, { -0.5f,-0.5f,0.5f }, { -0.5f,0.5f,0.5f }, { -0.5f,0.5f,-0.5f });
            face({ 0,1,0 }, { -0.5f,0.5f,0.5f }, { 0.5f,0.5f,0.5f }, { 0.5f,0.5f,-0.5f }, { -0.5f,0.5f,-0.5f });
            face({ 0,-1,0 }, { -0.5f,-0.5f,-0.5f }, { 0.5f,-0.5f,-0.5f }, { 0.5f,-0.5f,0.5f }, { -0.5f,-0.5f,0.5f });
            return v;
        }

        std::vector<MeshVertex> GeneratePlane()
        {
            // Unit quad on XZ, facing +Y - matches this project's Y-up convention.
            std::vector<MeshVertex> v;
            glm::vec3 n(0.0f, 1.0f, 0.0f);
            v.push_back({ {-0.5f, 0.0f, -0.5f}, n });
            v.push_back({ { 0.5f, 0.0f, -0.5f}, n });
            v.push_back({ { 0.5f, 0.0f,  0.5f}, n });
            v.push_back({ {-0.5f, 0.0f, -0.5f}, n });
            v.push_back({ { 0.5f, 0.0f,  0.5f}, n });
            v.push_back({ {-0.5f, 0.0f,  0.5f}, n });
            return v;
        }

        std::vector<MeshVertex> GenerateSphere(int rings = 16, int segments = 24)
        {
            std::vector<MeshVertex> v;
            const float radius = 0.5f;
            for (int i = 0; i < rings; ++i)
            {
                float lat0 = kPi * (-0.5f + static_cast<float>(i) / rings);
                float lat1 = kPi * (-0.5f + static_cast<float>(i + 1) / rings);
                float y0 = std::sin(lat0), r0 = std::cos(lat0);
                float y1 = std::sin(lat1), r1 = std::cos(lat1);

                for (int j = 0; j < segments; ++j)
                {
                    float lon0 = 2.0f * kPi * static_cast<float>(j) / segments;
                    float lon1 = 2.0f * kPi * static_cast<float>(j + 1) / segments;

                    glm::vec3 p00(r0 * std::cos(lon0), y0, r0 * std::sin(lon0));
                    glm::vec3 p01(r0 * std::cos(lon1), y0, r0 * std::sin(lon1));
                    glm::vec3 p10(r1 * std::cos(lon0), y1, r1 * std::sin(lon0));
                    glm::vec3 p11(r1 * std::cos(lon1), y1, r1 * std::sin(lon1));

                    v.push_back({ p00 * radius, glm::normalize(p00) });
                    v.push_back({ p10 * radius, glm::normalize(p10) });
                    v.push_back({ p11 * radius, glm::normalize(p11) });

                    v.push_back({ p00 * radius, glm::normalize(p00) });
                    v.push_back({ p11 * radius, glm::normalize(p11) });
                    v.push_back({ p01 * radius, glm::normalize(p01) });
                }
            }
            return v;
        }

        // ---- small GL helpers -----------------------------------------

        GLuint CompileShader(GLenum type, const char* src, const char* label)
        {
            GLuint shader = glCreateShader(type);
            glShaderSource(shader, 1, &src, nullptr);
            glCompileShader(shader);
            GLint success = 0;
            glGetShaderiv(shader, GL_COMPILE_STATUS, &success);
            if (!success)
            {
                char log[1024];
                glGetShaderInfoLog(shader, sizeof(log), nullptr, log);
                RT_LOG_ERROR("Shader compile error (" << label << "): " << log);
            }
            return shader;
        }

        GLuint LinkProgram(GLuint vs, GLuint fs, const char* label)
        {
            GLuint program = glCreateProgram();
            glAttachShader(program, vs);
            glAttachShader(program, fs);
            glLinkProgram(program);
            GLint success = 0;
            glGetProgramiv(program, GL_LINK_STATUS, &success);
            if (!success)
            {
                char log[1024];
                glGetProgramInfoLog(program, sizeof(log), nullptr, log);
                RT_LOG_ERROR("Shader link error (" << label << "): " << log);
            }
            return program;
        }

        struct PrimitiveMesh
        {
            GLuint vao = 0;
            GLuint vbo = 0;
            GLsizei vertexCount = 0;
        };

        PrimitiveMesh UploadMesh(const std::vector<MeshVertex>& vertices)
        {
            PrimitiveMesh mesh;
            mesh.vertexCount = static_cast<GLsizei>(vertices.size());
            glGenVertexArrays(1, &mesh.vao);
            glBindVertexArray(mesh.vao);
            glGenBuffers(1, &mesh.vbo);
            glBindBuffer(GL_ARRAY_BUFFER, mesh.vbo);
            glBufferData(GL_ARRAY_BUFFER,
                static_cast<GLsizeiptr>(vertices.size() * sizeof(MeshVertex)),
                vertices.data(), GL_STATIC_DRAW);
            glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(MeshVertex),
                reinterpret_cast<void*>(offsetof(MeshVertex, position)));
            glEnableVertexAttribArray(0);
            glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(MeshVertex),
                reinterpret_cast<void*>(offsetof(MeshVertex, normal)));
            glEnableVertexAttribArray(1);
            glBindVertexArray(0);
            return mesh;
        }

        void FramebufferSizeCallback(GLFWwindow*, int width, int height)
        {
            glViewport(0, 0, width, height);
        }
    } // namespace

    struct Renderer::Impl
    {
        GLFWwindow* window = nullptr;
        bool valid = false;

        GLuint meshProgram = 0;
        GLuint debugLineProgram = 0;
        std::unordered_map<PrimitiveShape, PrimitiveMesh> primitives;

        GLuint debugLineVao = 0;
        GLuint debugLineVbo = 0;

        Camera camera;
        Input input;
        double lastFrameTime = 0.0;
    };

    Renderer::Renderer(int width, int height, const char* title)
        : impl(std::make_unique<Impl>())
    {
        if (!glfwInit())
        {
            RT_LOG_FATAL("glfwInit() failed");
            return;
        }

        glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
        glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
        glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
        glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GLFW_TRUE); // required on macOS, harmless elsewhere

        impl->window = glfwCreateWindow(width, height, title, nullptr, nullptr);
        if (!impl->window)
        {
            RT_LOG_FATAL("glfwCreateWindow() failed - check driver/GL 3.3 core support");
            glfwTerminate();
            return;
        }

        glfwMakeContextCurrent(impl->window);
        glfwSetFramebufferSizeCallback(impl->window, FramebufferSizeCallback);

        impl->input.AttachWindow(impl->window);

        int gladVersion = gladLoadGL(glfwGetProcAddress);
        if (gladVersion == 0)
        {
            RT_LOG_FATAL("gladLoadGL() failed - could not load OpenGL function pointers");
            glfwDestroyWindow(impl->window);
            impl->window = nullptr;
            glfwTerminate();
            return;
        }
        RT_LOG_INFO("OpenGL " << GLAD_VERSION_MAJOR(gladVersion) << "."
            << GLAD_VERSION_MINOR(gladVersion) << " loaded");

        glEnable(GL_DEPTH_TEST);
        glViewport(0, 0, width, height);

        GLuint meshVS = CompileShader(GL_VERTEX_SHADER, kMeshVertexSrc, "mesh vertex");
        GLuint meshFS = CompileShader(GL_FRAGMENT_SHADER, kMeshFragmentSrc, "mesh fragment");
        impl->meshProgram = LinkProgram(meshVS, meshFS, "mesh");
        glDeleteShader(meshVS);
        glDeleteShader(meshFS);

        GLuint lineVS = CompileShader(GL_VERTEX_SHADER, kDebugLineVertexSrc, "debug line vertex");
        GLuint lineFS = CompileShader(GL_FRAGMENT_SHADER, kDebugLineFragmentSrc, "debug line fragment");
        impl->debugLineProgram = LinkProgram(lineVS, lineFS, "debug line");
        glDeleteShader(lineVS);
        glDeleteShader(lineFS);

        impl->primitives[PrimitiveShape::Cube] = UploadMesh(GenerateCube());
        impl->primitives[PrimitiveShape::Sphere] = UploadMesh(GenerateSphere());
        impl->primitives[PrimitiveShape::Plane] = UploadMesh(GeneratePlane());

        glGenVertexArrays(1, &impl->debugLineVao);
        glBindVertexArray(impl->debugLineVao);
        glGenBuffers(1, &impl->debugLineVbo);
        glBindBuffer(GL_ARRAY_BUFFER, impl->debugLineVbo);
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(DebugLineVertex),
            reinterpret_cast<void*>(offsetof(DebugLineVertex, position)));
        glEnableVertexAttribArray(0);
        glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(DebugLineVertex),
            reinterpret_cast<void*>(offsetof(DebugLineVertex, color)));
        glEnableVertexAttribArray(1);
        glBindVertexArray(0);

        impl->lastFrameTime = glfwGetTime();
        impl->valid = true;

        RT_LOG_INFO("Renderer initialized: " << width << "x" << height << " '" << title << "'");
    }

    Renderer::~Renderer()
    {
        if (impl && impl->valid)
        {
            glDeleteProgram(impl->meshProgram);
            glDeleteProgram(impl->debugLineProgram);
            for (auto& [shape, mesh] : impl->primitives)
            {
                glDeleteVertexArrays(1, &mesh.vao);
                glDeleteBuffers(1, &mesh.vbo);
            }
            glDeleteVertexArrays(1, &impl->debugLineVao);
            glDeleteBuffers(1, &impl->debugLineVbo);
        }

        if (impl && impl->window)
            glfwDestroyWindow(impl->window);

        glfwTerminate(); // documented safe to call even if init never succeeded
    }

    bool Renderer::IsValid() const { return impl->valid; }

    bool Renderer::ShouldClose() const
    {
        return impl->valid ? glfwWindowShouldClose(impl->window) : true;
    }

    void Renderer::BeginFrame()
    {
        if (!impl->valid) return;

        glfwPollEvents();
        impl->input.Update();

        double now = glfwGetTime();
        float dt = static_cast<float>(now - impl->lastFrameTime);
        impl->lastFrameTime = now;
        impl->camera.ProcessInput(impl->input, dt);

        glClearColor(0.08f, 0.08f, 0.10f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    }

    void Renderer::DrawMesh(const MeshComponent& mesh, const WorldTransformComponent& worldTransform)
    {
        if (!impl->valid) return;

        auto it = impl->primitives.find(mesh.shape);
        if (it == impl->primitives.end())
            return;

        // T * R * S, applied to a local vertex as (T * (R * (S * v))):
        // scale happens first (around the mesh's own local origin), then
        // rotate, then translate into world position - the same order
        // TransformPropagationSystem already uses when it composes world
        // transforms, so this stays consistent with it.
        glm::mat4 model = glm::translate(glm::mat4(1.0f), worldTransform.worldPosition)
            * glm::mat4_cast(worldTransform.worldRotation)
            * glm::scale(glm::mat4(1.0f), worldTransform.worldScale);

        int fbWidth = 1, fbHeight = 1;
        glfwGetFramebufferSize(impl->window, &fbWidth, &fbHeight);
        float aspect = fbHeight > 0 ? static_cast<float>(fbWidth) / static_cast<float>(fbHeight) : 1.0f;

        glUseProgram(impl->meshProgram);
        glUniformMatrix4fv(glGetUniformLocation(impl->meshProgram, "uModel"), 1, GL_FALSE, glm::value_ptr(model));
        glUniformMatrix4fv(glGetUniformLocation(impl->meshProgram, "uView"), 1, GL_FALSE,
            glm::value_ptr(impl->camera.GetViewMatrix()));
        glUniformMatrix4fv(glGetUniformLocation(impl->meshProgram, "uProjection"), 1, GL_FALSE,
            glm::value_ptr(impl->camera.GetProjectionMatrix(aspect)));
        glUniform3fv(glGetUniformLocation(impl->meshProgram, "uColor"), 1, glm::value_ptr(mesh.color));
        glm::vec3 lightDir = glm::normalize(glm::vec3(-0.4f, -1.0f, -0.3f));
        glUniform3fv(glGetUniformLocation(impl->meshProgram, "uLightDir"), 1, glm::value_ptr(lightDir));

        glBindVertexArray(it->second.vao);
        glDrawArrays(GL_TRIANGLES, 0, it->second.vertexCount);
    }

    void Renderer::FlushDebugDraw()
    {
        if (!impl->valid) return;

        std::vector<DebugLineVertex> lines = DebugDraw::TakeLines();
        if (lines.empty())
            return;

        glBindBuffer(GL_ARRAY_BUFFER, impl->debugLineVbo);
        glBufferData(GL_ARRAY_BUFFER, static_cast<GLsizeiptr>(lines.size() * sizeof(DebugLineVertex)),
            lines.data(), GL_DYNAMIC_DRAW);

        int fbWidth = 1, fbHeight = 1;
        glfwGetFramebufferSize(impl->window, &fbWidth, &fbHeight);
        float aspect = fbHeight > 0 ? static_cast<float>(fbWidth) / static_cast<float>(fbHeight) : 1.0f;

        glUseProgram(impl->debugLineProgram);
        glUniformMatrix4fv(glGetUniformLocation(impl->debugLineProgram, "uView"), 1, GL_FALSE,
            glm::value_ptr(impl->camera.GetViewMatrix()));
        glUniformMatrix4fv(glGetUniformLocation(impl->debugLineProgram, "uProjection"), 1, GL_FALSE,
            glm::value_ptr(impl->camera.GetProjectionMatrix(aspect)));

        glBindVertexArray(impl->debugLineVao);
        glDrawArrays(GL_LINES, 0, static_cast<GLsizei>(lines.size()));
    }

    void Renderer::EndFrame()
    {
        if (!impl->valid) return;
        glfwSwapBuffers(impl->window);
    }

    Camera& Renderer::GetCamera() { return impl->camera; }
    const Camera& Renderer::GetCamera() const { return impl->camera; }

    Input& Renderer::GetInput() { return impl->input; }
    const Input& Renderer::GetInput() const { return impl->input; }
}
