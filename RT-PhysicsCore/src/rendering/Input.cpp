#include "RT-PhysicsCore/rendering/Input.h"
#include <GLFW/glfw3.h>
#include <cstring>

namespace RT_PhysicsCore
{
    void Input::AttachWindow(GLFWwindow* w)
    {
        window = w;
        if (window)
        {
            glfwGetCursorPos(window, &lastMouseX, &lastMouseY);
            mouseX = lastMouseX;
            mouseY = lastMouseY;
        }
        firstUpdate = true;
    }

    void Input::Update()
    {
        if (!window)
            return;

        std::memcpy(previousKeys, currentKeys, sizeof(currentKeys));
        for (int key = 0; key < kMaxKeys; ++key)
            currentKeys[key] = (glfwGetKey(window, key) == GLFW_PRESS);

        for (int button = 0; button < kMaxMouseButtons; ++button)
            currentMouseButtons[button] = (glfwGetMouseButton(window, button) == GLFW_PRESS);

        glfwGetCursorPos(window, &mouseX, &mouseY);

        if (firstUpdate)
        {
            // First frame after attaching (or re-baselined by a capture
            // change) - report zero movement rather than a jump from
            // whatever lastMouseX/Y happened to be.
            lastMouseX = mouseX;
            lastMouseY = mouseY;
            firstUpdate = false;
        }

        mouseDeltaX = mouseX - lastMouseX;
        mouseDeltaY = mouseY - lastMouseY;
        lastMouseX = mouseX;
        lastMouseY = mouseY;
    }

    bool Input::IsKeyDown(int glfwKeyCode) const
    {
        return glfwKeyCode >= 0 && glfwKeyCode < kMaxKeys && currentKeys[glfwKeyCode];
    }

    bool Input::WasKeyPressed(int glfwKeyCode) const
    {
        if (glfwKeyCode < 0 || glfwKeyCode >= kMaxKeys)
            return false;
        return currentKeys[glfwKeyCode] && !previousKeys[glfwKeyCode];
    }

    bool Input::IsMouseButtonDown(int glfwMouseButton) const
    {
        return glfwMouseButton >= 0 && glfwMouseButton < kMaxMouseButtons && currentMouseButtons[glfwMouseButton];
    }

    double Input::MouseX() const { return mouseX; }
    double Input::MouseY() const { return mouseY; }
    double Input::MouseDeltaX() const { return mouseDeltaX; }
    double Input::MouseDeltaY() const { return mouseDeltaY; }

    void Input::SetCursorCaptured(bool captured)
    {
        if (!window || captured == cursorCaptured)
            return;

        cursorCaptured = captured;
        glfwSetInputMode(window, GLFW_CURSOR, captured ? GLFW_CURSOR_DISABLED : GLFW_CURSOR_NORMAL);

        // Re-baseline immediately rather than waiting for the next
        // Update(): without this, the frame capture begins on would still
        // compute a delta from the pre-capture cursor position.
        glfwGetCursorPos(window, &lastMouseX, &lastMouseY);
        mouseDeltaX = 0.0;
        mouseDeltaY = 0.0;
    }

    bool Input::IsCursorCaptured() const { return cursorCaptured; }
}
