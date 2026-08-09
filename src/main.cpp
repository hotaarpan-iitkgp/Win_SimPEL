#include <windows.h>
#include <GL/gl.h>
#include "imgui.h"
#include "imgui_internal.h"
#include "imgui_impl_win32.h"
#include "imgui_impl_opengl3.h"
#include "implot.h"
#include "ui/MainWindow.hpp"
#include <iostream>
#include <fstream>
#include <exception>
#include <csignal>
#include <chrono>
#include <ctime>

static void logCrash(const std::string& errorMsg) {
    std::string fullMsg = "\n========================================================\n";
    fullMsg += "CRITICAL ERROR / APPLICATION CRASH DETECTED:\n";
    fullMsg += errorMsg + "\n";
    fullMsg += "========================================================\n";
    
    // 1. Output to PowerShell / Console
    std::cerr << fullMsg << std::endl;
    std::cout << fullMsg << std::endl;
    std::fflush(stderr);
    std::fflush(stdout);

    // 2. Write to crash_log.txt in execution directory
    try {
        std::ofstream logFile("crash_log.txt", std::ios::app);
        if (logFile.is_open()) {
            auto now = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
            logFile << "[" << std::ctime(&now) << "] " << fullMsg << "\n";
            logFile.close();
        }
    } catch (...) {}
}

static LONG WINAPI customUnhandledExceptionFilter(EXCEPTION_POINTERS* pExceptionInfo) {
    char buf[512];
    DWORD code = pExceptionInfo ? pExceptionInfo->ExceptionRecord->ExceptionCode : 0;
    void* addr = pExceptionInfo ? pExceptionInfo->ExceptionRecord->ExceptionAddress : nullptr;
    snprintf(buf, sizeof(buf), "Unhandled Win32 SEH Exception: 0x%08X at Address 0x%p", (unsigned int)code, addr);
    logCrash(buf);
    MessageBoxA(NULL, buf, "CircuitSim Pro - Fatal Crash Handler", MB_ICONERROR | MB_OK);
    return EXCEPTION_EXECUTE_HANDLER;
}

static void customTerminateHandler() {
    logCrash("Unhandled C++ std::exception / terminate() called.");
    std::abort();
}

static void customSignalHandler(int sig) {
    char buf[256];
    snprintf(buf, sizeof(buf), "Fatal Signal Received: %d", sig);
    logCrash(buf);
    std::exit(sig);
}

// Forward declare Win32 message handler
extern IMGUI_IMPL_API LRESULT ImGui_ImplWin32_WndProcHandler(HWND hWnd, UINT msg, WPARAM wParam, LPARAM lParam);

LRESULT WINAPI WndProc(HWND hWnd, UINT msg, WPARAM wParam, LPARAM lParam) {
    if (ImGui_ImplWin32_WndProcHandler(hWnd, msg, wParam, lParam))
        return true;

    switch (msg) {
        case WM_SIZE:
            if (wParam != SIZE_MINIMIZED) {
                // Handle resize
            }
            return 0;
        case WM_SYSCOMMAND:
            if ((wParam & 0xfff0) == SC_KEYMENU) // Disable ALT application menu
                return 0;
            break;
        case WM_DESTROY:
            PostQuitMessage(0);
            return 0;
    }
    return DefWindowProc(hWnd, msg, wParam, lParam);
}

static HICON createCircuitSimIcon(int size) {
    std::vector<DWORD> pixels(size * size);
    float center = size / 2.0f;
    float radius = size * 0.44f;

    for (int y = 0; y < size; ++y) {
        for (int x = 0; x < size; ++x) {
            float dx = x - center;
            float dy = y - center;
            float dist = std::sqrt(dx * dx + dy * dy);

            if (dist <= radius) {
                if (dist >= radius - 2.5f) {
                    pixels[y * size + x] = 0xFFF8BD38; // Cyan/Gold border (BGRA)
                } else {
                    pixels[y * size + x] = 0xFF2A170F; // Dark navy background
                }
            } else {
                pixels[y * size + x] = 0x00000000; // Transparent
            }
        }
    }

    int midX = size / 2;
    int upperY = (int)(center - size * 0.18f);
    int lowerY = (int)(center + size * 0.22f);
    int arm = std::max(2, size / 7);

    // Draw '+' symbol
    for (int d = -arm; d <= arm; ++d) {
        if (upperY >= 0 && upperY < size && midX + d >= 0 && midX + d < size)
            pixels[upperY * size + (midX + d)] = 0xFF00E6FF;
        if (upperY + d >= 0 && upperY + d < size && midX >= 0 && midX < size)
            pixels[(upperY + d) * size + midX] = 0xFF00E6FF;
    }

    // Draw '-' symbol
    for (int d = -arm; d <= arm; ++d) {
        if (lowerY >= 0 && lowerY < size && midX + d >= 0 && midX + d < size)
            pixels[lowerY * size + (midX + d)] = 0xFF00E6FF;
    }

    HBITMAP hColor = CreateBitmap(size, size, 1, 32, pixels.data());
    HBITMAP hMask = CreateBitmap(size, size, 1, 1, NULL);

    ICONINFO ii = {0};
    ii.fIcon = TRUE;
    ii.hbmColor = hColor;
    ii.hbmMask = hMask;

    HICON hIcon = CreateIconIndirect(&ii);
    DeleteObject(hColor);
    DeleteObject(hMask);
    return hIcon;
}

int main(int argc, char** argv) {
    SetUnhandledExceptionFilter(customUnhandledExceptionFilter);
    std::set_terminate(customTerminateHandler);
    std::signal(SIGSEGV, customSignalHandler);
    std::signal(SIGABRT, customSignalHandler);
    std::signal(SIGFPE, customSignalHandler);
    std::signal(SIGILL, customSignalHandler);

    HICON hIconBig = createCircuitSimIcon(32);
    HICON hIconSmall = createCircuitSimIcon(16);

    // Register Win32 Window Class
    WNDCLASSEXW wc = { sizeof(WNDCLASSEXW), CS_OWNDC, WndProc, 0L, 0L, GetModuleHandle(NULL), hIconBig, NULL, NULL, NULL, L"CircuitSimProWinClass", hIconSmall };
    RegisterClassExW(&wc);
    HWND hwnd = CreateWindowW(wc.lpszClassName, L"CircuitSim Pro - Native High Performance C++ Windows Desktop Tool", WS_OVERLAPPEDWINDOW, 100, 100, 1400, 900, NULL, NULL, wc.hInstance, NULL);

    SendMessage(hwnd, WM_SETICON, ICON_BIG, (LPARAM)hIconBig);
    SendMessage(hwnd, WM_SETICON, ICON_SMALL, (LPARAM)hIconSmall);

    // Initialize OpenGL Context
    PIXELFORMATDESCRIPTOR pfd = {
        sizeof(PIXELFORMATDESCRIPTOR), 1, PFD_DRAW_TO_WINDOW | PFD_SUPPORT_OPENGL | PFD_DOUBLEBUFFER,
        PFD_TYPE_RGBA, 32, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 24, 8, 0, PFD_MAIN_PLANE, 0, 0, 0, 0
    };
    HDC hdc = GetDC(hwnd);
    int pf = ChoosePixelFormat(hdc, &pfd);
    SetPixelFormat(hdc, pf, &pfd);
    HGLRC hglrc = wglCreateContext(hdc);
    wglMakeCurrent(hdc, hglrc);

    ShowWindow(hwnd, SW_SHOWDEFAULT);
    UpdateWindow(hwnd);

    // Setup Dear ImGui context
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImPlot::CreateContext();

    ImGuiIO& io = ImGui::GetIO(); (void)io;
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;
    io.ConfigFlags |= ImGuiConfigFlags_DockingEnable;
    io.ConfigFlags |= ImGuiConfigFlags_ViewportsEnable;
    io.ConfigViewportsNoDecoration = false;
    io.ConfigViewportsNoTaskBarIcon = false;

    // Load Crisp High-Quality TrueType Vector Font (Segoe UI)
    if (GetFileAttributesA("C:\\Windows\\Fonts\\segoeui.ttf") != INVALID_FILE_ATTRIBUTES) {
        io.Fonts->AddFontFromFileTTF("C:\\Windows\\Fonts\\segoeui.ttf", 16.0f);
    } else if (GetFileAttributesA("C:\\Windows\\Fonts\\arial.ttf") != INVALID_FILE_ATTRIBUTES) {
        io.Fonts->AddFontFromFileTTF("C:\\Windows\\Fonts\\arial.ttf", 15.0f);
    } else {
        io.Fonts->AddFontDefault();
    }

    ImGui::StyleColorsDark();

    // When viewports are enabled, tweak WindowRounding/WindowBg so platform windows look consistent
    ImGuiStyle& style = ImGui::GetStyle();
    if (io.ConfigFlags & ImGuiConfigFlags_ViewportsEnable) {
        style.WindowRounding = 0.0f;
        style.Colors[ImGuiCol_WindowBg].w = 1.0f;
    }

    // Global HGLRC context reference for multi-viewport rendering
    static HGLRC s_hglrc = hglrc;
    static HDC s_mainHdc = hdc;

    // Initialize ImGui Win32 & OpenGL3 Backends
    ImGui_ImplWin32_Init(hwnd);
    ImGui_ImplOpenGL3_Init("#version 130");

    // Setup platform renderer callback for secondary viewport OS windows
    if (io.ConfigFlags & ImGuiConfigFlags_ViewportsEnable) {
        ImGuiPlatformIO& platform_io = ImGui::GetPlatformIO();
        platform_io.Renderer_RenderWindow = [](ImGuiViewport* viewport, void*) {
            HWND hwnd = (HWND)viewport->PlatformHandle;
            if (!hwnd) return;
            HDC hdc = ::GetDC(hwnd);
            if (hdc) {
                PIXELFORMATDESCRIPTOR pfd = {
                    sizeof(PIXELFORMATDESCRIPTOR), 1, PFD_DRAW_TO_WINDOW | PFD_SUPPORT_OPENGL | PFD_DOUBLEBUFFER,
                    PFD_TYPE_RGBA, 32, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 24, 8, 0, PFD_MAIN_PLANE, 0, 0, 0, 0
                };
                int pf = ::ChoosePixelFormat(hdc, &pfd);
                if (pf) {
                    ::SetPixelFormat(hdc, pf, &pfd);
                }
                ::wglMakeCurrent(hdc, s_hglrc);
                ::glViewport(0, 0, (GLsizei)viewport->Size.x, (GLsizei)viewport->Size.y);
                ::glClearColor(0.12f, 0.12f, 0.14f, 1.0f);
                ::glClear(GL_COLOR_BUFFER_BIT);
                if (viewport->DrawData) {
                    ImGui_ImplOpenGL3_RenderDrawData(viewport->DrawData);
                }
                ::SwapBuffers(hdc);
                ::ReleaseDC(hwnd, hdc);
            }
        };
    }

    CircuitSim::MainWindow mainWindow;

    // Main render loop
    bool done = false;
    while (!done) {
        MSG msg;
        while (PeekMessage(&msg, NULL, 0U, 0U, PM_REMOVE)) {
            TranslateMessage(&msg);
            DispatchMessage(&msg);
            if (msg.message == WM_QUIT) done = true;
        }
        if (done) break;

        // Start Dear ImGui Frame
        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplWin32_NewFrame();
        ImGui::NewFrame();

        // Enable full workspace docking window
        ImGuiDockNodeFlags dockspace_flags = ImGuiDockNodeFlags_None;
        ImGuiWindowFlags window_flags = ImGuiWindowFlags_MenuBar | ImGuiWindowFlags_NoDocking;
        ImGuiViewport* viewport = ImGui::GetMainViewport();
        ImGui::SetNextWindowPos(viewport->WorkPos);
        ImGui::SetNextWindowSize(viewport->WorkSize);
        ImGui::SetNextWindowViewport(viewport->ID);
        ImGui::PushStyleVar(ImGuiStyleVar_WindowRounding, 0.0f);
        ImGui::PushStyleVar(ImGuiStyleVar_WindowBorderSize, 0.0f);
        window_flags |= ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove;
        window_flags |= ImGuiWindowFlags_NoBringToFrontOnFocus | ImGuiWindowFlags_NoNavFocus;

        ImGui::Begin("MainDockSpace", nullptr, window_flags);
        ImGui::PopStyleVar(2);

        ImGuiID dockspace_id = ImGui::GetID("MyDockSpace");
        ImGui::DockSpace(dockspace_id, ImVec2(0.0f, 0.0f), dockspace_flags);

        static bool first_layout = true;
        if (first_layout) {
            first_layout = false;
            ImGui::DockBuilderRemoveNode(dockspace_id);
            ImGui::DockBuilderAddNode(dockspace_id, ImGuiDockNodeFlags_DockSpace);
            ImGui::DockBuilderSetNodeSize(dockspace_id, viewport->WorkSize);

            ImGuiID dock_main_id = dockspace_id;
            ImGuiID dock_left_id = ImGui::DockBuilderSplitNode(dock_main_id, ImGuiDir_Left, 0.20f, nullptr, &dock_main_id);
            ImGuiID dock_right_id = ImGui::DockBuilderSplitNode(dock_main_id, ImGuiDir_Right, 0.22f, nullptr, &dock_main_id);
            ImGuiID dock_top_id = ImGui::DockBuilderSplitNode(dock_main_id, ImGuiDir_Up, 0.08f, nullptr, &dock_main_id);
            ImGuiID dock_bottom_id = ImGui::DockBuilderSplitNode(dock_main_id, ImGuiDir_Down, 0.35f, nullptr, &dock_main_id);

            ImGui::DockBuilderDockWindow("Component Pane", dock_left_id);
            ImGui::DockBuilderDockWindow("Simulation Control", dock_top_id);
            ImGui::DockBuilderDockWindow("Property Inspector", dock_right_id);
            ImGui::DockBuilderDockWindow("Schematic Editor Canvas", dock_main_id);
            ImGui::DockBuilderDockWindow("Real-Time Oscilloscope Waveforms", dock_bottom_id);

            ImGui::DockBuilderFinish(dockspace_id);
        }

        try {
            mainWindow.render();
        } catch (const std::exception& e) {
            logCrash(std::string("Exception caught in main render loop: ") + e.what());
        } catch (...) {
            logCrash("Unknown exception caught in main render loop.");
        }

        ImGui::End();

        // Rendering
        ImGui::Render();
        RECT clientRect;
        GetClientRect(hwnd, &clientRect);
        int fbW = clientRect.right - clientRect.left;
        int fbH = clientRect.bottom - clientRect.top;
        glViewport(0, 0, fbW, fbH);
        glClearColor(0.96f, 0.95f, 0.81f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT);
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

        // Multi-viewport: update and render additional platform windows
        if (io.ConfigFlags & ImGuiConfigFlags_ViewportsEnable) {
            ImGui::UpdatePlatformWindows();
            ImGui::RenderPlatformWindowsDefault();
            wglMakeCurrent(hdc, hglrc); // Restore main context after rendering sub-viewports
        }

        SwapBuffers(hdc);
    }

    // Cleanup
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplWin32_Shutdown();
    ImPlot::DestroyContext();
    ImGui::DestroyContext();

    wglMakeCurrent(NULL, NULL);
    wglDeleteContext(hglrc);
    ReleaseDC(hwnd, hdc);
    DestroyWindow(hwnd);
    UnregisterClassW(wc.lpszClassName, wc.hInstance);

    return 0;
}
