// imgui_main.cpp
#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"
#include <GLFW/glfw3.h>
#include <fstream>
#include <ctime>
#include <string>

static bool show_text_input = false;
static char input_text[512] = "";
static bool input_focused = false;
static GLFWwindow* g_window = nullptr;

// 날짜 문자열 (예: 251022)
std::string getDateString() {
    time_t t = time(nullptr);
    struct tm* tm_info = localtime(&t);
    char buffer[16];
    strftime(buffer, sizeof(buffer), "%y%m%d", tm_info);
    return std::string(buffer);
}

// 시간 문자열 (예: 14:23:45)
std::string getTimeString() {
    time_t t = time(nullptr);
    struct tm* tm_info = localtime(&t);
    char buffer[16];
    strftime(buffer, sizeof(buffer), "%H:%M:%S", tm_info);
    return std::string(buffer);
}

// CSV 저장
void saveToCSV(const std::string& text)
{
    std::string filename = getDateString() + "_LMG_comment.csv";
    std::ofstream file(filename, std::ios::app);
    if (file.is_open()) {
        file << getTimeString() << "," << text << "\n";
        file.close();
    }
}

// ImGui 초기화
void ImGui_Init(GLFWwindow* window)
{
    g_window = window;
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO();

    // 한글 폰트 로드
    io.Fonts->AddFontFromFileTTF("/usr/share/fonts/truetype/nanum/NanumGothic.ttf", 18.0f, nullptr, io.Fonts->GetGlyphRangesKorean());

    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL3_Init("#version 150");
}

// 입력창 띄우기
void ImGui_ShowTextInputWindow()
{
    show_text_input = true;
    input_text[0] = '\0';
    input_focused = false;
}

// 입력창 활성 여부 반환
bool ImGui_IsTextInputActive()
{
    return show_text_input;
}

// 렌더링
void ImGui_Render()
{
    ImGui_ImplOpenGL3_NewFrame();
    ImGui_ImplGlfw_NewFrame();
    ImGui::NewFrame();

    if (show_text_input)
    {
        ImGui::Begin("Comment Input", &show_text_input, ImGuiWindowFlags_AlwaysAutoResize);

        ImGui::Text("Type your comment (Korean supported)");
        ImGui::Separator();

        // 자동 포커스
        ImGui::SetKeyboardFocusHere();
        ImGui::InputTextMultiline("##input", input_text, IM_ARRAYSIZE(input_text),
                                  ImVec2(400, 100),
                                  ImGuiInputTextFlags_EnterReturnsTrue |
                                  ImGuiInputTextFlags_CallbackAlways);

        if (!input_focused) {
            ImGui::SetItemDefaultFocus();
            input_focused = true;
        }

        // Save / Cancel 버튼
        if (ImGui::Button("Save") || 
            ImGui::IsKeyDown(ImGuiKey_LeftCtrl) && ImGui::IsKeyPressed(ImGuiKey_S, false) ||
            ImGui::IsKeyPressed(ImGuiKey_Enter, false) || 
            ImGui::IsKeyPressed(ImGuiKey_KeypadEnter, false))
        {
            // 한글 입력 불완성 방지: 프레임 끝까지 강제 갱신
            ImGui::Render();
            ImGui::EndFrame();

            saveToCSV(input_text);
            show_text_input = false;
        }

        ImGui::SameLine();
        if (ImGui::Button("Cancel") || ImGui::IsKeyPressed(ImGuiKey_Escape, false))
        {
            show_text_input = false;
        }

        ImGui::End();
    }

    ImGui::Render();
    ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
}

// 종료
void ImGui_Shutdown()
{
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();
}