#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"
#include <stdio.h>
#include <GLFW/glfw3.h>
#include <iostream>

#define FONT_PATH "NanumGothic.ttf"
static char inputText[256] = "";
static bool showInputPopup = true;  // 시작 시 자동으로 팝업 표시
static bool needFocus = true;

void SaveTextToCSV(const char* text, const char* filename)
{
    FILE* file = fopen(filename, "a");
    if (file)
    {
        fprintf(file, "\"%s\"\n", text);
        fclose(file);
    }
    else
    {
        std::cerr << "파일 열기 실패: " << filename << std::endl;
    }
}

int main()
{
    if (!glfwInit()) return 1;
    const char* glsl_version = "#version 130";
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 0);

    GLFWwindow* window = glfwCreateWindow(800, 600, "ImGui CSV Saver (한글 자동 입력창)", NULL, NULL);
    if (!window) return 1;
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);

    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO(); (void)io;

    io.Fonts->AddFontFromFileTTF(FONT_PATH, 18.0f, NULL, io.Fonts->GetGlyphRangesKorean());
    ImGui::StyleColorsDark();

    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL3_Init(glsl_version);

    // 팝업을 미리 열기
    ImGui::OpenPopup("텍스트 입력");

    while (!glfwWindowShouldClose(window))
    {
        glfwPollEvents();

        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        // 메인 윈도우 (계속 유지됨)
        ImGui::Begin("메인 윈도우");
        ImGui::Text("이 창은 계속 동작합니다.");
        ImGui::Text("텍스트 입력창은 자동으로 표시됩니다.");
        ImGui::End();

        // 팝업 윈도우 (자동 표시)
        if (showInputPopup)
        {
            ImGui::SetNextWindowSize(ImVec2(400, 150));
            if (ImGui::BeginPopupModal("텍스트 입력", NULL, ImGuiWindowFlags_AlwaysAutoResize))
            {
                ImGui::Text("CSV에 저장할 텍스트를 입력하세요:");

                // 처음 열릴 때 자동 포커스
                if (needFocus)
                {
                    ImGui::SetKeyboardFocusHere();
                    needFocus = false;
                }

                ImGui::InputText("##input", inputText, IM_ARRAYSIZE(inputText), ImGuiInputTextFlags_AutoSelectAll);

                // Save 버튼
                if (ImGui::Button("Save"))
                {
                    // 한글 조합 완성 강제
                    glfwFocusWindow(window);
                    glfwPollEvents();

                    SaveTextToCSV(inputText, "output.csv");
                    inputText[0] = '\0';
                    showInputPopup = false;
                    ImGui::CloseCurrentPopup();
                }

                ImGui::SameLine();

                // Cancel 버튼
                if (ImGui::Button("Cancel"))
                {
                    inputText[0] = '\0';
                    showInputPopup = false;
                    ImGui::CloseCurrentPopup();
                }

                ImGui::EndPopup();
            }
            else
            {
                // 팝업이 닫혔으면 프로그램 종료
                glfwSetWindowShouldClose(window, GLFW_TRUE);
            }
        }

        // 렌더링
        ImGui::Render();
        int w, h;
        glfwGetFramebufferSize(window, &w, &h);
        glViewport(0, 0, w, h);
        glClearColor(0.1f, 0.1f, 0.1f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT);
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
        glfwSwapBuffers(window);
    }

    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();
    glfwDestroyWindow(window);
    glfwTerminate();
    return 0;
}