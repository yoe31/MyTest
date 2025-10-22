#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"
#include <stdio.h>
#include <GLFW/glfw3.h>
#include <iostream>

#define FONT_PATH "NanumGothic.ttf"
static char inputText[256] = "";
static bool showInputPopup = false;

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

    GLFWwindow* window = glfwCreateWindow(800, 600, "Main GL Window", NULL, NULL);
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

    while (!glfwWindowShouldClose(window))
    {
        glfwPollEvents();

        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        // 메인 윈도우 내용 (계속 돌아감)
        ImGui::Begin("메인 윈도우");
        ImGui::Text("이 창은 계속 돌아갑니다.");
        if (ImGui::Button("텍스트 입력창 열기"))
        {
            showInputPopup = true;
            ImGui::OpenPopup("텍스트 입력");
        }
        ImGui::End();

        // 팝업 윈도우 (모달)
        if (showInputPopup)
        {
            ImGui::SetNextWindowSize(ImVec2(400, 150));
            if (ImGui::BeginPopupModal("텍스트 입력", NULL, ImGuiWindowFlags_AlwaysAutoResize))
            {
                ImGui::Text("CSV에 저장할 텍스트를 입력하세요:");
                ImGui::InputText("##input", inputText, IM_ARRAYSIZE(inputText), ImGuiInputTextFlags_AutoSelectAll);

                if (ImGui::Button("Save"))
                {
                    SaveTextToCSV(inputText, "output.csv");
                    inputText[0] = '\0';
                    showInputPopup = false;
                    ImGui::CloseCurrentPopup();
                }

                ImGui::SameLine();
                if (ImGui::Button("Cancel"))
                {
                    inputText[0] = '\0';
                    showInputPopup = false;
                    ImGui::CloseCurrentPopup();
                }

                ImGui::EndPopup();
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