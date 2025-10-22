#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"
#include <stdio.h>
#include <GLFW/glfw3.h> // Include glfw3.h after our OpenGL definitions
#include <iostream>

// UTF-8 한글 폰트 로드용 경로 설정
#define FONT_PATH "NanumGothic.ttf"

// 텍스트 버퍼 크기
static char inputText[256] = "";

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

int main(int, char**)
{
    // GLFW 초기화
    if (!glfwInit())
        return 1;

    // OpenGL 버전 설정
    const char* glsl_version = "#version 130";
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 0);

    // 윈도우 생성
    GLFWwindow* window = glfwCreateWindow(600, 200, "ImGui CSV Saver (한글 지원)", NULL, NULL);
    if (window == NULL)
        return 1;
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1); // Enable vsync

    // ImGui 초기화
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO(); (void)io;

    // 한글 폰트 추가 (UTF-8)
    io.Fonts->AddFontFromFileTTF(FONT_PATH, 18.0f, NULL, io.Fonts->GetGlyphRangesKorean());

    ImGui::StyleColorsDark();
    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL3_Init(glsl_version);

    // 메인 루프
    while (!glfwWindowShouldClose(window))
    {
        glfwPollEvents();

        // 새 프레임 시작
        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        ImGui::Begin("텍스트 입력 및 저장");

        ImGui::Text("CSV에 저장할 텍스트를 입력하세요:");
        ImGui::InputText("##input", inputText, IM_ARRAYSIZE(inputText));

        // Save 버튼
        if (ImGui::Button("Save") || (ImGui::IsKeyDown(ImGuiKey_LeftCtrl) && ImGui::IsKeyPressed(ImGuiKey_S)))
        {
            SaveTextToCSV(inputText, "output.csv");
            inputText[0] = '\0'; // 입력창 초기화
        }

        ImGui::SameLine();
        if (ImGui::Button("Cancel"))
        {
            inputText[0] = '\0';
        }

        ImGui::End();

        // 렌더링
        ImGui::Render();
        int display_w, display_h;
        glfwGetFramebufferSize(window, &display_w, &display_h);
        glViewport(0, 0, display_w, display_h);
        glClearColor(0.1f, 0.1f, 0.1f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT);
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

        glfwSwapBuffers(window);
    }

    // 종료 처리
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();
    glfwDestroyWindow(window);
    glfwTerminate();

    return 0;
}