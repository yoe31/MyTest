#include <GLFW/glfw3.h>
#include <imgui.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl3.h>
#include <iostream>
#include <fstream>
#include <ctime>

bool show_input_window = false;
char input_buffer[256] = {0};

std::string getTodayCSV() {
    time_t t = time(nullptr);
    struct tm *now = localtime(&t);
    char filename[64];
    strftime(filename, sizeof(filename), "%Y%m%d.csv", now);
    return std::string(filename);
}

void saveToCSV(const std::string &text) {
    std::string filename = getTodayCSV();
    std::ofstream file(filename, std::ios::app);
    if (file.is_open()) {
        file << text << "\n";
        file.close();
    } else {
        std::cerr << "파일을 열 수 없습니다: " << filename << std::endl;
    }
}

void key_callback(GLFWwindow *window, int key, int scancode, int action, int mods) {
    if (action == GLFW_PRESS && key == GLFW_KEY_C && (mods & GLFW_MOD_CONTROL)) {
        show_input_window = true;
    }
}

int main() {
    if (!glfwInit()) return -1;

    GLFWwindow *window = glfwCreateWindow(800, 600, "OpenGL CSV Input Example", NULL, NULL);
    if (!window) {
        glfwTerminate();
        return -1;
    }
    glfwMakeContextCurrent(window);
    glfwSetKeyCallback(window, key_callback);

    // ImGui 초기화
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO &io = ImGui::GetIO();
    (void)io;

    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL3_Init("#version 130");

    while (!glfwWindowShouldClose(window)) {
        glfwPollEvents();

        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        // 입력 창 표시
        if (show_input_window) {
            ImGui::Begin("문자열 입력", &show_input_window);
            ImGui::InputText("입력", input_buffer, IM_ARRAYSIZE(input_buffer));

            if (ImGui::Button("Save")) {
                saveToCSV(input_buffer);
                input_buffer[0] = '\0'; // 초기화
                show_input_window = false;
            }
            ImGui::SameLine();
            if (ImGui::Button("Cancel")) {
                show_input_window = false;
            }
            ImGui::End();
        }

        // 화면 그리기
        glClear(GL_COLOR_BUFFER_BIT);
        ImGui::Render();
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

        glfwSwapBuffers(window);
    }

    // 정리
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();
    glfwDestroyWindow(window);
    glfwTerminate();

    return 0;
}