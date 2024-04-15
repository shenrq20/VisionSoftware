/**
 * @file log.h
 * @author Shibo LIU (shibo.liu@saiwider.com)
 * @brief 
 * @version 0.1
 * @date 2024-01-19
 * 
 * @copyright Copyright (c) 2024
 * 
 */
#pragma once

#define     INFO        1
#define     WARNING     2
#define     ERROR       3

#include <array>
#include <imgui.h>
#include <string>

class AppLog {
public:
    ImGuiTextBuffer     Buf;
    ImGuiTextFilter     Filter;
    ImVector<int>       LineOffsets;
    bool                AutoScroll;
    std::array<std::string, 3>  LogInfoLevel;

    AppLog();

    /**
     * @brief Clear infromation inside the buffer
     * 
     */
    void Clear();

    /**
     * @brief Add log to the bugger. log.AddLog("")
     * 
     */
    void AddLog(const char* fmt, ...) IM_FMTARGS(2);
    void Draw(const char* title, bool* p_open = NULL);
};
