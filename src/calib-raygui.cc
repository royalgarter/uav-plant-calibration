#ifdef WINGUI

#include "raylib.h"

// Resolve name conflicts between raylib and windows.h
#define Rectangle _WinRectangle
#define CloseWindow _WinCloseWindow
#define ShowCursor _WinShowCursor
#include <windows.h>
#include <shlobj.h>
#include <commctrl.h>
#undef Rectangle
#undef CloseWindow
#undef ShowCursor

#define RAYGUI_IMPLEMENTATION
#include "raygui.h"

#include <string>
#include <vector>
#include <cstring>

using namespace std;

// --- Win32 Helpers for File/Folder Dialogs ---
static string fromWide(const wstring& s) {
    if (s.empty()) return "";
    int size = WideCharToMultiByte(CP_UTF8, 0, s.c_str(), -1, nullptr, 0, nullptr, nullptr);
    string result(size - 1, 0);
    WideCharToMultiByte(CP_UTF8, 0, s.c_str(), -1, &result[0], size, nullptr, nullptr);
    return result;
}

static wstring toWide(const string& s) {
    if (s.empty()) return L"";
    int size = MultiByteToWideChar(CP_UTF8, 0, s.c_str(), -1, nullptr, 0);
    wstring result(size - 1, 0);
    MultiByteToWideChar(CP_UTF8, 0, s.c_str(), -1, &result[0], size);
    return result;
}

static string browseForFolder(const string& title) {
    BROWSEINFOW bi = {};
    bi.lpszTitle = L"Select Folder";
    bi.ulFlags = BIF_RETURNONLYFSDIRS | BIF_NEWDIALOGSTYLE;
    LPITEMIDLIST pidl = SHBrowseForFolderW(&bi);
    if (pidl != nullptr) {
        wchar_t path[MAX_PATH];
        if (SHGetPathFromIDListW(pidl, path)) {
            CoTaskMemFree(pidl);
            return fromWide(path);
        }
        CoTaskMemFree(pidl);
    }
    return "";
}

static string browseForFile(const string& title, const string& filter) {
    OPENFILENAMEW ofn = {};
    wchar_t szFile[MAX_PATH] = L"";
    ofn.lStructSize = sizeof(ofn);
    ofn.lpstrFile = szFile;
    ofn.nMaxFile = MAX_PATH;
    ofn.lpstrFilter = L"All Files\0*.*\0CSV Files\0*.csv\0Images\0*.jpg;*.jpeg;*.png;*.bmp\0";
    ofn.nFilterIndex = 1;
    ofn.Flags = OFN_PATHMUSTEXIST | OFN_FILEMUSTEXIST;
    if (GetOpenFileNameW(&ofn)) return fromWide(szFile);
    return "";
}

// --- Main GUI Entry Point ---
bool runCalibGui(string& inputFolder, string& outputFolder, string& radioRefFile,
                 bool& enableRadio, bool& twoPointClickMode, bool& autoDetectBoard, int& boardThickness,
                 string& radioTemplatePath) {
    
    // Initialize COM and Common Controls for file dialogs
    CoInitialize(nullptr);
    INITCOMMONCONTROLSEX icex;
    icex.dwSize = sizeof(icex);
    icex.dwICC = ICC_STANDARD_CLASSES;
    InitCommonControlsEx(&icex);

    // Initialize state from existing values
    char bufInput[512]; strncpy(bufInput, inputFolder.c_str(), 511); bufInput[511] = '\0';
    char bufOutput[512]; strncpy(bufOutput, outputFolder.c_str(), 511); bufOutput[511] = '\0';
    char bufRadioRef[512]; strncpy(bufRadioRef, radioRefFile.c_str(), 511); bufRadioRef[511] = '\0';
    char bufTemplate[512]; strncpy(bufTemplate, radioTemplatePath.c_str(), 511); bufTemplate[511] = '\0';
    
    bool submitted = false;
    bool cancelled = false;

    // Use a smaller window size for the UI
    const int screenWidth = 720;
    const int screenHeight = 580;

    // Set config for better UI look
    InitWindow(screenWidth, screenHeight, "UAV Plant Calibration");
    SetTargetFPS(60);

    bool editInput = false;
    bool editOutput = false;
    bool editRadioRef = false;
    bool editTemplate = false;
    bool editThickness = false;

    while (!WindowShouldClose()) {
        if (submitted || cancelled) break;

        BeginDrawing();
        ClearBackground(GetColor(GuiGetStyle(DEFAULT, BACKGROUND_COLOR)));

        // Draw GUI
        if (GuiWindowBox((Rectangle){ 0, 0, (float)screenWidth, (float)screenHeight }, "UAV Plant Calibration Parameters")) {
            cancelled = true;
        }

        float x = 20;
        float y = 50;
        float labelW = 140;
        float editW = 420;
        float btnW = 100;
        float h = 30;
        float gap = 45;

        // Input Folder
        GuiLabel((Rectangle){ x, y, labelW, h }, "Input Folder:");
        if (GuiTextBox((Rectangle){ x + labelW, y, editW, h }, bufInput, 512, editInput)) editInput = !editInput;
        if (GuiButton((Rectangle){ x + labelW + editW + 10, y, btnW, h }, "Browse...")) {
            string path = browseForFolder("Select Input Folder");
            if (!path.empty()) strncpy(bufInput, path.c_str(), 511);
        }
        y += gap;

        // Output Folder
        GuiLabel((Rectangle){ x, y, labelW, h }, "Output Folder:");
        if (GuiTextBox((Rectangle){ x + labelW, y, editW, h }, bufOutput, 512, editOutput)) editOutput = !editOutput;
        if (GuiButton((Rectangle){ x + labelW + editW + 10, y, btnW, h }, "Browse...")) {
            string path = browseForFolder("Select Output Folder");
            if (!path.empty()) strncpy(bufOutput, path.c_str(), 511);
        }
        y += gap;

        // Radiometric Ref File
        GuiLabel((Rectangle){ x, y, labelW, h }, "Radiometric Ref:");
        if (GuiTextBox((Rectangle){ x + labelW, y, editW, h }, bufRadioRef, 512, editRadioRef)) editRadioRef = !editRadioRef;
        if (GuiButton((Rectangle){ x + labelW + editW + 10, y, btnW, h }, "Browse...")) {
            string path = browseForFile("Select Radiometric Reference CSV", "*.csv");
            if (!path.empty()) strncpy(bufRadioRef, path.c_str(), 511);
        }
        y += gap;

        // Template File
        GuiLabel((Rectangle){ x, y, labelW, h }, "Board Template:");
        if (GuiTextBox((Rectangle){ x + labelW, y, editW, h }, bufTemplate, 512, editTemplate)) editTemplate = !editTemplate;
        if (GuiButton((Rectangle){ x + labelW + editW + 10, y, btnW, h }, "Browse...")) {
            string path = browseForFile("Select Board Template Image", "*.jpg;*.png");
            if (!path.empty()) strncpy(bufTemplate, path.c_str(), 511);
        }
        y += gap;

        // Checkboxes
        GuiCheckBox((Rectangle){ x, y, 20, 20 }, "Enable Radiometric Calibration (--radio)", &enableRadio);
        y += gap - 10;

        bool prevTwoPoint = twoPointClickMode;
        GuiCheckBox((Rectangle){ x, y, 20, 20 }, "2-Point Click Mode for Board Detection", &twoPointClickMode);
        if (twoPointClickMode && !prevTwoPoint) autoDetectBoard = false;
        y += gap - 10;

        bool prevAuto = autoDetectBoard;
        GuiCheckBox((Rectangle){ x, y, 20, 20 }, "Auto-Detect Radiometric Board (--auto)", &autoDetectBoard);
        if (autoDetectBoard && !prevAuto) twoPointClickMode = false;
        y += gap - 10;

        // Thickness
        GuiLabel((Rectangle){ x, y, labelW, h }, "Thickness (px):");
        if (GuiValueBox((Rectangle){ x + labelW, y, 100, h }, NULL, &boardThickness, -1, 1000, editThickness)) editThickness = !editThickness;
        y += gap + 20;

        // Start Button
        if (GuiButton((Rectangle){ x, y, (float)screenWidth - 40, 50 }, "START CALIBRATION")) {
            inputFolder = bufInput;
            outputFolder = bufOutput;
            radioRefFile = bufRadioRef;
            radioTemplatePath = bufTemplate;

            if (inputFolder.empty() || outputFolder.empty()) {
                // Show a simple warning? Raygui doesn't have easy popups without blocking.
                // For now, just print and don't submit if empty
            } else {
                submitted = true;
            }
        }

        EndDrawing();
    }

    CloseWindow();
    CoUninitialize();
    return submitted;
}

#endif
