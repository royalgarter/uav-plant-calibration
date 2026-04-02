#ifdef WINGUI

#include <windows.h>
#include <commctrl.h>
#include <string>
#include <iostream>
#include <vector>

using namespace std;

// Link with libraries
#pragma comment(lib, "user32.lib")
#pragma comment(lib, "gdi32.lib")
#pragma comment(lib, "Comctl32.lib")
#pragma comment(lib, "Comdlg32.lib") // For file dialogs

// --- CONTROL IDs ---
#define IDC_INPUT_FOLDER_EDIT     101
#define IDC_OUTPUT_FOLDER_EDIT    102
#define IDC_RADIO_REF_FILE_EDIT   103
#define IDC_RADIO_CHECK           104
#define IDC_TWO_POINT_CLICK_CHECK 105
#define IDC_AUTO_DETECT_CHECK     106
#define IDC_BOARD_THICKNESS_EDIT  107
#define IDC_START_BUTTON          108
#define IDC_BROWSE_INPUT_BTN      109
#define IDC_BROWSE_OUTPUT_BTN     110
#define IDC_BROWSE_RADIO_REF_BTN  111

// --- GLOBAL STATE ---
HWND hInputFolderEdit, hOutputFolderEdit, hRadioRefFileEdit;
HWND hRadioCheck, hTwoPointClickCheck, hAutoDetectCheck, hBoardThicknessEdit;
HWND hBrowseInputBtn, hBrowseOutputBtn, hBrowseRadioRefBtn;

string g_inputFolder, g_outputFolder, g_radioRefFile;
bool g_enableRadio = false;
bool g_twoPointClickMode = false;
bool g_autoDetectBoard = false;
int g_boardThickness = 0;

bool g_guiSubmitted = false;
bool g_guiCancelled = false;

// Forward declaration
LRESULT CALLBACK WndProc(HWND hwnd, UINT message, WPARAM wParam, LPARAM lParam);

// --- FILE DIALOG HELPERS ---
string browseForFolder(HWND hwnd, const string& title) {
	BROWSEINFO bi = {};
	bi.hwndOwner = hwnd;
	bi.lpszTitle = title.c_str();
	bi.ulFlags = BIF_RETURNONLYFSDIRS | BIF_NEWDIALOGSTYLE;

	LPITEMIDLIST pidl = SHBrowseForFolder(&bi);
	if (pidl != nullptr) {
		char path[MAX_PATH];
		if (SHGetPathFromIDList(pidl, path)) {
			CoTaskMemFree(pidl);
			return string(path);
		}
		CoTaskMemFree(pidl);
	}
	return "";
}

string browseForFile(HWND hwnd, const string& title, const string& filter) {
	OPENFILENAME ofn = {};
	char szFile[MAX_PATH] = "";

	ofn.lStructSize = sizeof(OPENFILENAME);
	ofn.hwndOwner = hwnd;
	ofn.lpstrFile = szFile;
	ofn.nMaxFile = MAX_PATH;
	ofn.lpstrFilter = filter.c_str();
	ofn.lpstrTitle = title.c_str();
	ofn.Flags = OFN_FILEMUSTEXIST | OFN_PATHMUSTEXIST;

	if (GetOpenFileName(&ofn)) {
		return string(szFile);
	}
	return "";
}

// --- GUI ENTRY POINT ---
bool runCalibGui(string& inputFolder, string& outputFolder, string& radioRefFile,
                 bool& enableRadio, bool& twoPointClickMode, bool& autoDetectBoard, int& boardThickness) {
	// Initialize common controls
	INITCOMMONCONTROLSEX icex;
	icex.dwSize = sizeof(INITCOMMONCONTROLSEX);
	icex.dwICC = ICC_STANDARD_CLASSES;
	InitCommonControlsEx(&icex);

	const char CLASS_NAME[] = "UavCalibratorWindowClass";

	WNDCLASSEX wc = {};
	wc.cbSize = sizeof(WNDCLASSEX);
	wc.lpfnWndProc = WndProc;
	wc.hInstance = GetModuleHandle(nullptr);
	wc.lpszClassName = CLASS_NAME;
	wc.hbrBackground = (HBRUSH)(COLOR_WINDOW + 1);
	wc.hCursor = LoadCursor(nullptr, IDC_ARROW);
	wc.hIcon = LoadIcon(nullptr, IDI_APPLICATION);

	if (!RegisterClassEx(&wc)) {
		MessageBox(nullptr, "Window Registration Failed!", "Error", MB_ICONEXCLAMATION | MB_OK);
		return false;
	}

	HWND hwnd = CreateWindowEx(
		0,
		CLASS_NAME,
		"UAV Plant Calibration",
		WS_OVERLAPPEDWINDOW & ~WS_THICKFRAME & ~WS_MAXIMIZEBOX,
		CW_USEDEFAULT, CW_USEDEFAULT,
		720, 580,
		nullptr,
		nullptr,
		GetModuleHandle(nullptr),
		nullptr
	);

	if (hwnd == nullptr) {
		MessageBox(nullptr, "Window Creation Failed!", "Error", MB_ICONEXCLAMATION | MB_OK);
		return false;
	}

	ShowWindow(hwnd, SW_SHOWDEFAULT);
	UpdateWindow(hwnd);

	MSG msg = {};
	while (GetMessage(&msg, nullptr, 0, 0)) {
		TranslateMessage(&msg);
		DispatchMessage(&msg);
		if (g_guiSubmitted || g_guiCancelled) {
			break;
		}
	}

	if (g_guiSubmitted) {
		inputFolder = g_inputFolder;
		outputFolder = g_outputFolder;
		radioRefFile = g_radioRefFile;
		enableRadio = g_enableRadio;
		twoPointClickMode = g_twoPointClickMode;
		autoDetectBoard = g_autoDetectBoard;
		boardThickness = g_boardThickness;
	}

	return g_guiSubmitted;
}

// --- WINDOW PROCEDURE ---
LRESULT CALLBACK WndProc(HWND hwnd, UINT message, WPARAM wParam, LPARAM lParam) {
	switch (message) {
		case WM_CREATE: {
			int yPos = 20;
			int xPos = 20;
			int labelWidth = 140;
			int editWidth = 420;
			int btnWidth = 100;
			int height = 24;
			int gap = 45;

			// Title
			CreateWindow("STATIC", "UAV Plant Calibration Parameters", WS_VISIBLE | WS_CHILD | SS_CENTER,
						 xPos, yPos, 660, 30, hwnd, nullptr, GetModuleHandle(nullptr), nullptr);
			yPos += 40;

			// Input Folder
			CreateWindow("STATIC", "Input Folder:", WS_VISIBLE | WS_CHILD,
						 xPos, yPos, labelWidth, height, hwnd, nullptr, GetModuleHandle(nullptr), nullptr);
			hInputFolderEdit = CreateWindow("EDIT", ".input", WS_VISIBLE | WS_CHILD | WS_BORDER | ES_AUTOHSCROLL,
											xPos + labelWidth, yPos, editWidth, height, hwnd, (HMENU)IDC_INPUT_FOLDER_EDIT, GetModuleHandle(nullptr), nullptr);
			hBrowseInputBtn = CreateWindow("BUTTON", "Browse...", WS_VISIBLE | WS_CHILD | BS_PUSHBUTTON,
										   xPos + labelWidth + editWidth + 10, yPos - 2, btnWidth, 28, hwnd, (HMENU)IDC_BROWSE_INPUT_BTN, GetModuleHandle(nullptr), nullptr);
			yPos += gap;

			// Output Folder
			CreateWindow("STATIC", "Output Folder:", WS_VISIBLE | WS_CHILD,
						 xPos, yPos, labelWidth, height, hwnd, nullptr, GetModuleHandle(nullptr), nullptr);
			hOutputFolderEdit = CreateWindow("EDIT", ".output", WS_VISIBLE | WS_CHILD | WS_BORDER | ES_AUTOHSCROLL,
											 xPos + labelWidth, yPos, editWidth, height, hwnd, (HMENU)IDC_OUTPUT_FOLDER_EDIT, GetModuleHandle(nullptr), nullptr);
			hBrowseOutputBtn = CreateWindow("BUTTON", "Browse...", WS_VISIBLE | WS_CHILD | BS_PUSHBUTTON,
											xPos + labelWidth + editWidth + 10, yPos - 2, btnWidth, 28, hwnd, (HMENU)IDC_BROWSE_OUTPUT_BTN, GetModuleHandle(nullptr), nullptr);
			yPos += gap;

			// Radiometric Reference File
			CreateWindow("STATIC", "Radiometric Ref File:", WS_VISIBLE | WS_CHILD,
						 xPos, yPos, labelWidth, height, hwnd, nullptr, GetModuleHandle(nullptr), nullptr);
			hRadioRefFileEdit = CreateWindow("EDIT", "radiometric_reference.csv", WS_VISIBLE | WS_CHILD | WS_BORDER | ES_AUTOHSCROLL,
											 xPos + labelWidth, yPos, editWidth, height, hwnd, (HMENU)IDC_RADIO_REF_FILE_EDIT, GetModuleHandle(nullptr), nullptr);
			hBrowseRadioRefBtn = CreateWindow("BUTTON", "Browse...", WS_VISIBLE | WS_CHILD | BS_PUSHBUTTON,
											  xPos + labelWidth + editWidth + 10, yPos - 2, btnWidth, 28, hwnd, (HMENU)IDC_BROWSE_RADIO_REF_BTN, GetModuleHandle(nullptr), nullptr);
			yPos += gap;

			// Enable Radiometric Calibration Checkbox
			hRadioCheck = CreateWindow("BUTTON", "Enable Radiometric Calibration (--radio)",
									   WS_VISIBLE | WS_CHILD | BS_AUTOCHECKBOX,
									   xPos, yPos, 400, height, hwnd, (HMENU)IDC_RADIO_CHECK, GetModuleHandle(nullptr), nullptr);
			yPos += gap - 10;

			// 2-Point Click Mode Checkbox
			hTwoPointClickCheck = CreateWindow("BUTTON", "2-Point Click Mode for Board Detection",
											   WS_VISIBLE | WS_CHILD | BS_AUTOCHECKBOX,
											   xPos, yPos, 400, height, hwnd, (HMENU)IDC_TWO_POINT_CLICK_CHECK, GetModuleHandle(nullptr), nullptr);
			yPos += gap - 10;

			// Auto-Detect Board Checkbox
			hAutoDetectCheck = CreateWindow("BUTTON", "Auto-Detect Radiometric Board (--auto)",
											WS_VISIBLE | WS_CHILD | BS_AUTOCHECKBOX,
											xPos, yPos, 400, height, hwnd, (HMENU)IDC_AUTO_DETECT_CHECK, GetModuleHandle(nullptr), nullptr);
			yPos += gap - 10;

			// Board Thickness
			CreateWindow("STATIC", "Board Thickness (pixels):", WS_VISIBLE | WS_CHILD,
						 xPos, yPos, labelWidth, height, hwnd, nullptr, GetModuleHandle(nullptr), nullptr);
			hBoardThicknessEdit = CreateWindow("EDIT", "0", WS_VISIBLE | WS_CHILD | WS_BORDER | ES_NUMBER | ES_AUTOHSCROLL,
											   xPos + labelWidth, yPos, 100, height, hwnd, (HMENU)IDC_BOARD_THICKNESS_EDIT, GetModuleHandle(nullptr), nullptr);
			CreateWindow("STATIC", "(0 = auto-detect thickness, -1 = disable auto-detect)", WS_VISIBLE | WS_CHILD | SS_GRAYTEXT,
						 xPos + labelWidth + 110, yPos + 3, 300, height, hwnd, nullptr, GetModuleHandle(nullptr), nullptr);
			yPos += gap + 10;

			// Separator line
			CreateWindow("STATIC", "", WS_VISIBLE | WS_CHILD | SS_ETCHEDHORZ,
						 xPos, yPos, 660, 4, hwnd, nullptr, GetModuleHandle(nullptr), nullptr);
			yPos += 30;

			// Start Button
			CreateWindow("BUTTON", "START CALIBRATION", WS_VISIBLE | WS_CHILD | BS_PUSHBUTTON,
						 xPos, yPos, 660, 45, hwnd, (HMENU)IDC_START_BUTTON, GetModuleHandle(nullptr), nullptr);

			break;
		}

		case WM_COMMAND: {
			switch (LOWORD(wParam)) {
				case IDC_BROWSE_INPUT_BTN: {
					string folder = browseForFolder(hwnd, "Select Input Folder");
					if (!folder.empty()) {
						SetWindowText(hInputFolderEdit, folder.c_str());
					}
					break;
				}

				case IDC_BROWSE_OUTPUT_BTN: {
					string folder = browseForFolder(hwnd, "Select Output Folder");
					if (!folder.empty()) {
						SetWindowText(hOutputFolderEdit, folder.c_str());
					}
					break;
				}

				case IDC_BROWSE_RADIO_REF_BTN: {
					string file = browseForFile(hwnd, "Select Radiometric Reference CSV",
					                            "CSV Files (*.csv)\0*.csv\0All Files (*.*)\0*.*\0");
					if (!file.empty()) {
						SetWindowText(hRadioRefFileEdit, file.c_str());
					}
					break;
				}

				case IDC_START_BUTTON: {
					char buffer[MAX_PATH];

					// Get input folder
					GetWindowText(hInputFolderEdit, buffer, MAX_PATH);
					g_inputFolder = buffer;

					// Get output folder
					GetWindowText(hOutputFolderEdit, buffer, MAX_PATH);
					g_outputFolder = buffer;

					// Get radiometric reference file
					GetWindowText(hRadioRefFileEdit, buffer, MAX_PATH);
					g_radioRefFile = buffer;

					// Get checkbox states
					g_enableRadio = (SendMessage(hRadioCheck, BM_GETCHECK, 0, 0) == BST_CHECKED);
					g_twoPointClickMode = (SendMessage(hTwoPointClickCheck, BM_GETCHECK, 0, 0) == BST_CHECKED);
					g_autoDetectBoard = (SendMessage(hAutoDetectCheck, BM_GETCHECK, 0, 0) == BST_CHECKED);

					// Get board thickness
					GetWindowText(hBoardThicknessEdit, buffer, 10);
					try {
						g_boardThickness = stoi(buffer);
					} catch (const exception& e) {
						MessageBox(hwnd, "Invalid board thickness value. Please enter a valid integer.", "Error", MB_ICONERROR | MB_OK);
						return 0;
					}

					// Validate inputs
					if (g_inputFolder.empty()) {
						MessageBox(hwnd, "Please specify an input folder.", "Validation Error", MB_ICONWARNING | MB_OK);
						return 0;
					}

					if (g_outputFolder.empty()) {
						MessageBox(hwnd, "Please specify an output folder.", "Validation Error", MB_ICONWARNING | MB_OK);
						return 0;
					}

					if (g_enableRadio && g_radioRefFile.empty()) {
						int result = MessageBox(hwnd, "Radiometric reference file is not specified. Continue without radiometric calibration?",
						                        "Warning", MB_YESNO | MB_ICONWARNING);
						if (result == IDNO) {
							return 0;
						}
						g_enableRadio = false;
					}

					g_guiSubmitted = true;
					DestroyWindow(hwnd);
					break;
				}

				case IDC_AUTO_DETECT_CHECK: {
					// When auto-detect is enabled, ensure 2-point click is disabled
					if (SendMessage(hAutoDetectCheck, BM_GETCHECK, 0, 0) == BST_CHECKED) {
						SendMessage(hTwoPointClickCheck, BM_SETCHECK, BST_UNCHECKED, 0);
					}
					break;
				}

				case IDC_TWO_POINT_CLICK_CHECK: {
					// When 2-point click is enabled, ensure auto-detect is disabled
					if (SendMessage(hTwoPointClickCheck, BM_GETCHECK, 0, 0) == BST_CHECKED) {
						SendMessage(hAutoDetectCheck, BM_SETCHECK, BST_UNCHECKED, 0);
					}
					break;
				}
			}
			break;
		}

		case WM_CLOSE:
			g_guiCancelled = true;
			DestroyWindow(hwnd);
			break;

		case WM_DESTROY:
			PostQuitMessage(0);
			break;

		default:
			return DefWindowProc(hwnd, message, wParam, lParam);
	}
	return 0;
}

#endif
