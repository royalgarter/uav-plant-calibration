#ifdef WINGUI

#define UNICODE
#define _UNICODE

#include <windows.h>
#include <shlobj.h>
#include <commctrl.h>
#include <string>
#include <iostream>
#include <vector>

using namespace std;

// Link with libraries
#pragma comment(lib, "user32.lib")
#pragma comment(lib, "gdi32.lib")
#pragma comment(lib, "Comctl32.lib")
#pragma comment(lib, "Comdlg32.lib")
#pragma comment(lib, "Shell32.lib")
#pragma comment(lib, "Ole32.lib")

// Define SS_GRAYTEXT if not available
#ifndef SS_GRAYTEXT
#define SS_GRAYTEXT 0x00000008L
#endif

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
#define IDC_TEMPLATE_FILE_EDIT    112
#define IDC_BROWSE_TEMPLATE_BTN   113

// --- GLOBAL STATE ---
HWND hInputFolderEdit, hOutputFolderEdit, hRadioRefFileEdit, hTemplateFileEdit;
HWND hRadioCheck, hTwoPointClickCheck, hAutoDetectCheck, hBoardThicknessEdit;
HWND hBrowseInputBtn, hBrowseOutputBtn, hBrowseRadioRefBtn, hBrowseTemplateBtn;

string g_inputFolder, g_outputFolder, g_radioRefFile, g_radioTemplatePath;
bool g_enableRadio = false;
bool g_twoPointClickMode = false;
bool g_autoDetectBoard = false;
int g_boardThickness = 0;

// Background brush for consistent gray color
HBRUSH g_hBackgroundBrush = nullptr;

bool g_guiSubmitted = false;
bool g_guiCancelled = false;

// Forward declaration
LRESULT CALLBACK WndProc(HWND hwnd, UINT message, WPARAM wParam, LPARAM lParam);

// Helper to convert string to wstring
wstring toWide(const string& s) {
	if (s.empty()) return L"";
	int size = MultiByteToWideChar(CP_UTF8, 0, s.c_str(), -1, nullptr, 0);
	wstring result(size - 1, 0);
	MultiByteToWideChar(CP_UTF8, 0, s.c_str(), -1, &result[0], size);
	return result;
}

// Helper to convert wstring to string
string fromWide(const wstring& s) {
	if (s.empty()) return "";
	int size = WideCharToMultiByte(CP_UTF8, 0, s.c_str(), -1, nullptr, 0, nullptr, nullptr);
	string result(size - 1, 0);
	WideCharToMultiByte(CP_UTF8, 0, s.c_str(), -1, &result[0], size, nullptr, nullptr);
	return result;
}

// --- FILE DIALOG HELPERS ---
string browseForFolder(HWND hwnd, const string& title) {
	BROWSEINFOW bi = {};
	bi.hwndOwner = hwnd;
	bi.lpszTitle = toWide(title).c_str();
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

string browseForFile(HWND hwnd, const string& title, const string& filter) {
	OPENFILENAMEW ofn = {};
	wchar_t szFile[MAX_PATH] = L"";

	ofn.lStructSize = sizeof(OPENFILENAMEW);
	ofn.hwndOwner = hwnd;
	ofn.lpstrFile = szFile;
	ofn.nMaxFile = MAX_PATH;
	ofn.lpstrFilter = toWide(filter).c_str();
	ofn.lpstrTitle = toWide(title).c_str();
	ofn.Flags = OFN_FILEMUSTEXIST | OFN_PATHMUSTEXIST;

	if (GetOpenFileNameW(&ofn)) {
		return fromWide(szFile);
	}
	return "";
}

// --- GUI ENTRY POINT ---
bool runCalibGui(string& inputFolder, string& outputFolder, string& radioRefFile,
                 bool& enableRadio, bool& twoPointClickMode, bool& autoDetectBoard, int& boardThickness,
                 string& radioTemplatePath) {
	// Initialize COM for file dialogs
	CoInitialize(nullptr);

	// Initialize common controls
	INITCOMMONCONTROLSEX icex;
	icex.dwSize = sizeof(INITCOMMONCONTROLSEX);
	icex.dwICC = ICC_STANDARD_CLASSES;
	InitCommonControlsEx(&icex);

	// Create gray background brush (COLOR_BTNFACE is the standard dialog gray)
	g_hBackgroundBrush = CreateSolidBrush(GetSysColor(COLOR_BTNFACE));

	const wchar_t CLASS_NAME[] = L"UavCalibratorWindowClass";

	WNDCLASSEXW wc = {};
	wc.cbSize = sizeof(WNDCLASSEXW);
	wc.lpfnWndProc = WndProc;
	wc.hInstance = GetModuleHandle(nullptr);
	wc.lpszClassName = CLASS_NAME;
	wc.hbrBackground = g_hBackgroundBrush;
	wc.hCursor = LoadCursor(nullptr, IDC_ARROW);
	wc.hIcon = LoadIcon(nullptr, IDI_APPLICATION);

	if (!RegisterClassExW(&wc)) {
		MessageBoxW(nullptr, L"Window Registration Failed!", L"Error", MB_ICONEXCLAMATION | MB_OK);
		return false;
	}

	HWND hwnd = CreateWindowExW(
		0,
		CLASS_NAME,
		L"UAV Plant Calibration",
		WS_OVERLAPPEDWINDOW & ~WS_THICKFRAME & ~WS_MAXIMIZEBOX,
		CW_USEDEFAULT, CW_USEDEFAULT,
		720, 580,
		nullptr,
		nullptr,
		GetModuleHandle(nullptr),
		nullptr
	);

	if (hwnd == nullptr) {
		MessageBoxW(nullptr, L"Window Creation Failed!", L"Error", MB_ICONEXCLAMATION | MB_OK);
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
		radioTemplatePath = g_radioTemplatePath;
		enableRadio = g_enableRadio;
		twoPointClickMode = g_twoPointClickMode;
		autoDetectBoard = g_autoDetectBoard;
		boardThickness = g_boardThickness;
	}

	// Uninitialize COM
	CoUninitialize();

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
			CreateWindowExW(0, L"STATIC", L"UAV Plant Calibration Parameters", WS_VISIBLE | WS_CHILD | SS_CENTER,
							xPos, yPos, 660, 30, hwnd, nullptr, GetModuleHandle(nullptr), nullptr);
			yPos += 40;

			// Input Folder
			CreateWindowExW(0, L"STATIC", L"Input Folder:", WS_VISIBLE | WS_CHILD,
							xPos, yPos, labelWidth, height, hwnd, nullptr, GetModuleHandle(nullptr), nullptr);
			hInputFolderEdit = CreateWindowExW(0, L"EDIT", L".input", WS_VISIBLE | WS_CHILD | WS_BORDER | ES_AUTOHSCROLL,
											xPos + labelWidth, yPos, editWidth, height, hwnd, (HMENU)IDC_INPUT_FOLDER_EDIT, GetModuleHandle(nullptr), nullptr);
			hBrowseInputBtn = CreateWindowExW(0, L"BUTTON", L"Browse...", WS_VISIBLE | WS_CHILD | BS_PUSHBUTTON,
											xPos + labelWidth + editWidth + 10, yPos - 2, btnWidth, 28, hwnd, (HMENU)IDC_BROWSE_INPUT_BTN, GetModuleHandle(nullptr), nullptr);
			yPos += gap;

			// Output Folder
			CreateWindowExW(0, L"STATIC", L"Output Folder:", WS_VISIBLE | WS_CHILD,
							xPos, yPos, labelWidth, height, hwnd, nullptr, GetModuleHandle(nullptr), nullptr);
			hOutputFolderEdit = CreateWindowExW(0, L"EDIT", L".output", WS_VISIBLE | WS_CHILD | WS_BORDER | ES_AUTOHSCROLL,
												xPos + labelWidth, yPos, editWidth, height, hwnd, (HMENU)IDC_OUTPUT_FOLDER_EDIT, GetModuleHandle(nullptr), nullptr);
			hBrowseOutputBtn = CreateWindowExW(0, L"BUTTON", L"Browse...", WS_VISIBLE | WS_CHILD | BS_PUSHBUTTON,
											xPos + labelWidth + editWidth + 10, yPos - 2, btnWidth, 28, hwnd, (HMENU)IDC_BROWSE_OUTPUT_BTN, GetModuleHandle(nullptr), nullptr);
			yPos += gap;

			// Radiometric Reference File
			CreateWindowExW(0, L"STATIC", L"Radiometric Ref File:", WS_VISIBLE | WS_CHILD,
							xPos, yPos, labelWidth, height, hwnd, nullptr, GetModuleHandle(nullptr), nullptr);
			hRadioRefFileEdit = CreateWindowExW(0, L"EDIT", L"radiometric_reference.csv", WS_VISIBLE | WS_CHILD | WS_BORDER | ES_AUTOHSCROLL,
												xPos + labelWidth, yPos, editWidth, height, hwnd, (HMENU)IDC_RADIO_REF_FILE_EDIT, GetModuleHandle(nullptr), nullptr);
			hBrowseRadioRefBtn = CreateWindowExW(0, L"BUTTON", L"Browse...", WS_VISIBLE | WS_CHILD | BS_PUSHBUTTON,
												xPos + labelWidth + editWidth + 10, yPos - 2, btnWidth, 28, hwnd, (HMENU)IDC_BROWSE_RADIO_REF_BTN, GetModuleHandle(nullptr), nullptr);
			yPos += gap;

			// Radiometric Template File
			CreateWindowExW(0, L"STATIC", L"Board Template File:", WS_VISIBLE | WS_CHILD,
							xPos, yPos, labelWidth, height, hwnd, nullptr, GetModuleHandle(nullptr), nullptr);
			hTemplateFileEdit = CreateWindowExW(0, L"EDIT", L"example/calib/radiometric_board.jpg", WS_VISIBLE | WS_CHILD | WS_BORDER | ES_AUTOHSCROLL,
												xPos + labelWidth, yPos, editWidth, height, hwnd, (HMENU)IDC_TEMPLATE_FILE_EDIT, GetModuleHandle(nullptr), nullptr);
			hBrowseTemplateBtn = CreateWindowExW(0, L"BUTTON", L"Browse...", WS_VISIBLE | WS_CHILD | BS_PUSHBUTTON,
												xPos + labelWidth + editWidth + 10, yPos - 2, btnWidth, 28, hwnd, (HMENU)IDC_BROWSE_TEMPLATE_BTN, GetModuleHandle(nullptr), nullptr);
			yPos += gap;

			// Enable Radiometric Calibration Checkbox
			hRadioCheck = CreateWindowExW(0, L"BUTTON", L"Enable Radiometric Calibration (--radio)",
									   WS_VISIBLE | WS_CHILD | BS_AUTOCHECKBOX,
									   xPos, yPos, 400, height, hwnd, (HMENU)IDC_RADIO_CHECK, GetModuleHandle(nullptr), nullptr);
			yPos += gap - 10;

			// 2-Point Click Mode Checkbox
			hTwoPointClickCheck = CreateWindowExW(0, L"BUTTON", L"2-Point Click Mode for Board Detection",
											   WS_VISIBLE | WS_CHILD | BS_AUTOCHECKBOX,
											   xPos, yPos, 400, height, hwnd, (HMENU)IDC_TWO_POINT_CLICK_CHECK, GetModuleHandle(nullptr), nullptr);
			yPos += gap - 10;

			// Auto-Detect Board Checkbox
			hAutoDetectCheck = CreateWindowExW(0, L"BUTTON", L"Auto-Detect Radiometric Board (--auto)",
											WS_VISIBLE | WS_CHILD | BS_AUTOCHECKBOX,
											xPos, yPos, 400, height, hwnd, (HMENU)IDC_AUTO_DETECT_CHECK, GetModuleHandle(nullptr), nullptr);
			yPos += gap - 10;

			// Board Thickness
			CreateWindowExW(0, L"STATIC", L"Thickness (px):", WS_VISIBLE | WS_CHILD,
							xPos, yPos, labelWidth, height, hwnd, nullptr, GetModuleHandle(nullptr), nullptr);
			hBoardThicknessEdit = CreateWindowExW(0, L"EDIT", L"10", WS_VISIBLE | WS_CHILD | WS_BORDER | ES_NUMBER | ES_AUTOHSCROLL,
												xPos + labelWidth, yPos, 100, height, hwnd, (HMENU)IDC_BOARD_THICKNESS_EDIT, GetModuleHandle(nullptr), nullptr);
			
			// CreateWindowExW(0, L"STATIC", L"(0 = auto-detect thickness, -1 = disable auto-detect)", WS_VISIBLE | WS_CHILD | SS_GRAYTEXT,
			// 				xPos + labelWidth + 110, yPos + 3, 300, height, hwnd, nullptr, GetModuleHandle(nullptr), nullptr);
			
			yPos += gap + 10;

			// Separator line
			CreateWindowExW(0, L"STATIC", L"", WS_VISIBLE | WS_CHILD | SS_ETCHEDHORZ,
							xPos, yPos, 660, 4, hwnd, nullptr, GetModuleHandle(nullptr), nullptr);
			yPos += 30;

			// Start Button
			CreateWindowExW(0, L"BUTTON", L"START CALIBRATION", WS_VISIBLE | WS_CHILD | BS_PUSHBUTTON,
							xPos, yPos, 660, 45, hwnd, (HMENU)IDC_START_BUTTON, GetModuleHandle(nullptr), nullptr);

			break;
		}

		case WM_COMMAND: {
			switch (LOWORD(wParam)) {
				case IDC_BROWSE_INPUT_BTN: {
					string folder = browseForFolder(hwnd, "Select Input Folder");
					if (!folder.empty()) {
						SetWindowTextA(hInputFolderEdit, folder.c_str());
					}
					break;
				}

				case IDC_BROWSE_OUTPUT_BTN: {
					string folder = browseForFolder(hwnd, "Select Output Folder");
					if (!folder.empty()) {
						SetWindowTextA(hOutputFolderEdit, folder.c_str());
					}
					break;
				}

				case IDC_BROWSE_RADIO_REF_BTN: {
					string file = browseForFile(hwnd, "Select Radiometric Reference CSV",
					                            "CSV Files (*.csv)\0*.csv\0All Files (*.*)\0*.*\0");
					if (!file.empty()) {
						SetWindowTextA(hRadioRefFileEdit, file.c_str());
					}
					break;
				}

				case IDC_BROWSE_TEMPLATE_BTN: {
					string file = browseForFile(hwnd, "Select Board Template Image",
					                            "Image Files (*.jpg;*.jpeg;*.png;*.bmp)\0*.jpg;*.jpeg;*.png;*.bmp\0All Files (*.*)\0*.*\0");
					if (!file.empty()) {
						SetWindowTextA(hTemplateFileEdit, file.c_str());
					}
					break;
				}

				case IDC_START_BUTTON: {
					wchar_t buffer[MAX_PATH];

					// Get input folder
					GetWindowTextW(hInputFolderEdit, buffer, MAX_PATH);
					g_inputFolder = fromWide(buffer);

					// Get output folder
					GetWindowTextW(hOutputFolderEdit, buffer, MAX_PATH);
					g_outputFolder = fromWide(buffer);

					// Get radiometric reference file
					GetWindowTextW(hRadioRefFileEdit, buffer, MAX_PATH);
					g_radioRefFile = fromWide(buffer);

					// Get radiometric template file
					GetWindowTextW(hTemplateFileEdit, buffer, MAX_PATH);
					g_radioTemplatePath = fromWide(buffer);

					// Get checkbox states
					g_enableRadio = (SendMessage(hRadioCheck, BM_GETCHECK, 0, 0) == BST_CHECKED);
					g_twoPointClickMode = (SendMessage(hTwoPointClickCheck, BM_GETCHECK, 0, 0) == BST_CHECKED);
					g_autoDetectBoard = (SendMessage(hAutoDetectCheck, BM_GETCHECK, 0, 0) == BST_CHECKED);

					// Get board thickness
					GetWindowTextW(hBoardThicknessEdit, buffer, 10);
					try {
						g_boardThickness = stoi(fromWide(buffer));
					} catch (const exception& e) {
						MessageBoxW(hwnd, L"Invalid board thickness value. Please enter a valid integer.", L"Error", MB_ICONERROR | MB_OK);
						return 0;
					}

					// Validate inputs
					if (g_inputFolder.empty()) {
						MessageBoxW(hwnd, L"Please specify an input folder.", L"Validation Error", MB_ICONWARNING | MB_OK);
						return 0;
					}

					if (g_outputFolder.empty()) {
						MessageBoxW(hwnd, L"Please specify an output folder.", L"Validation Error", MB_ICONWARNING | MB_OK);
						return 0;
					}

					if (g_enableRadio && g_radioRefFile.empty()) {
						int result = MessageBoxW(hwnd, L"Radiometric reference file is not specified. Continue without radiometric calibration?",
						                        L"Warning", MB_YESNO | MB_ICONWARNING);
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

		case WM_CTLCOLORSTATIC: {
			// Set gray background for static controls (labels, edit boxes, etc.)
			HDC hdcStatic = (HDC)wParam;
			SetBkColor(hdcStatic, GetSysColor(COLOR_BTNFACE));
			SetTextColor(hdcStatic, GetSysColor(COLOR_BTNTEXT));
			return (LRESULT)g_hBackgroundBrush;
		}

		case WM_CTLCOLOREDIT: {
			// Set gray background for edit controls
			HDC hdcEdit = (HDC)wParam;
			SetBkColor(hdcEdit, GetSysColor(COLOR_BTNFACE));
			SetTextColor(hdcEdit, GetSysColor(COLOR_BTNTEXT));
			return (LRESULT)g_hBackgroundBrush;
		}

		case WM_DESTROY:
			if (g_hBackgroundBrush) {
				DeleteObject(g_hBackgroundBrush);
				g_hBackgroundBrush = nullptr;
			}
			PostQuitMessage(0);
			break;

		default:
			return DefWindowProc(hwnd, message, wParam, lParam);
	}
	return 0;
}

#endif
