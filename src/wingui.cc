#ifdef WINGUI

#include <windows.h> // For Windows GUI
#include <commctrl.h> // For common controls like edit boxes
#include <string>
#include <iostream>

using namespace std;

// Link with libraries
#pragma comment(lib, "user32.lib")
#pragma comment(lib, "gdi32.lib")
#pragma comment(lib, "Comctl32.lib") // For InitCommonControlsEx

// --- WINDOWS GUI GLOBALS AND CONSTANTS ---
// Control IDs
#define IDC_SRC_PATH_EDIT 101
#define IDC_DEST_PATH_EDIT 102
#define IDC_SAMPLES_DIR_EDIT 103
#define IDC_CHECKBOARD_WIDTH_EDIT 104
#define IDC_CHECKBOARD_HEIGHT_EDIT 105
#define IDC_START_BUTTON 106

// Global handles for controls and data storage
HWND hSrcPathEdit, hDestPathEdit, hSamplesDirEdit, hWidthEdit, hHeightEdit;
string g_srcPath, g_destPath, g_samplesDir;
int g_checkboardWidth, g_checkboardHeight;
bool g_guiSubmitted = false;
bool g_guiCancelled = false;

// Forward declaration of WndProc
LRESULT CALLBACK WndProc(HWND hwnd, UINT message, WPARAM wParam, LPARAM lParam);

// Function to run the Windows GUI
bool runWinGuiMode(string& src, string& dest, string& samples, int& w, int& h) {
	// To ensure common controls are available
	INITCOMMONCONTROLSEX icex;
	icex.dwSize = sizeof(INITCOMMONCONTROLSEX);
	icex.dwICC = ICC_STANDARD_CLASSES;
	InitCommonControlsEx(&icex);

	const char CLASS_NAME[] = "FisheyeCalibratorWindowClass";

	WNDCLASSEX wc = {};
	wc.cbSize = sizeof(WNDCLASSEX);
	wc.lpfnWndProc = WndProc;
	wc.hInstance = GetModuleHandle(nullptr);
	wc.lpszClassName = CLASS_NAME;
	wc.hbrBackground = (HBRUSH)(COLOR_WINDOW + 1); // Set background color

	if (!RegisterClassEx(&wc)) {
		MessageBox(nullptr, "Window Registration Failed!", "Error", MB_ICONEXCLAMATION | MB_OK);
		return false;
	}

	HWND hwnd = CreateWindowEx(
		0,                              // Optional window style
		CLASS_NAME,                     // Window class
		"Fisheye Calibrator Input",     // Window text
		WS_OVERLAPPEDWINDOW | WS_CLIPCHILDREN, // Window style
		CW_USEDEFAULT, CW_USEDEFAULT,   // Position
		660, 420,                       // Size (width, height)
		nullptr,                        // Parent window
		nullptr,                        // Menu
		GetModuleHandle(nullptr),       // Instance handle
		nullptr                         // Additional application data
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
			break; // Exit message loop
		}
	}

	if (g_guiSubmitted) {
		src = g_srcPath;
		dest = g_destPath;
		samples = g_samplesDir;
		w = g_checkboardWidth;
		h = g_checkboardHeight;
	}

	return g_guiSubmitted;
}

// --- WINDOW PROCEDURE ---
LRESULT CALLBACK WndProc(HWND hwnd, UINT message, WPARAM wParam, LPARAM lParam) {
	switch (message) {
		case WM_CREATE: {
			int yPos = 20;
			int xPos = 20;
			int width = 600;
			int height = 24;
			int labelWidth = 150;
			int editWidth = 450;
			int gap = 50;

			// Source Image Path
			CreateWindow("STATIC", "Source Image Path:", WS_VISIBLE | WS_CHILD,
						 xPos, yPos, labelWidth, height, hwnd, nullptr, GetModuleHandle(nullptr), nullptr);
			hSrcPathEdit = CreateWindow("EDIT", "example/samples/IMG-0.jpg", WS_VISIBLE | WS_CHILD | WS_BORDER | ES_AUTOHSCROLL,
										xPos + labelWidth, yPos, editWidth, height, hwnd, (HMENU)IDC_SRC_PATH_EDIT, GetModuleHandle(nullptr), nullptr);
			yPos += gap;

			// Destination Image Path
			CreateWindow("STATIC", "Destination Image Path:", WS_VISIBLE | WS_CHILD,
						 xPos, yPos, labelWidth, height, hwnd, nullptr, GetModuleHandle(nullptr), nullptr);
			hDestPathEdit = CreateWindow("EDIT", "undistorted.jpg", WS_VISIBLE | WS_CHILD | WS_BORDER | ES_AUTOHSCROLL,
										 xPos + labelWidth, yPos, editWidth, height, hwnd, (HMENU)IDC_DEST_PATH_EDIT, GetModuleHandle(nullptr), nullptr);
			yPos += gap;

			// Samples Directory
			CreateWindow("STATIC", "Samples Directory:", WS_VISIBLE | WS_CHILD,
						 xPos, yPos, labelWidth, height, hwnd, nullptr, GetModuleHandle(nullptr), nullptr);
			hSamplesDirEdit = CreateWindow("EDIT", "example/samples", WS_VISIBLE | WS_CHILD | WS_BORDER | ES_AUTOHSCROLL,
										   xPos + labelWidth, yPos, editWidth, height, hwnd, (HMENU)IDC_SAMPLES_DIR_EDIT, GetModuleHandle(nullptr), nullptr);
			yPos += gap;

			// Checkboard Width
			CreateWindow("STATIC", "Checkboard Width (cols):", WS_VISIBLE | WS_CHILD,
						 xPos, yPos, labelWidth, height, hwnd, nullptr, GetModuleHandle(nullptr), nullptr);
			hWidthEdit = CreateWindow("EDIT", "9", WS_VISIBLE | WS_CHILD | WS_BORDER | ES_NUMBER | ES_AUTOHSCROLL,
									  xPos + labelWidth, yPos, editWidth / 2, height, hwnd, (HMENU)IDC_CHECKBOARD_WIDTH_EDIT, GetModuleHandle(nullptr), nullptr);
			yPos += gap;

			// Checkboard Height
			CreateWindow("STATIC", "Checkboard Height (rows):", WS_VISIBLE | WS_CHILD,
						 xPos, yPos, labelWidth, height, hwnd, nullptr, GetModuleHandle(nullptr), nullptr);
			hHeightEdit = CreateWindow("EDIT", "6", WS_VISIBLE | WS_CHILD | WS_BORDER | ES_NUMBER | ES_AUTOHSCROLL,
									   xPos + labelWidth, yPos, editWidth / 2, height, hwnd, (HMENU)IDC_CHECKBOARD_HEIGHT_EDIT, GetModuleHandle(nullptr), nullptr);
			yPos += gap + 20;

			// Start Button
			CreateWindow("BUTTON", "Start Calibration", WS_VISIBLE | WS_CHILD | BS_PUSHBUTTON,
						 xPos, yPos, width, 40, hwnd, (HMENU)IDC_START_BUTTON, GetModuleHandle(nullptr), nullptr);

			break;
		}
		case WM_COMMAND: {
			if (LOWORD(wParam) == IDC_START_BUTTON) {
				char buffer[MAX_PATH]; // Use MAX_PATH for paths, smaller for numbers

				GetWindowText(hSrcPathEdit, buffer, MAX_PATH);
				g_srcPath = buffer;

				GetWindowText(hDestPathEdit, buffer, MAX_PATH);
				g_destPath = buffer;

				GetWindowText(hSamplesDirEdit, buffer, MAX_PATH);
				g_samplesDir = buffer;

				GetWindowText(hWidthEdit, buffer, 10); // Checkboard dimensions are small
				try {
					g_checkboardWidth = stoi(buffer);
				} catch (const exception& e) {
					MessageBox(hwnd, ("Invalid width input: " + string(e.what())).c_str(), "Error", MB_ICONERROR | MB_OK);
					return 0;
				}

				GetWindowText(hHeightEdit, buffer, 10);
				try {
					g_checkboardHeight = stoi(buffer);
				} catch (const exception& e) {
					MessageBox(hwnd, ("Invalid height input: " + string(e.what())).c_str(), "Error", MB_ICONERROR | MB_OK);
					return 0;
				}

				g_guiSubmitted = true;
				DestroyWindow(hwnd); // Close the GUI window
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
