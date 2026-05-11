#include "stdafx.h"
#include "Calibration.h"
#include "CalibrationMetrics.h"   // WriteLogAnnotation — used to capture VR_Init
                                  // and overlay-creation outcomes for cross-launch
                                  // diff (script vs Start menu).
#include "Configuration.h"
#include "EmbeddedFiles.h"
#include "UserInterface.h"

#include <imgui/imgui.h>
#include <imgui/imgui_internal.h>
#include <imgui/backends/imgui_impl_glfw.h>
#include <imgui/backends/imgui_impl_opengl3.h>
#include <implot/implot.h>
#include <GL/gl3w.h>
#define GLFW_EXPOSE_NATIVE_WIN32
#include <GLFW/glfw3.h>
#include <GLFW/glfw3native.h>
#include <openvr.h>
#include <direct.h>
#include <chrono>
#include <thread>
#include <dwmapi.h>
#include <algorithm>
#include <filesystem>
#include <fstream>

#include <shellapi.h>

#include <stb_image.h>

#pragma comment(linker,"\"/manifestdependency:type='win32' \
name='Microsoft.Windows.Common-Controls' version='6.0.0.0' \
processorArchitecture='*' publicKeyToken='6595b64144ccf1df' language='*'\"")

#define LEGACY_OPENVR_APPLICATION_KEY "pushrax.SpaceCalibrator"
#define OPENVR_APPLICATION_KEY "steam.overlay.3368750"
std::string c_SPACE_CALIBRATOR_STEAM_APP_ID = "3368750";
std::string c_STEAMVR_STEAM_APP_ID = "250820";
constexpr const char* STEAM_MUTEX_KEY = "Global\\MUTEX__SpaceCalibrator_Steam";
HANDLE hSteamMutex = INVALID_HANDLE_VALUE;
// Set true at startup when we detect that Steam's library has the Steam-store
// version of Space Calibrator installed alongside this fork. We're the GitHub
// fork (actively maintained); the Steam Store version is the upstream that
// shipped before the fork and isn't getting these fixes. When both are
// installed they fight over the same SteamVR overlay manifest, so the user
// should remove the Steam version. The reverse-direction check the original
// code did ("uninstall the GitHub version") was wrong for this fork.
bool s_isSteamVersionInstalled = false;
std::string s_steamVersionPath;     // populated when detection succeeds, for display.

extern "C" __declspec(dllexport) DWORD NvOptimusEnablement = 0x00000001;
extern "C" __declspec(dllexport) DWORD AmdPowerXpressRequestHighPerformance = 0x00000001;

void CreateConsole()
{
	static bool created = false;
	if (!created)
	{
		AllocConsole();
		FILE *file = nullptr;
		freopen_s(&file, "CONIN$", "r", stdin);
		freopen_s(&file, "CONOUT$", "w", stdout);
		freopen_s(&file, "CONOUT$", "w", stderr);
		created = true;
	}
}

std::string GetRegistryString(const HKEY hKeyGroup, const char* szRegistryKey, const char* szRegistryPropKey) noexcept {
	DWORD dwType = REG_SZ;
	HKEY hKey = 0;
	// 4 KiB string buffer
	char buffer[4 * 1024] = {};
	DWORD bufferSize = sizeof(buffer);
	LSTATUS lResult = RegOpenKeyExA(hKeyGroup, szRegistryKey, 0, KEY_QUERY_VALUE, &hKey);
	if (lResult == ERROR_SUCCESS) {
		lResult = RegQueryValueExA(hKey, szRegistryPropKey, NULL, &dwType, (LPBYTE)&buffer, &bufferSize);
		if (lResult == ERROR_SUCCESS) {
			return buffer;
		}
	}
	return "";
}

//#define DEBUG_LOGS

void GLFWErrorCallback(int error, const char* description)
{
	fprintf(stderr, "GLFW Error %d: %s\n", error, description);
}

void openGLDebugCallback(GLenum source, GLenum type, GLuint id, GLenum severity, GLsizei length, const GLchar *message, const void *userParam)
{
	fprintf(stderr, "OpenGL Debug %u: %.*s\n", id, length, message);
}

static void HandleCommandLine(LPWSTR lpCmdLine);

static GLFWwindow *glfwWindow = nullptr;
static vr::VROverlayHandle_t overlayMainHandle = 0, overlayThumbnailHandle = 0;

// Asks the main loop to exit cleanly on the next frame. Used by the in-app
// updater to hand off to the installer (the installer can't replace a running
// EXE, so we have to be gone before it finishes copying files).
void RequestExit() {
	if (glfwWindow) glfwSetWindowShouldClose(glfwWindow, 1);
}

static GLuint fboHandle = 0, fboTextureHandle = 0;
static int fboTextureWidth = 0, fboTextureHeight = 0;

static char cwd[MAX_PATH];
const float MINIMIZED_MAX_FPS = 60.0f;

enum DWMA_USE_IMMSERSIVE_DARK_MODE_ENUM {
	DWMA_USE_IMMERSIVE_DARK_MODE = 20,
	DWMA_USE_IMMERSIVE_DARK_MODE_PRE_20H1 = 19,
};

const bool EnableDarkModeTopBar(const HWND windowHwmd) {
	const BOOL darkBorder = TRUE;
	const bool ok =
		SUCCEEDED(DwmSetWindowAttribute(windowHwmd, DWMA_USE_IMMERSIVE_DARK_MODE, &darkBorder, sizeof(darkBorder)))
		|| SUCCEEDED(DwmSetWindowAttribute(windowHwmd, DWMA_USE_IMMERSIVE_DARK_MODE_PRE_20H1, &darkBorder, sizeof(darkBorder)));
	return ok;
}

void CreateGLFWWindow()
{
	glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 2);
	glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
	glfwWindowHint(GLFW_RESIZABLE, false);

#ifdef DEBUG_LOGS
	glfwWindowHint(GLFW_OPENGL_DEBUG_CONTEXT, GL_TRUE);
#endif

	fboTextureWidth = 1200;
	fboTextureHeight = 800;

	glfwWindow = glfwCreateWindow(fboTextureWidth, fboTextureHeight, "Space Calibrator", nullptr, nullptr);
	if (!glfwWindow)
		throw std::runtime_error("Failed to create window");

	glfwMakeContextCurrent(glfwWindow);
	glfwSwapInterval(1);
	gl3wInit();

	// Minimise the window
	glfwIconifyWindow(glfwWindow);
	HWND windowHwmd = glfwGetWin32Window(glfwWindow);
	EnableDarkModeTopBar(windowHwmd);

	// Load icon and set it in the window
	GLFWimage images[1] = {};
	std::string iconPath = cwd;
	iconPath += "\\taskbar_icon.png";
	images[0].pixels = stbi_load(iconPath.c_str(), &images[0].width, &images[0].height, 0, 4);
	glfwSetWindowIcon(glfwWindow, 1, images);
	stbi_image_free(images[0].pixels);

#ifdef DEBUG_LOGS
	glDebugMessageCallback(openGLDebugCallback, nullptr);
	glEnable(GL_DEBUG_OUTPUT);
#endif

	ImGui::CreateContext();
	ImPlot::CreateContext();
	ImGuiIO &io = ImGui::GetIO();
	io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;
	io.ConfigFlags |= ImGuiConfigFlags_NavEnableGamepad;
	io.IniFilename = nullptr;
	io.Fonts->AddFontFromMemoryCompressedTTF(DroidSans_compressed_data, DroidSans_compressed_size, 24.0f);

	ImGui_ImplGlfw_InitForOpenGL(glfwWindow, true);
	ImGui_ImplOpenGL3_Init("#version 330");

	ImGui::StyleColorsDark();

	glGenTextures(1, &fboTextureHandle);
	glBindTexture(GL_TEXTURE_2D, fboTextureHandle);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, fboTextureWidth, fboTextureHeight, 0, GL_RGBA, GL_UNSIGNED_BYTE, 0);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);

	glGenFramebuffers(1, &fboHandle);
	glBindFramebuffer(GL_FRAMEBUFFER, fboHandle);
	glFramebufferTexture(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, fboTextureHandle, 0);

	GLenum drawBuffers[1] = { GL_COLOR_ATTACHMENT0 };
	glDrawBuffers(1, drawBuffers);

	if (glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE)
	{
		throw std::runtime_error("OpenGL framebuffer incomplete");
	}
}

void TryCreateVROverlay() {
	if (overlayMainHandle || !vr::VROverlay())
		return;

	vr::VROverlayError error = vr::VROverlay()->CreateDashboardOverlay(
		OPENVR_APPLICATION_KEY, "Space Calibrator",
		&overlayMainHandle, &overlayThumbnailHandle
	);

	// Annotation BEFORE the throw paths so we capture the error code even if
	// the throw aborts startup. KeyInUse specifically would tell us another
	// SC overlay is already registered with vrserver — relevant to the
	// script-relaunch hypothesis (force-killed previous instance left a
	// stale registration that vrserver hasn't reaped yet).
	{
		char annot[256];
		snprintf(annot, sizeof annot, "vr_overlay_create: result=%d (%s) mainHandle=%llu thumbHandle=%llu",
			(int)error,
			vr::VROverlay()->GetOverlayErrorNameFromEnum(error),
			(unsigned long long)overlayMainHandle,
			(unsigned long long)overlayThumbnailHandle);
		Metrics::WriteLogAnnotation(annot);
	}

	if (error == vr::VROverlayError_KeyInUse) {
		throw std::runtime_error("Another instance of Space Calibrator is already running");
	} else if (error != vr::VROverlayError_None) {
		throw std::runtime_error("Error creating VR overlay: " + std::string(vr::VROverlay()->GetOverlayErrorNameFromEnum(error)));
	}

	vr::VROverlay()->SetOverlayWidthInMeters(overlayMainHandle, 3.0f);
	vr::VROverlay()->SetOverlayInputMethod(overlayMainHandle, vr::VROverlayInputMethod_Mouse);
	vr::VROverlay()->SetOverlayFlag(overlayMainHandle, vr::VROverlayFlags_SendVRDiscreteScrollEvents, true);

	std::string iconPath = cwd;
	iconPath += "\\icon.png";
	vr::VROverlay()->SetOverlayFromFile(overlayThumbnailHandle, iconPath.c_str());
}

void ActivateMultipleDrivers()
{
	vr::EVRSettingsError vrSettingsError;
	bool enabled = vr::VRSettings()->GetBool(vr::k_pch_SteamVR_Section, vr::k_pch_SteamVR_ActivateMultipleDrivers_Bool, &vrSettingsError);

	if (vrSettingsError != vr::VRSettingsError_None)
	{
		std::string err = "Could not read \"" + std::string(vr::k_pch_SteamVR_ActivateMultipleDrivers_Bool) + "\" setting: "
			+ vr::VRSettings()->GetSettingsErrorNameFromEnum(vrSettingsError);

		throw std::runtime_error(err);
	}

	if (!enabled)
	{
		vr::VRSettings()->SetBool(vr::k_pch_SteamVR_Section, vr::k_pch_SteamVR_ActivateMultipleDrivers_Bool, true, &vrSettingsError);
		if (vrSettingsError != vr::VRSettingsError_None)
		{
			std::string err = "Could not set \"" + std::string(vr::k_pch_SteamVR_ActivateMultipleDrivers_Bool) + "\" setting: "
				+ vr::VRSettings()->GetSettingsErrorNameFromEnum(vrSettingsError);

			throw std::runtime_error(err);
		}

		std::cerr << "Enabled \"" << vr::k_pch_SteamVR_ActivateMultipleDrivers_Bool << "\" setting" << std::endl;
	}
	else
	{
		std::cerr << "\"" << vr::k_pch_SteamVR_ActivateMultipleDrivers_Bool << "\" setting previously enabled" << std::endl;
	}
}

// Forward decls for the deferred-VR connection helpers (full definitions
// live further down: VerifySetupCorrect at the manifest-handling block,
// InitCalibrator over in Calibration.cpp).
void VerifySetupCorrect();

// === Deferred VR connection ================================================
// The original startup called VR_Init() up front and threw a runtime_error if
// SteamVR wasn't running, which terminated the process before the user could
// see anything. The current flow: bring up the GLFW window + ImGui + load the
// profile FIRST, then attempt the VR connection in a retry loop driven by
// RunLoop. The user can browse the Settings / Logs / Advanced / Prediction
// tabs while waiting for SteamVR; calibration actions and device dropdowns
// are disabled until g_vrReady flips true.
//
// Flow:
//   - g_vrReady = false at startup.
//   - RunLoop calls TryInitVRStack() once per second when !g_vrReady.
//   - TryInitVRStack does the InitCalibrator + VerifySetupCorrect work,
//     non-throwing: any failure leaves g_vrReady false and stashes a
//     human-readable error for the banner.
//   - Once successful, sets g_vrReady = true. CalibrationTick (which already
//     short-circuits on !vr::VRSystem()) starts producing useful output and
//     the UI gates flip on.
static bool g_vrReady = false;
static std::string g_lastVRError;     // "Waiting for SteamVR..." style message.
static double g_lastVRRetryTime = -1e9;
static bool g_vrManifestSetupDone = false;  // VerifySetupCorrect runs once per VR session.

bool IsVRReady() { return g_vrReady; }
const std::string& LastVRConnectError() { return g_lastVRError; }

// Attempt the full connection sequence. Returns true if the stack is fully up.
// Does not throw; on partial failure cleans up VR_Shutdown so the next retry
// starts from a clean state.
//
// **Crucial**: we probe with VRApplication_Background first, NOT
// VRApplication_Overlay. Overlay-type init auto-launches SteamVR if it's not
// running; Background-type fails cleanly with VRInitError_Init_NoServerFor-
// BackgroundApp instead. We only upgrade to Overlay once the probe confirms
// SteamVR is already running, so the user can launch the calibrator without
// inadvertently spinning up the entire VR runtime.
static bool TryInitVRStack()
{
	if (g_vrReady) return true;

	// Phase 1: probe with Background. If SteamVR isn't up, this fails fast
	// without launching anything. Common errors:
	//   VRInitError_Init_NoServerForBackgroundApp -- vrserver isn't running.
	//   VRInitError_Init_VRClientDLLNotFound      -- SteamVR isn't installed.
	auto initError = vr::VRInitError_None;
	vr::VR_Init(&initError, vr::VRApplication_Background);
	{
		// Log every VR_Init outcome so we can see if SteamVR's response
		// differs across launch contexts (script vs Start menu).
		// VR_GetVRInitErrorAsEnglishDescription is safe to call even on
		// VRInitError_None (returns "None").
		char annot[256];
		snprintf(annot, sizeof annot, "vr_init_phase1_background: result=%d (%s)",
			(int)initError, vr::VR_GetVRInitErrorAsEnglishDescription(initError));
		Metrics::WriteLogAnnotation(annot);
	}
	if (initError != vr::VRInitError_None) {
		g_lastVRError = std::string("Waiting for SteamVR: ")
			+ vr::VR_GetVRInitErrorAsEnglishDescription(initError);
		return false;
	}

	// Phase 2: SteamVR confirmed up. Tear down the probe and re-init as
	// Overlay so we can create dashboard overlays. Doing it this way (probe
	// + shutdown + overlay-init) is the standard pattern for overlay apps
	// that want to be launched independently of SteamVR.
	vr::VR_Shutdown();

	vr::VR_Init(&initError, vr::VRApplication_Overlay);
	{
		char annot[256];
		snprintf(annot, sizeof annot, "vr_init_phase2_overlay: result=%d (%s)",
			(int)initError, vr::VR_GetVRInitErrorAsEnglishDescription(initError));
		Metrics::WriteLogAnnotation(annot);
	}
	if (initError != vr::VRInitError_None) {
		// Race: SteamVR was up a moment ago but the overlay init failed.
		// Could be a momentary state during SteamVR shutdown, or a permission
		// issue. Retry on the next tick.
		g_lastVRError = std::string("Waiting for SteamVR: ")
			+ vr::VR_GetVRInitErrorAsEnglishDescription(initError);
		return false;
	}

	// Catastrophic version-mismatch errors: openvr.dll on disk doesn't match
	// the headers we built against. Retrying won't fix it. Show the error and
	// keep VR shut down so we don't fight a broken runtime.
	if (!vr::VR_IsInterfaceVersionValid(vr::IVRSystem_Version)
		|| !vr::VR_IsInterfaceVersionValid(vr::IVRSettings_Version)
		|| !vr::VR_IsInterfaceVersionValid(vr::IVROverlay_Version))
	{
		g_lastVRError = "OpenVR interface version mismatch (runtime DLL out of date).";
		vr::VR_Shutdown();
		return false;
	}

	// Settings tweak (one-time per session, but we run it every successful
	// init -- it's idempotent and small). Failures here are non-fatal: log
	// and continue.
	try {
		ActivateMultipleDrivers();
	}
	catch (const std::exception& e) {
		fprintf(stderr, "[VR connect] ActivateMultipleDrivers warning: %s\n", e.what());
	}

	// Manifest registration: only do once per process lifetime. If we already
	// did it earlier in this process (or a previous successful init), don't
	// rerun -- it touches application registry state and we don't want to
	// reapply on every reconnect.
	if (!g_vrManifestSetupDone) {
		try {
			VerifySetupCorrect();
			g_vrManifestSetupDone = true;
		}
		catch (const std::exception& e) {
			fprintf(stderr, "[VR connect] VerifySetupCorrect warning: %s\n", e.what());
			// Don't gate VR readiness on manifest registration -- the calibration
			// stack works without the app being in the SteamVR overlay menu.
		}
	}

	// Driver IPC + shmem. THIS is the gate: if our driver isn't running (or
	// SteamVR's vrserver hasn't loaded it yet), Driver.Connect throws and we
	// can't do anything useful. Tear down VR and retry.
	try {
		InitCalibrator();
	}
	catch (const std::exception& e) {
		g_lastVRError = std::string("Waiting for Space Calibrator driver: ") + e.what();
		vr::VR_Shutdown();
		return false;
	}

	// All up.
	g_lastVRError.clear();
	g_vrReady = true;
	return true;
}

static char textBuf[0x400] = {};

static bool immediateRedraw;
void RequestImmediateRedraw() {
	immediateRedraw = true;
}

bool UninstallGithubSpaceCalibrator() {

	// find the uninstall key

	// HKLM\Software\Microsoft\Windows\CurrentVersion\Uninstall\OpenVRSpaceCalibrator
	std::string uninstallKeyValue = GetRegistryString(HKEY_LOCAL_MACHINE, "Software\\Microsoft\\Windows\\CurrentVersion\\Uninstall\\OpenVRSpaceCalibrator", "UninstallString");
	if (uninstallKeyValue.empty()) {
		// HKLM\SOFTWARE\WOW6432Node\Microsoft\Windows\CurrentVersion\Uninstall\OpenVRSpaceCalibrator
		uninstallKeyValue = GetRegistryString(HKEY_LOCAL_MACHINE, "SOFTWARE\\WOW6432Node\\Microsoft\\Windows\\CurrentVersion\\Uninstall\\OpenVRSpaceCalibrator", "UninstallString");
		if (uninstallKeyValue.empty()) {
			// HKCU\Software\Microsoft\Windows\CurrentVersion\Uninstall
			uninstallKeyValue = GetRegistryString(HKEY_CURRENT_USER, "Software\\Microsoft\\Windows\\CurrentVersion\\Uninstall\\OpenVRSpaceCalibrator", "UninstallString");
		}
	}

	printf("uninst: %s\n", uninstallKeyValue.c_str());

	if (!uninstallKeyValue.empty()) {

		int size_needed = MultiByteToWideChar(CP_UTF8, 0, &uninstallKeyValue[0], (int)uninstallKeyValue.size(), 0, 0);
		std::wstring uninstallPathUnicode(size_needed, 0);
		MultiByteToWideChar(CP_UTF8, 0, &uninstallKeyValue[0], (int)uninstallKeyValue.size(), &uninstallPathUnicode[0], size_needed);

		// split uninstall key into file and cli args
		std::wstring executablePath = uninstallPathUnicode;
		std::wstring commandLineArgs = uninstallPathUnicode;

		// some form of parsing incase a fork changes the executable path and stuff
		if (executablePath[0] == L'"') {
			// executable name is wrapped in double quotes, find closing quote
			auto it = std::find(executablePath.begin(), executablePath.end(), '"');
			it = std::find(it + 1, executablePath.end(), '"');
			if (it != executablePath.end()) {
				size_t idx = it - executablePath.begin();
				// now that we know where the quotes are, substring
				executablePath = uninstallPathUnicode.substr(1, idx - 1);
				commandLineArgs = uninstallPathUnicode.substr(idx + 1);
			}
		} else {
			// executable name is until the first space, find it and split accordingly
			auto it = std::find(executablePath.begin(), executablePath.end(), ' ');
			if (it != executablePath.end()) {
				size_t idx = it - executablePath.begin();
				// now that we know where the quotes are, substring
				executablePath = uninstallPathUnicode.substr(1, idx - 1);
				commandLineArgs = uninstallPathUnicode.substr(idx);
			}
		}

		SHELLEXECUTEINFO shExInfo = { 0 };
		shExInfo.cbSize = sizeof(shExInfo);
		shExInfo.fMask = SEE_MASK_NOCLOSEPROCESS;
		shExInfo.hwnd = 0;
		shExInfo.lpVerb = L"open";
		shExInfo.lpFile = L"ms-settings:appsfeatures";
		shExInfo.lpParameters = L"";
		shExInfo.lpDirectory = 0;
		shExInfo.nShow = SW_SHOW;
		shExInfo.hInstApp = 0;

		if (ShellExecuteExW(&shExInfo))
		{
			// valid
			return true;
		}
	}

	return false;
}

double lastFrameStartTime = glfwGetTime();
void RunLoop() {
	while (!glfwWindowShouldClose(glfwWindow))
	{
		double time = glfwGetTime();

		// Retry the VR connection once per second when it's not up. Costs
		// nothing when SteamVR is already connected (TryInitVRStack short-
		// circuits on g_vrReady) and lands the user in a fully-working state
		// the moment SteamVR finishes booting.
		if (!g_vrReady && (time - g_lastVRRetryTime) > 1.0) {
			g_lastVRRetryTime = time;
			TryInitVRStack();
		}

		TryCreateVROverlay();
		CalibrationTick(time);

		bool dashboardVisible = false;
		int width, height;
		glfwGetFramebufferSize(glfwWindow, &width, &height);
		const bool windowVisible = (width > 0 && height > 0);

		if (overlayMainHandle && vr::VROverlay())
		{
			auto &io = ImGui::GetIO();
			dashboardVisible = vr::VROverlay()->IsActiveDashboardOverlay(overlayMainHandle);

			static bool keyboardOpen = false, keyboardJustClosed = false;

			// After closing the keyboard, this code waits one frame for ImGui to pick up the new text from SetActiveText
			// before clearing the active widget. Then it waits another frame before allowing the keyboard to open again,
			// otherwise it will do so instantly since WantTextInput is still true on the second frame.
			if (keyboardJustClosed && keyboardOpen)
			{
				ImGui::ClearActiveID();
				keyboardOpen = false;
			}
			else if (keyboardJustClosed)
			{
				keyboardJustClosed = false;
			}
			else if (!io.WantTextInput)
			{
				// User might close the keyboard without hitting Done, so we unset the flag to allow it to open again.
				keyboardOpen = false;
			}
			else if (io.WantTextInput && !keyboardOpen && !keyboardJustClosed)
			{
				int id = ImGui::GetActiveID();
				auto textInfo = ImGui::GetInputTextState(id);

				if (textInfo != nullptr) {
					textBuf[0] = 0;
					int len = WideCharToMultiByte(CP_UTF8, 0, (LPCWCH)textInfo->TextA.Data, textInfo->TextA.Size, textBuf, sizeof(textBuf), nullptr, nullptr);
					textBuf[std::min(static_cast<size_t>(len), sizeof(textBuf) - 1)] = 0;

					uint32_t unFlags = 0; // EKeyboardFlags 

					vr::VROverlay()->ShowKeyboardForOverlay(
						overlayMainHandle, vr::k_EGamepadTextInputModeNormal, vr::k_EGamepadTextInputLineModeSingleLine,
						unFlags, "Space Calibrator Overlay", sizeof textBuf, textBuf, 0
					);
					keyboardOpen = true;
				}
			}

			vr::VREvent_t vrEvent;
			while (vr::VROverlay()->PollNextOverlayEvent(overlayMainHandle, &vrEvent, sizeof(vrEvent)))
			{
				switch (vrEvent.eventType) {
				case vr::VREvent_MouseMove:
					io.AddMousePosEvent(vrEvent.data.mouse.x, vrEvent.data.mouse.y);
					break;
				case vr::VREvent_MouseButtonDown:
					io.AddMouseButtonEvent((vrEvent.data.mouse.button & vr::VRMouseButton_Left) == vr::VRMouseButton_Left ? 0 : 1, true);
					break;
				case vr::VREvent_MouseButtonUp:
					io.AddMouseButtonEvent((vrEvent.data.mouse.button & vr::VRMouseButton_Left) == vr::VRMouseButton_Left ? 0 : 1, false);
					break;
				case vr::VREvent_ScrollDiscrete:
				{
					float x = vrEvent.data.scroll.xdelta * 360.0f * 8.0f;
					float y = vrEvent.data.scroll.ydelta * 360.0f * 8.0f;
					io.AddMouseWheelEvent(x, y);
					break;
				}
				case vr::VREvent_KeyboardDone: {
					vr::VROverlay()->GetKeyboardText(textBuf, sizeof textBuf);

					int id = ImGui::GetActiveID();
					auto textInfo = ImGui::GetInputTextState(id);
					int bufSize = MultiByteToWideChar(CP_UTF8, 0, textBuf, -1, nullptr, 0);
					textInfo->TextA.resize(bufSize);
					MultiByteToWideChar(CP_UTF8, 0, textBuf, -1, (LPWSTR)textInfo->TextA.Data, bufSize);
					textInfo->CurLenA = bufSize;
					textInfo->CurLenA = WideCharToMultiByte(CP_UTF8, 0, (LPCWCH)textInfo->TextA.Data, textInfo->TextA.Size, nullptr, 0, nullptr, nullptr);
					
					keyboardJustClosed = true;
					break;
				}
				case vr::VREvent_Quit:
					return;
				}
			}
		}
		
		if (windowVisible || dashboardVisible)
		{
			auto &io = ImGui::GetIO();
			
			// These change state now, so we must execute these before doing our own modifications to the io state for VR
			ImGui_ImplOpenGL3_NewFrame();
			ImGui_ImplGlfw_NewFrame();

			io.DisplaySize = ImVec2((float)fboTextureWidth, (float)fboTextureHeight);
			io.DisplayFramebufferScale = ImVec2(1.0f, 1.0f);

			io.ConfigFlags = io.ConfigFlags & ~ImGuiConfigFlags_NoMouseCursorChange;
			if (dashboardVisible) {
				io.ConfigFlags = io.ConfigFlags | ImGuiConfigFlags_NoMouseCursorChange;
			}

			ImGui::NewFrame();

			BuildMainWindow(dashboardVisible);

			// Conflict popup -- the Steam-store version is installed in addition to
			// this GitHub fork.  Both register the same SteamVR overlay manifest
			// and fight over which binary launches when the user opens the
			// dashboard.  Recommend uninstalling the Steam version since this fork
			// is what's actively maintained.
			static bool steamPopupDismissed = false;

			if (s_isSteamVersionInstalled && !steamPopupDismissed) {
				ImGui::OpenPopup("Conflicting Space Calibrator install");
				if (ImGui::BeginPopupModal("Conflicting Space Calibrator install", 0, ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize)) {
					ImGui::TextWrapped(
						"You have both the Steam-store version and this GitHub fork of Space Calibrator installed.\n\n"
						"This fork is the actively-maintained version. We recommend uninstalling the Steam version "
						"so they don't fight over the same SteamVR overlay manifest.\n\n"
						"Open Windows Settings -> Apps to uninstall the Steam version? (SteamVR will need to be closed.)");

					ImGui::NewLine();

					float windowWidth = ImGui::GetWindowWidth();
					ImGui::SetCursorPosX(windowWidth / 11.0f);
					if (ImGui::Button("Open Windows Settings", ImVec2(windowWidth * 4.0f / 11.0f, 0))) {
						// Same Settings deep-link the original code used; users
						// uninstall Steam apps from Windows-Settings -> Apps too.
						UninstallGithubSpaceCalibrator();
						steamPopupDismissed = true;
					}
					ImGui::SameLine();
					ImGui::SetCursorPosX(windowWidth / 11.0f * 6.0f);
					if (ImGui::Button("Dismiss", ImVec2(windowWidth * 4.0f / 11.0f, 0))) {
						steamPopupDismissed = true;
					}

					ImGui::EndPopup();
				}
			}

			ImGui::Render();

			glBindFramebuffer(GL_FRAMEBUFFER, fboHandle);
			glViewport(0, 0, fboTextureWidth, fboTextureHeight);
			glClearColor(0, 0, 0, 1);
			glClear(GL_COLOR_BUFFER_BIT);

			ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

			glBindFramebuffer(GL_FRAMEBUFFER, 0);

			if (width && height)
			{
				glBindFramebuffer(GL_READ_FRAMEBUFFER, fboHandle);
				glBlitFramebuffer(0, 0, width, height, 0, 0, width, height, GL_COLOR_BUFFER_BIT, GL_NEAREST);
				glfwSwapBuffers(glfwWindow);
			}

			if (dashboardVisible)
			{
				vr::Texture_t vrTex = {
					.handle = (void*)
#if defined _WIN64 || defined _LP64
				(uint64_t)
#endif
						fboTextureHandle,
					.eType = vr::TextureType_OpenGL,
					.eColorSpace = vr::ColorSpace_Auto,
				};

				vr::HmdVector2_t mouseScale = { (float) fboTextureWidth, (float) fboTextureHeight };

				vr::VROverlay()->SetOverlayTexture(overlayMainHandle, &vrTex);
				vr::VROverlay()->SetOverlayMouseScale(overlayMainHandle, &mouseScale);
			}
		}

		const double dashboardInterval = 1.0 / 90.0; // fps
		double waitEventsTimeout = std::max(CalCtx.wantedUpdateInterval, dashboardInterval);

		if (dashboardVisible && waitEventsTimeout > dashboardInterval)
			waitEventsTimeout = dashboardInterval;

		if (immediateRedraw) {
			waitEventsTimeout = 0;
			immediateRedraw = false;
		}

		glfwWaitEventsTimeout(waitEventsTimeout);

		// If we're minimized rendering won't limit our frame rate so we need to do it ourselves.
		if (glfwGetWindowAttrib(glfwWindow, GLFW_ICONIFIED))
		{
			double targetFrameTime = 1 / MINIMIZED_MAX_FPS;
			double waitTime = targetFrameTime - (glfwGetTime() - lastFrameStartTime);
			if (waitTime > 0)
			{
				std::this_thread::sleep_for(std::chrono::duration<double>(waitTime));
			}
		
			lastFrameStartTime += targetFrameTime;
		}
	}
}

void VerifySetupCorrect() {
	// register the manifest so that it shows up in the overlays menu
	if (!vr::VRApplications()->IsApplicationInstalled(OPENVR_APPLICATION_KEY)) {
		std::string manifestPath = std::format("{}\\{}", cwd, "manifest.vrmanifest");
		std::cout << "Adding manifest path: " << manifestPath << std::endl;
		// If manifest is not installed, try installing it, and set it to auto-start with SteamVR
		auto vrAppErr = vr::VRApplications()->AddApplicationManifest(manifestPath.c_str());
		if (vrAppErr != vr::VRApplicationError_None) {
			fprintf(stderr, "Failed to add manifest: %s\n", vr::VRApplications()->GetApplicationsErrorNameFromEnum(vrAppErr));
		} else {
			vr::VRApplications()->SetApplicationAutoLaunch(OPENVR_APPLICATION_KEY, true);
		}
	} else {
		// Application is already registered, do not alter settings
		std::cout << "Space Calibrator already registered with SteamVR. Skipping..." << std::endl;
	}

	// try removing the legacy app manifest from Steam, otherwise people will have multiple entries in the overlays menu
	if (vr::VRApplications()->IsApplicationInstalled(LEGACY_OPENVR_APPLICATION_KEY)) {
		std::cout << "Found a legacy version of Space Calibrator..." << std::endl;
		// GitHub key is installed, uninstall it
		vr::EVRApplicationError appErr = vr::EVRApplicationError::VRApplicationError_None;
		char manifestPathBuffer[MAX_PATH + 32 /* for good measure */] = {};
		uint32_t szBufferSize = vr::VRApplications()->GetApplicationPropertyString(LEGACY_OPENVR_APPLICATION_KEY, vr::VRApplicationProperty_BinaryPath_String, manifestPathBuffer, sizeof(manifestPathBuffer), &appErr);
		if (appErr != vr::VRApplicationError_None) {
			std::cout << "Failed to get binary path of " << LEGACY_OPENVR_APPLICATION_KEY << std::endl;
			return;
		}
		
		// replace XXX.exe with manifest.vrmanifest
		const char* newFileName = "manifest.vrmanifest";
		char* lastSlash = strrchr(manifestPathBuffer, '\\');
		if (lastSlash) {
			*(lastSlash + 1) = '\0';
			// strcat_s instead of strcat — silences MSVC C4996 deprecation and
			// hard-fails (instead of buffer-overflowing) on the off chance
			// SteamVR ever returns a path that uses every byte of the buffer.
			strcat_s(manifestPathBuffer, sizeof manifestPathBuffer, newFileName);
		}

		appErr = vr::VRApplications()->RemoveApplicationManifest(manifestPathBuffer);
		if (appErr != vr::VRApplicationError_None) {
			std::cout << "Failed to remove legacy application manifest. You may have duplicate entries in the overlays list." << std::endl;
		}
	}
}

// Steam-store install detection.
//
// This fork is the canonical GitHub-distributed version. The previous logic
// here detected the GitHub install and asked the user to remove it -- correct
// when Steam was the canonical source (pre-fork), backwards now. We've flipped
// the check: detect a *Steam-store* install of the same app, and prompt the
// user to remove THAT one instead so it doesn't fight us over the SteamVR
// overlay manifest registration.
//
// Detection: SteamApp ID 3368750 is OpenVR Space Calibrator on the Steam
// store. If Steam has it installed, an `appmanifest_3368750.acf` file exists
// in one of Steam's library folders. We:
//   1. Find Steam's install path from the registry.
//   2. Check the default `<steam>/steamapps/` library.
//   3. Parse `<steam>/config/libraryfolders.vdf` for additional libraries.
// If the manifest is found anywhere, the Steam version is installed.
//
// This deliberately does NOT use SteamVR's IsApplicationInstalled() for the
// `steam.overlay.3368750` key -- both the Steam version and this fork register
// against that key, so the answer is always "yes installed" and useless for
// distinguishing.
namespace {

// Resolve Steam's install directory (e.g. C:\Program Files (x86)\Steam).
// Returns empty string when Steam isn't installed at all.
std::filesystem::path GetSteamInstallPath() {
	// 32-bit-as-WOW6432 (most common — Steam ships as a 32-bit installer).
	std::string p = GetRegistryString(HKEY_LOCAL_MACHINE,
		"SOFTWARE\\WOW6432Node\\Valve\\Steam", "InstallPath");
	if (p.empty()) {
		// Native 64-bit fallback.
		p = GetRegistryString(HKEY_LOCAL_MACHINE,
			"SOFTWARE\\Valve\\Steam", "InstallPath");
	}
	if (p.empty()) {
		// Per-user install (rare; Steam is usually system-wide).
		p = GetRegistryString(HKEY_CURRENT_USER, "SOFTWARE\\Valve\\Steam", "SteamPath");
	}
	if (p.empty()) return {};
	return std::filesystem::path(p);
}

// Read libraryfolders.vdf and yield every library path it lists. The VDF
// format is hierarchical key/value text; we don't need a full parser, just
// the paths. They appear as `"path"  "<C:\\Some\\Library>"` lines (the
// double-backslashes are how Steam escapes them on disk).
std::vector<std::filesystem::path> EnumerateSteamLibraries(const std::filesystem::path& steamRoot) {
	std::vector<std::filesystem::path> out;
	if (steamRoot.empty()) return out;

	// The default library is always `<steam>/steamapps/`.
	auto defaultLib = steamRoot / "steamapps";
	if (std::filesystem::exists(defaultLib)) out.push_back(defaultLib);

	// Newer Steam (post-2021) puts this under config/. Older/legacy installs
	// keep it next to steamapps/. Try both.
	std::filesystem::path candidates[] = {
		steamRoot / "config" / "libraryfolders.vdf",
		steamRoot / "steamapps" / "libraryfolders.vdf",
	};
	for (const auto& vdfPath : candidates) {
		if (!std::filesystem::exists(vdfPath)) continue;
		std::ifstream in(vdfPath);
		if (!in) continue;
		std::string line;
		while (std::getline(in, line)) {
			// Match `"path"  "<value>"` -- the VDF emits two tabs between
			// key and value but the exact spacing varies, so do a permissive
			// substring scan.
			auto keyPos = line.find("\"path\"");
			if (keyPos == std::string::npos) continue;
			auto firstQuote = line.find('"', keyPos + 6);
			if (firstQuote == std::string::npos) continue;
			auto secondQuote = line.find('"', firstQuote + 1);
			if (secondQuote == std::string::npos) continue;
			std::string raw = line.substr(firstQuote + 1, secondQuote - firstQuote - 1);
			// Steam writes "C:\\Some\\Library" with escaped backslashes. Normalise.
			std::string normalised;
			normalised.reserve(raw.size());
			for (size_t i = 0; i < raw.size(); ++i) {
				if (raw[i] == '\\' && i + 1 < raw.size() && raw[i + 1] == '\\') {
					normalised.push_back('\\');
					++i;
				} else {
					normalised.push_back(raw[i]);
				}
			}
			std::filesystem::path lib = std::filesystem::path(normalised) / "steamapps";
			if (std::filesystem::exists(lib)) out.push_back(std::move(lib));
		}
		break; // first successfully-opened VDF wins.
	}

	// De-duplicate (some Steam installs list the default library a second
	// time inside libraryfolders.vdf with index "0").
	std::sort(out.begin(), out.end());
	out.erase(std::unique(out.begin(), out.end()), out.end());
	return out;
}

constexpr const char* kSteamSpaceCalibratorAppId = "3368750";

} // namespace

// Returns true if a Steam-store install of Space Calibrator (App ID 3368750)
// is detected on the system. Populates outManifestPath with the manifest's
// path on success so the popup can show the user *where* the conflicting
// install lives.
bool CheckSteamVersionInstalled(std::string& outManifestPath) {
	auto steamRoot = GetSteamInstallPath();
	if (steamRoot.empty()) return false;
	auto libraries = EnumerateSteamLibraries(steamRoot);
	const std::string manifestName = std::string("appmanifest_") + kSteamSpaceCalibratorAppId + ".acf";
	for (const auto& lib : libraries) {
		auto manifest = lib / manifestName;
		if (std::filesystem::exists(manifest)) {
			outManifestPath = manifest.string();
			return true;
		}
	}
	return false;
}

int APIENTRY wWinMain(_In_ HINSTANCE hInstance, _In_opt_ HINSTANCE hPrevInstance, _In_ LPWSTR lpCmdLine, _In_ int nCmdShow)
{
	// Anchor cwd to the EXE's directory rather than wherever the user
	// launched us from. The manifest path, taskbar icon, and overlay
	// thumbnail are loaded relative to cwd; if the user double-clicks the
	// exe from File Explorer cwd is right, but if they launch it from a
	// different working directory (a shortcut with no Start In, an IDE run,
	// `dist/SpaceCalibrator.exe` from a parent shell, etc.) the icon
	// silently fails to load and the registered manifest path is wrong.
	// GetModuleFileName + strrchr to the last separator is the canonical
	// "directory containing this exe" pattern on Windows.
	{
		char modulePath[MAX_PATH] = { 0 };
		DWORD modLen = GetModuleFileNameA(nullptr, modulePath, MAX_PATH);
		if (modLen > 0 && modLen < MAX_PATH) {
			char* lastSep = strrchr(modulePath, '\\');
			if (lastSep) {
				*lastSep = '\0';
				strncpy_s(cwd, modulePath, MAX_PATH);
				SetCurrentDirectoryA(cwd);
			}
		}
		// If GetModuleFileName failed (extremely rare), fall back to the
		// shell's idea of cwd so we at least try _something_.
		if (cwd[0] == '\0') {
			if (_getcwd(cwd, MAX_PATH) == nullptr) {
				// @TODO: Handle Invalid working dir case. Should never happen.
			}
		}
	}
	HandleCommandLine(lpCmdLine);

#ifdef DEBUG_LOGS
	CreateConsole();
#endif

	if (!glfwInit())
	{
		MessageBox(nullptr, L"Failed to initialize GLFW", L"", 0);
		return 0;
	}

	glfwSetErrorCallback(GLFWErrorCallback);

	bool isRunningViaSteam = false;
	char steamAppId[256] = {};
	DWORD result = GetEnvironmentVariableA("SteamAppId", steamAppId, sizeof(steamAppId));
	if (result > 0 && steamAppId != nullptr) {
		if (c_SPACE_CALIBRATOR_STEAM_APP_ID == steamAppId ||
			c_STEAMVR_STEAM_APP_ID == steamAppId) {
			// We got launched via the Steam client UI.
			hSteamMutex = CreateMutexA(NULL, FALSE, STEAM_MUTEX_KEY);
			isRunningViaSteam = true;
			if (hSteamMutex == nullptr) {
				hSteamMutex = INVALID_HANDLE_VALUE;
			} else {
				// mutex opened, check if we opened one, if so exit
				if (GetLastError() == ERROR_ALREADY_EXISTS) {
					CloseHandle(hSteamMutex);
					hSteamMutex = INVALID_HANDLE_VALUE;
					return 0;
				}
			}
		}
	}

	try {
		printf("isSteam: %d\n", isRunningViaSteam);

		// Always check for a Steam-store version, regardless of how this fork
		// was launched. The previous logic only checked when running under
		// SteamVR — but the conflict exists either way (both register the same
		// SteamVR overlay manifest), so users who launch via the start menu
		// should also see the warning.
		s_isSteamVersionInstalled = CheckSteamVersionInstalled(s_steamVersionPath);
		printf("foundSteamVersion: %d (manifest: %s)\n",
			s_isSteamVersionInstalled,
			s_steamVersionPath.c_str());

		// Bring up the window + ImGui + the user's saved profile FIRST, before
		// touching the VR stack. With this ordering, launching the program
		// while SteamVR is closed shows the calibration UI immediately --
		// settings tabs / logs / advanced are all browsable while we retry the
		// VR connection in the background. The retry happens inside RunLoop.
		CreateGLFWWindow();
		LoadProfile(CalCtx);

		// First connection attempt at startup. If it succeeds, we go straight
		// into a fully-working state. If not (SteamVR isn't running yet, etc.),
		// RunLoop will retry once per second until it lands -- the user sees
		// the "Waiting for SteamVR" banner in the meantime.
		TryInitVRStack();

		RunLoop();

		vr::VR_Shutdown();

		if (fboHandle)
			glDeleteFramebuffers(1, &fboHandle);

		if (fboTextureHandle)
			glDeleteTextures(1, &fboTextureHandle);

		ImGui_ImplOpenGL3_Shutdown();
		ImGui_ImplGlfw_Shutdown();
		ImPlot::DestroyContext();
		ImGui::DestroyContext();
	}
	catch (std::runtime_error &e)
	{
		std::cerr << "Runtime error: " << e.what() << std::endl;
		wchar_t message[1024];
		swprintf(message, 1024, L"%hs", e.what());
		MessageBox(nullptr, message, L"Runtime Error", 0);
	}

	if (hSteamMutex != INVALID_HANDLE_VALUE && hSteamMutex != nullptr) {
		CloseHandle(hSteamMutex);
		hSteamMutex = nullptr;
	}

	if (glfwWindow)
		glfwDestroyWindow(glfwWindow);

	glfwTerminate();
	return 0;
}

static void HandleCommandLine(LPWSTR lpCmdLine)
{
	if (lstrcmp(lpCmdLine, L"-openvrpath") == 0)
	{
		auto vrErr = vr::VRInitError_None;
		vr::VR_Init(&vrErr, vr::VRApplication_Utility);
		if (vrErr == vr::VRInitError_None)
		{
			char cruntimePath[MAX_PATH] = { 0 };
			unsigned int pathLen;
			vr::VR_GetRuntimePath(cruntimePath, MAX_PATH, &pathLen);

			printf("%s", cruntimePath);
			vr::VR_Shutdown();
			exit(0);
		}
		fprintf(stderr, "Failed to initialize OpenVR: %s\n", vr::VR_GetVRInitErrorAsEnglishDescription(vrErr));
		vr::VR_Shutdown();
		exit(-2);
	}
	else if (lstrcmp(lpCmdLine, L"-installmanifest") == 0)
	{
		auto vrErr = vr::VRInitError_None;
		vr::VR_Init(&vrErr, vr::VRApplication_Utility);
		if (vrErr == vr::VRInitError_None)
		{
			if (vr::VRApplications()->IsApplicationInstalled(OPENVR_APPLICATION_KEY))
			{
				char oldWd[MAX_PATH] = { 0 };
				auto vrAppErr = vr::VRApplicationError_None;
				vr::VRApplications()->GetApplicationPropertyString(OPENVR_APPLICATION_KEY, vr::VRApplicationProperty_WorkingDirectory_String, oldWd, MAX_PATH, &vrAppErr);
				if (vrAppErr != vr::VRApplicationError_None)
				{
					fprintf(stderr, "Failed to get old working dir, skipping removal: %s\n", vr::VRApplications()->GetApplicationsErrorNameFromEnum(vrAppErr));
				}
				else
				{
					std::string manifestPath = oldWd;
					manifestPath += "\\manifest.vrmanifest";
					std::cout << "Removing old manifest path: " << manifestPath << std::endl;
					vr::VRApplications()->RemoveApplicationManifest(manifestPath.c_str());
				}
			}
			std::string manifestPath = cwd;
			manifestPath += "\\manifest.vrmanifest";
			std::cout << "Adding manifest path: " << manifestPath << std::endl;
			auto vrAppErr = vr::VRApplications()->AddApplicationManifest(manifestPath.c_str());
			if (vrAppErr != vr::VRApplicationError_None)
			{
				fprintf(stderr, "Failed to add manifest: %s\n", vr::VRApplications()->GetApplicationsErrorNameFromEnum(vrAppErr));
			}
			else
			{
				vr::VRApplications()->SetApplicationAutoLaunch(OPENVR_APPLICATION_KEY, true);
			}
			vr::VR_Shutdown();
			exit(-2);
		}
		fprintf(stderr, "Failed to initialize OpenVR: %s\n", vr::VR_GetVRInitErrorAsEnglishDescription(vrErr));
		vr::VR_Shutdown();
		exit(-2);
	}
	else if (lstrcmp(lpCmdLine, L"-removemanifest") == 0)
	{
		auto vrErr = vr::VRInitError_None;
		vr::VR_Init(&vrErr, vr::VRApplication_Utility);
		if (vrErr == vr::VRInitError_None)
		{
			if (vr::VRApplications()->IsApplicationInstalled(OPENVR_APPLICATION_KEY))
			{
				std::string manifestPath = cwd;
				manifestPath += "\\manifest.vrmanifest";
				std::cout << "Removing manifest path: " << manifestPath << std::endl;
				vr::VRApplications()->RemoveApplicationManifest(manifestPath.c_str());
			}
			vr::VR_Shutdown();
			exit(0);
		}
		fprintf(stderr, "Failed to initialize OpenVR: %s\n", vr::VR_GetVRInitErrorAsEnglishDescription(vrErr));
		vr::VR_Shutdown();
		exit(-2);
	}
	else if (lstrcmp(lpCmdLine, L"-activatemultipledrivers") == 0)
	{
		int ret = -2;
		auto vrErr = vr::VRInitError_None;
		vr::VR_Init(&vrErr, vr::VRApplication_Utility);
		if (vrErr == vr::VRInitError_None)
		{
			try
			{
				ActivateMultipleDrivers();
				ret = 0;
			}
			catch (std::runtime_error &e)
			{
				std::cerr << e.what() << std::endl;
			}
		}
		else
		{
			fprintf(stderr, "Failed to initialize OpenVR: %s\n", vr::VR_GetVRInitErrorAsEnglishDescription(vrErr));
		}
		vr::VR_Shutdown();
		exit(ret);
	}
#ifndef DEBUG_LOGS
	else if (lstrcmp(lpCmdLine, L"-console") == 0) {
		CreateConsole();
	}
#endif
}