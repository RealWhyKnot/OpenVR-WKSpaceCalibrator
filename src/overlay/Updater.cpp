#include "stdafx.h"
#include "Updater.h"

#include <windows.h>
#include <winhttp.h>
#include <bcrypt.h>
#include <shellapi.h>

#pragma comment(lib, "winhttp.lib")
#pragma comment(lib, "bcrypt.lib")

#include <array>
#include <cwctype>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <vector>

#ifndef NT_SUCCESS
#define NT_SUCCESS(s) (((NTSTATUS)(s)) >= 0)
#endif

namespace spacecal::updates {

namespace {

constexpr wchar_t kUserAgent[] = L"OpenVR-WKSpaceCalibrator-Updater/1.0";

std::wstring Utf8ToWide(const std::string& utf8) {
	if (utf8.empty()) return {};
	int n = MultiByteToWideChar(CP_UTF8, 0, utf8.data(), (int)utf8.size(), nullptr, 0);
	std::wstring out(n, L'\0');
	MultiByteToWideChar(CP_UTF8, 0, utf8.data(), (int)utf8.size(), out.data(), n);
	return out;
}

std::string WideToUtf8(const std::wstring& w) {
	if (w.empty()) return {};
	int n = WideCharToMultiByte(CP_UTF8, 0, w.data(), (int)w.size(), nullptr, 0, nullptr, nullptr);
	std::string out(n, '\0');
	WideCharToMultiByte(CP_UTF8, 0, w.data(), (int)w.size(), out.data(), n, nullptr, nullptr);
	return out;
}

// Best-effort GetLastError -> human string.
std::string LastErrorString(DWORD code) {
	wchar_t* buf = nullptr;
	FormatMessageW(
		FORMAT_MESSAGE_ALLOCATE_BUFFER | FORMAT_MESSAGE_FROM_SYSTEM | FORMAT_MESSAGE_IGNORE_INSERTS,
		nullptr, code, 0, (LPWSTR)&buf, 0, nullptr);
	std::string out;
	if (buf) {
		out = WideToUtf8(buf);
		LocalFree(buf);
		// Strip trailing CRLF that FormatMessage tacks on.
		while (!out.empty() && (out.back() == '\n' || out.back() == '\r' || out.back() == ' '))
			out.pop_back();
	}
	if (out.empty()) out = "error code " + std::to_string(code);
	return out;
}

// Take a URL like "https://example.com/path/file.exe" and split into
// host, path, and the secure-port flag. WinHttp wants these separately.
struct ParsedUrl {
	std::wstring scheme;
	std::wstring host;
	std::wstring path;       // includes leading "/"; "/" if none.
	INTERNET_PORT port = 0;
	bool https = true;
};

bool ParseUrl(const std::wstring& url, ParsedUrl& out) {
	URL_COMPONENTSW uc{};
	uc.dwStructSize = sizeof(uc);
	wchar_t scheme[16], host[256], path[2048];
	uc.lpszScheme = scheme;     uc.dwSchemeLength = _countof(scheme);
	uc.lpszHostName = host;     uc.dwHostNameLength = _countof(host);
	uc.lpszUrlPath = path;      uc.dwUrlPathLength = _countof(path);
	if (!WinHttpCrackUrl(url.c_str(), (DWORD)url.size(), 0, &uc)) return false;

	out.scheme.assign(scheme, uc.dwSchemeLength);
	out.host.assign(host, uc.dwHostNameLength);
	out.path.assign(path, uc.dwUrlPathLength);
	if (out.path.empty()) out.path = L"/";
	out.https = (uc.nScheme == INTERNET_SCHEME_HTTPS);
	out.port = uc.nPort;
	return true;
}

// Lowercase a hex string in place.
std::string LowerHex(std::string s) {
	for (char& c : s) {
		if (c >= 'A' && c <= 'F') c = (char)(c + ('a' - 'A'));
	}
	return s;
}

// SHA-256 a file via BCrypt. Returns lowercase hex on success, empty on
// failure (with `err` populated). 64 KiB chunks; sufficient even for fairly
// large installers since this runs on a worker thread.
std::string Sha256File(const std::wstring& path, std::string& err) {
	BCRYPT_ALG_HANDLE alg = nullptr;
	BCRYPT_HASH_HANDLE hash = nullptr;
	std::vector<uint8_t> hashObj;
	std::vector<uint8_t> digest;
	std::string out;

	auto cleanup = [&](void) {
		if (hash) BCryptDestroyHash(hash);
		if (alg) BCryptCloseAlgorithmProvider(alg, 0);
	};

	NTSTATUS s = BCryptOpenAlgorithmProvider(&alg, BCRYPT_SHA256_ALGORITHM, nullptr, 0);
	if (!NT_SUCCESS(s)) { err = "BCryptOpenAlgorithmProvider failed"; cleanup(); return {}; }

	DWORD objLen = 0, hashLen = 0, cbResult = 0;
	if (!NT_SUCCESS(BCryptGetProperty(alg, BCRYPT_OBJECT_LENGTH, (PUCHAR)&objLen, sizeof(objLen), &cbResult, 0)) ||
		!NT_SUCCESS(BCryptGetProperty(alg, BCRYPT_HASH_LENGTH, (PUCHAR)&hashLen, sizeof(hashLen), &cbResult, 0))) {
		err = "BCryptGetProperty failed"; cleanup(); return {};
	}
	hashObj.resize(objLen);
	digest.resize(hashLen);

	if (!NT_SUCCESS(BCryptCreateHash(alg, &hash, hashObj.data(), objLen, nullptr, 0, 0))) {
		err = "BCryptCreateHash failed"; cleanup(); return {};
	}

	std::ifstream f(path, std::ios::binary);
	if (!f) { err = "open hash input failed"; cleanup(); return {}; }
	std::vector<char> buf(64 * 1024);
	while (f) {
		f.read(buf.data(), (std::streamsize)buf.size());
		std::streamsize n = f.gcount();
		if (n <= 0) break;
		if (!NT_SUCCESS(BCryptHashData(hash, (PUCHAR)buf.data(), (ULONG)n, 0))) {
			err = "BCryptHashData failed"; cleanup(); return {};
		}
	}

	if (!NT_SUCCESS(BCryptFinishHash(hash, digest.data(), (ULONG)digest.size(), 0))) {
		err = "BCryptFinishHash failed"; cleanup(); return {};
	}

	cleanup();

	static const char hex[] = "0123456789abcdef";
	out.resize(digest.size() * 2);
	for (size_t i = 0; i < digest.size(); ++i) {
		out[i * 2 + 0] = hex[(digest[i] >> 4) & 0xF];
		out[i * 2 + 1] = hex[(digest[i] >> 0) & 0xF];
	}
	return out;
}

// Drop the Mark-of-the-Web alternate data stream (`<file>:Zone.Identifier`)
// that WinHttp/IE attach to downloaded files. SmartScreen uses it to decide
// whether to block; deleting it is the official, documented way for an
// auto-updater to skip the warning when the binary is already trusted (we
// just verified its SHA-256). DeleteFileW on the named ADS is the right API.
void StripMarkOfTheWeb(const std::wstring& path) {
	std::wstring zone = path + L":Zone.Identifier";
	DeleteFileW(zone.c_str()); // ignore result; the ADS may not exist.
}

// Find a writable directory for the download. Prefer GetTempPath; if that
// somehow fails (very rare) fall back to the EXE directory.
std::wstring GetDownloadDir() {
	wchar_t buf[MAX_PATH + 1] = {};
	DWORD n = GetTempPathW(MAX_PATH, buf);
	if (n > 0 && n < MAX_PATH) return std::wstring(buf, n);

	wchar_t exe[MAX_PATH + 1] = {};
	if (GetModuleFileNameW(nullptr, exe, MAX_PATH) > 0) {
		std::wstring p = exe;
		auto slash = p.find_last_of(L"\\/");
		if (slash != std::wstring::npos) return p.substr(0, slash + 1);
	}
	return L".\\";
}

// HTTP download with redirect support and progress reporting. WinHttp follows
// 3xx by default but only for the same server; cross-host redirects (which is
// what GitHub's `browser_download_url` will do, going off to objects.githubusercontent.com)
// require WINHTTP_OPTION_REDIRECT_POLICY = WINHTTP_OPTION_REDIRECT_POLICY_ALWAYS.
bool DownloadFile(const std::wstring& url,
	const std::wstring& outPath,
	std::atomic<uint64_t>& bytesReceived,
	std::atomic<uint64_t>& bytesTotal,
	std::string& err)
{
	ParsedUrl p;
	if (!ParseUrl(url, p)) { err = "Failed to parse URL"; return false; }

	HINTERNET hSession = WinHttpOpen(kUserAgent, WINHTTP_ACCESS_TYPE_DEFAULT_PROXY,
		WINHTTP_NO_PROXY_NAME, WINHTTP_NO_PROXY_BYPASS, 0);
	if (!hSession) { err = "WinHttpOpen: " + LastErrorString(GetLastError()); return false; }

	DWORD redirectPolicy = WINHTTP_OPTION_REDIRECT_POLICY_ALWAYS;
	WinHttpSetOption(hSession, WINHTTP_OPTION_REDIRECT_POLICY, &redirectPolicy, sizeof(redirectPolicy));

	HINTERNET hConnect = WinHttpConnect(hSession, p.host.c_str(), p.port, 0);
	if (!hConnect) {
		err = "WinHttpConnect: " + LastErrorString(GetLastError());
		WinHttpCloseHandle(hSession);
		return false;
	}

	DWORD flags = p.https ? WINHTTP_FLAG_SECURE : 0;
	HINTERNET hRequest = WinHttpOpenRequest(hConnect, L"GET", p.path.c_str(), nullptr,
		WINHTTP_NO_REFERER, WINHTTP_DEFAULT_ACCEPT_TYPES, flags);
	if (!hRequest) {
		err = "WinHttpOpenRequest: " + LastErrorString(GetLastError());
		WinHttpCloseHandle(hConnect);
		WinHttpCloseHandle(hSession);
		return false;
	}

	if (!WinHttpSendRequest(hRequest, WINHTTP_NO_ADDITIONAL_HEADERS, 0,
			WINHTTP_NO_REQUEST_DATA, 0, 0, 0)
		|| !WinHttpReceiveResponse(hRequest, nullptr))
	{
		err = "WinHttp request failed: " + LastErrorString(GetLastError());
		WinHttpCloseHandle(hRequest);
		WinHttpCloseHandle(hConnect);
		WinHttpCloseHandle(hSession);
		return false;
	}

	DWORD status = 0, statusSize = sizeof status;
	WinHttpQueryHeaders(hRequest,
		WINHTTP_QUERY_STATUS_CODE | WINHTTP_QUERY_FLAG_NUMBER,
		WINHTTP_HEADER_NAME_BY_INDEX, &status, &statusSize, WINHTTP_NO_HEADER_INDEX);
	if (status != 200) {
		err = "Download HTTP " + std::to_string(status);
		WinHttpCloseHandle(hRequest);
		WinHttpCloseHandle(hConnect);
		WinHttpCloseHandle(hSession);
		return false;
	}

	uint64_t total = 0;
	DWORD lenSize = sizeof(uint64_t);
	uint64_t lenVal = 0;
	if (WinHttpQueryHeaders(hRequest,
		WINHTTP_QUERY_CONTENT_LENGTH | WINHTTP_QUERY_FLAG_NUMBER64,
		WINHTTP_HEADER_NAME_BY_INDEX, &lenVal, &lenSize, WINHTTP_NO_HEADER_INDEX)) {
		total = lenVal;
	}
	bytesTotal.store(total);

	HANDLE hFile = CreateFileW(outPath.c_str(), GENERIC_WRITE, 0, nullptr,
		CREATE_ALWAYS, FILE_ATTRIBUTE_NORMAL, nullptr);
	if (hFile == INVALID_HANDLE_VALUE) {
		err = "CreateFile: " + LastErrorString(GetLastError());
		WinHttpCloseHandle(hRequest);
		WinHttpCloseHandle(hConnect);
		WinHttpCloseHandle(hSession);
		return false;
	}

	bool ok = true;
	std::vector<char> chunk;
	uint64_t got = 0;
	for (;;) {
		DWORD avail = 0;
		if (!WinHttpQueryDataAvailable(hRequest, &avail)) {
			err = "WinHttpQueryDataAvailable: " + LastErrorString(GetLastError());
			ok = false; break;
		}
		if (avail == 0) break;
		if (chunk.size() < avail) chunk.resize(avail);
		DWORD read = 0;
		if (!WinHttpReadData(hRequest, chunk.data(), avail, &read)) {
			err = "WinHttpReadData: " + LastErrorString(GetLastError());
			ok = false; break;
		}
		if (read == 0) break;
		DWORD written = 0;
		if (!WriteFile(hFile, chunk.data(), read, &written, nullptr) || written != read) {
			err = "WriteFile: " + LastErrorString(GetLastError());
			ok = false; break;
		}
		got += read;
		bytesReceived.store(got);
	}

	CloseHandle(hFile);
	WinHttpCloseHandle(hRequest);
	WinHttpCloseHandle(hConnect);
	WinHttpCloseHandle(hSession);

	if (!ok) {
		DeleteFileW(outPath.c_str()); // partial download is useless
		return false;
	}

	// If the server didn't send Content-Length, fix up the totals so the UI
	// shows 100% rather than an indeterminate bar after we're done.
	if (total == 0) bytesTotal.store(got);
	return true;
}

// Launch the installer. Try unelevated first (it'll get a UAC prompt anyway
// because the NSIS installer is marked `RequestExecutionLevel admin`), and
// only fall back to an explicit `runas` verb if that fails — typically on
// SmartScreen blocks where ShellExecute fails with ERROR_CANCELLED before
// the manifest's auto-elevation kicks in.
//
// We don't wait for the installer to finish; we hand off and exit so the
// installer can replace our running EXE.
bool LaunchInstaller(const std::wstring& path, std::string& err) {
	auto tryLaunch = [&](LPCWSTR verb) -> DWORD {
		SHELLEXECUTEINFOW info{};
		info.cbSize = sizeof(info);
		info.fMask = SEE_MASK_NOCLOSEPROCESS | SEE_MASK_FLAG_NO_UI;
		info.lpVerb = verb;
		info.lpFile = path.c_str();
		info.nShow = SW_SHOWNORMAL;
		if (ShellExecuteExW(&info)) {
			if (info.hProcess) CloseHandle(info.hProcess);
			return ERROR_SUCCESS;
		}
		return GetLastError();
	};

	DWORD e = tryLaunch(L"open");
	if (e == ERROR_SUCCESS) return true;

	// Fallback: explicit elevation. SmartScreen / AppLocker / a missing
	// manifest auto-elevate would all cause the unelevated launch to fail
	// with ERROR_CANCELLED (1223) or ERROR_ACCESS_DENIED (5). `runas` re-
	// triggers the UAC consent flow with a fresh elevation request that
	// tends to bypass the SmartScreen warning when the file has had its
	// MOTW stripped.
	DWORD e2 = tryLaunch(L"runas");
	if (e2 == ERROR_SUCCESS) return true;

	err = "Installer launch failed: " + LastErrorString(e2);
	return false;
}

} // namespace

Updater::~Updater() {
	if (m_worker.joinable()) m_worker.join();
}

void Updater::DownloadAndLaunch(const std::string& url,
	const std::string& expectedSha256,
	const std::string& filename)
{
	DownloadState st = m_state.load();
	// Idempotent: ignore if already in flight or already done.
	if (st == DownloadState::Downloading || st == DownloadState::Verifying ||
		st == DownloadState::Launching || st == DownloadState::Done) {
		return;
	}
	if (m_worker.joinable()) m_worker.join();

	{
		std::lock_guard<std::mutex> lock(m_mutex);
		m_errorMessage.clear();
		m_downloadedPath.clear();
	}
	m_received.store(0);
	m_total.store(0);
	m_state.store(DownloadState::Downloading);

	m_worker = std::thread([this, url, expectedSha256, filename]() {
		Run(url, expectedSha256, filename);
	});
}

DownloadProgress Updater::GetProgress() const {
	DownloadProgress p;
	p.state = m_state.load();
	p.bytesReceived = m_received.load();
	p.bytesTotal = m_total.load();
	{
		std::lock_guard<std::mutex> lock(m_mutex);
		p.errorMessage = m_errorMessage;
		p.downloadedPath = m_downloadedPath;
	}
	return p;
}

void Updater::Run(std::string url, std::string expectedSha256, std::string filename) {
	auto fail = [&](const std::string& msg) {
		std::lock_guard<std::mutex> lock(m_mutex);
		m_errorMessage = msg;
		m_state.store(DownloadState::Failed);
	};

	if (filename.empty()) filename = "OpenVR-WKSpaceCalibrator-Setup.exe";
	// Reject any path-traversal in the asset name; we control the source
	// (RealWhyKnot fork release manifest) but defense in depth costs nothing.
	for (char c : filename) {
		if (c == '/' || c == '\\' || c == ':' || c == '\0') {
			fail("Refusing installer name containing path separator: " + filename);
			return;
		}
	}

	std::wstring dir = GetDownloadDir();
	std::wstring outPath = dir + Utf8ToWide(filename);
	std::wstring wurl = Utf8ToWide(url);

	std::string err;
	if (!DownloadFile(wurl, outPath, m_received, m_total, err)) {
		fail("Download failed: " + err);
		return;
	}

	if (!expectedSha256.empty()) {
		m_state.store(DownloadState::Verifying);
		std::string actual = Sha256File(outPath, err);
		if (actual.empty()) {
			DeleteFileW(outPath.c_str());
			fail("Hash failed: " + err);
			return;
		}
		std::string expected = LowerHex(expectedSha256);
		actual = LowerHex(actual);
		if (actual != expected) {
			DeleteFileW(outPath.c_str());
			fail("SHA-256 mismatch.\nExpected: " + expected + "\nGot:      " + actual);
			return;
		}
	}

	StripMarkOfTheWeb(outPath);

	m_state.store(DownloadState::Launching);
	if (!LaunchInstaller(outPath, err)) {
		fail(err);
		return;
	}

	{
		std::lock_guard<std::mutex> lock(m_mutex);
		m_downloadedPath = WideToUtf8(outPath);
	}
	m_state.store(DownloadState::Done);
}

} // namespace spacecal::updates
