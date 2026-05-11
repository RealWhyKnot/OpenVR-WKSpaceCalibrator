#include "stdafx.h"
#include "UpdateChecker.h"
#include "BuildStamp.h"

#include <picojson.h>
#include <windows.h>
#include <winhttp.h>

#pragma comment(lib, "winhttp.lib")

#include <sstream>
#include <vector>

namespace spacecal::updates {

namespace {

// GitHub API endpoint for the latest release of the WhyKnot fork. Hard-coded;
// changing the source-of-truth repository requires a recompile (intentional —
// you don't want a runtime-mutable update URL).
constexpr wchar_t kHost[] = L"api.github.com";
constexpr wchar_t kPath[] = L"/repos/RealWhyKnot/OpenVR-WKSpaceCalibrator/releases/latest";
constexpr wchar_t kUserAgent[] = L"OpenVR-WKSpaceCalibrator-Updater/1.0";

// Parse a YYYY.M.D.N(-XXXX)? tag (with optional leading "v") into four numeric
// components. Returns false on malformed input. The trailing -XXXX hex suffix
// (dev builds) is ignored for comparison purposes.
bool ParseStamp(const std::string& tag, int (&out)[4]) {
	std::string s = tag;
	if (!s.empty() && s[0] == 'v') s.erase(0, 1);
	auto dash = s.find('-');
	if (dash != std::string::npos) s.resize(dash);
	int parsed[4] = { 0, 0, 0, 0 };
	int idx = 0;
	const char* p = s.c_str();
	while (idx < 4 && *p) {
		char* end = nullptr;
		long v = std::strtol(p, &end, 10);
		if (end == p) return false;
		parsed[idx++] = static_cast<int>(v);
		p = end;
		if (*p == '.') p++;
		else if (*p != '\0') return false;
	}
	if (idx != 4) return false;
	for (int i = 0; i < 4; i++) out[i] = parsed[i];
	return true;
}

bool IsRemoteNewer(const std::string& remote, const std::string& local) {
	int r[4] = { 0 }, l[4] = { 0 };
	if (!ParseStamp(remote, r) || !ParseStamp(local, l)) return false;
	for (int i = 0; i < 4; i++) {
		if (r[i] > l[i]) return true;
		if (r[i] < l[i]) return false;
	}
	return false;
}

std::wstring Utf8ToWide(const std::string& utf8) {
	if (utf8.empty()) return {};
	int n = MultiByteToWideChar(CP_UTF8, 0, utf8.data(), (int)utf8.size(), nullptr, 0);
	std::wstring out(n, L'\0');
	MultiByteToWideChar(CP_UTF8, 0, utf8.data(), (int)utf8.size(), out.data(), n);
	return out;
}

std::string LastErrorString(DWORD code) {
	wchar_t* buf = nullptr;
	FormatMessageW(
		FORMAT_MESSAGE_ALLOCATE_BUFFER | FORMAT_MESSAGE_FROM_SYSTEM | FORMAT_MESSAGE_IGNORE_INSERTS,
		nullptr, code, 0, (LPWSTR)&buf, 0, nullptr);
	std::string out;
	if (buf) {
		int n = WideCharToMultiByte(CP_UTF8, 0, buf, -1, nullptr, 0, nullptr, nullptr);
		if (n > 1) {
			out.resize(n - 1);
			WideCharToMultiByte(CP_UTF8, 0, buf, -1, out.data(), n, nullptr, nullptr);
		}
		LocalFree(buf);
	}
	if (out.empty()) out = "error code " + std::to_string(code);
	return out;
}

// HTTP GET via WinHttp. Returns the body as a string on success; sets `err` and
// returns empty on failure. Follows redirects (default behaviour).
std::string HttpGet(const wchar_t* host, const wchar_t* path, std::string& err) {
	HINTERNET hSession = WinHttpOpen(kUserAgent, WINHTTP_ACCESS_TYPE_DEFAULT_PROXY,
		WINHTTP_NO_PROXY_NAME, WINHTTP_NO_PROXY_BYPASS, 0);
	if (!hSession) { err = "WinHttpOpen failed: " + LastErrorString(GetLastError()); return {}; }

	HINTERNET hConnect = WinHttpConnect(hSession, host, INTERNET_DEFAULT_HTTPS_PORT, 0);
	if (!hConnect) {
		err = "WinHttpConnect failed: " + LastErrorString(GetLastError());
		WinHttpCloseHandle(hSession);
		return {};
	}

	HINTERNET hRequest = WinHttpOpenRequest(hConnect, L"GET", path, nullptr,
		WINHTTP_NO_REFERER, WINHTTP_DEFAULT_ACCEPT_TYPES, WINHTTP_FLAG_SECURE);
	if (!hRequest) {
		err = "WinHttpOpenRequest failed: " + LastErrorString(GetLastError());
		WinHttpCloseHandle(hConnect);
		WinHttpCloseHandle(hSession);
		return {};
	}

	// GitHub API requires the Accept header. Without it you get a permanent
	// redirect to a different content negotiation that doesn't preserve the JSON.
	const wchar_t kHeaders[] = L"Accept: application/vnd.github+json\r\nX-GitHub-Api-Version: 2022-11-28\r\n";
	if (!WinHttpSendRequest(hRequest, kHeaders, (DWORD)wcslen(kHeaders),
		WINHTTP_NO_REQUEST_DATA, 0, 0, 0)
		|| !WinHttpReceiveResponse(hRequest, nullptr))
	{
		err = "WinHttp request failed: " + LastErrorString(GetLastError());
		WinHttpCloseHandle(hRequest);
		WinHttpCloseHandle(hConnect);
		WinHttpCloseHandle(hSession);
		return {};
	}

	DWORD status = 0, statusSize = sizeof status;
	WinHttpQueryHeaders(hRequest,
		WINHTTP_QUERY_STATUS_CODE | WINHTTP_QUERY_FLAG_NUMBER,
		WINHTTP_HEADER_NAME_BY_INDEX, &status, &statusSize, WINHTTP_NO_HEADER_INDEX);
	if (status != 200) {
		err = "GitHub API returned HTTP " + std::to_string(status);
		WinHttpCloseHandle(hRequest);
		WinHttpCloseHandle(hConnect);
		WinHttpCloseHandle(hSession);
		return {};
	}

	std::string body;
	for (;;) {
		DWORD avail = 0;
		if (!WinHttpQueryDataAvailable(hRequest, &avail)) {
			err = "WinHttpQueryDataAvailable failed: " + LastErrorString(GetLastError());
			body.clear();
			break;
		}
		if (avail == 0) break;
		std::vector<char> chunk(avail);
		DWORD read = 0;
		if (!WinHttpReadData(hRequest, chunk.data(), avail, &read)) {
			err = "WinHttpReadData failed: " + LastErrorString(GetLastError());
			body.clear();
			break;
		}
		body.append(chunk.data(), read);
	}

	WinHttpCloseHandle(hRequest);
	WinHttpCloseHandle(hConnect);
	WinHttpCloseHandle(hSession);
	return body;
}

} // namespace

UpdateChecker::~UpdateChecker() {
	if (m_worker.joinable()) m_worker.join();
}

void UpdateChecker::CheckAsync() {
	State expected = State::Idle;
	if (!m_state.compare_exchange_strong(expected, State::Checking)) {
		// Already checking, or we have a result and the caller should reset state
		// by calling GetResult and clearing externally if a re-check is wanted.
		// Allow re-check after HasResult/Failed by resetting here instead.
		if (m_state.load() == State::Checking) return;
		// Permit re-check by joining any prior worker first.
		if (m_worker.joinable()) m_worker.join();
		m_state.store(State::Checking);
	}
	if (m_worker.joinable()) m_worker.join();
	m_worker = std::thread([this] { RunCheck(); });
}

State UpdateChecker::GetState() const {
	return m_state.load();
}

UpdateInfo UpdateChecker::GetResult() const {
	std::lock_guard<std::mutex> lock(m_mutex);
	return m_result;
}

void UpdateChecker::RunCheck() {
	UpdateInfo info;
	std::string err;
	std::string body = HttpGet(kHost, kPath, err);
	if (body.empty()) {
		info.errorMessage = err.empty() ? std::string("empty response") : err;
		{
			std::lock_guard<std::mutex> lock(m_mutex);
			m_result = std::move(info);
		}
		m_state.store(State::Failed);
		return;
	}

	picojson::value v;
	std::string parseErr = picojson::parse(v, body);
	if (!parseErr.empty() || !v.is<picojson::object>()) {
		info.errorMessage = "JSON parse failed: " + parseErr;
		{
			std::lock_guard<std::mutex> lock(m_mutex);
			m_result = std::move(info);
		}
		m_state.store(State::Failed);
		return;
	}

	const auto& obj = v.get<picojson::object>();
	auto getStr = [&obj](const char* key) -> std::string {
		auto it = obj.find(key);
		if (it == obj.end() || !it->second.is<std::string>()) return {};
		return it->second.get<std::string>();
	};

	info.latestTag = getStr("tag_name");
	info.releaseNotesUrl = getStr("html_url");
	if (info.latestTag.empty()) {
		info.errorMessage = "GitHub response missing tag_name";
		{
			std::lock_guard<std::mutex> lock(m_mutex);
			m_result = std::move(info);
		}
		m_state.store(State::Failed);
		return;
	}
	info.latestVersion = info.latestTag;
	if (!info.latestVersion.empty() && info.latestVersion[0] == 'v')
		info.latestVersion.erase(0, 1);

	// Walk the assets array; pick the .exe (installer). If only the .zip is
	// present, fall back to it — but the launch path is installer-aware.
	auto itAssets = obj.find("assets");
	if (itAssets != obj.end() && itAssets->second.is<picojson::array>()) {
		for (const auto& a : itAssets->second.get<picojson::array>()) {
			if (!a.is<picojson::object>()) continue;
			const auto& aobj = a.get<picojson::object>();
			auto nameIt = aobj.find("name");
			auto urlIt = aobj.find("browser_download_url");
			auto digestIt = aobj.find("digest");
			auto sizeIt = aobj.find("size");
			if (nameIt == aobj.end() || !nameIt->second.is<std::string>()) continue;
			std::string name = nameIt->second.get<std::string>();
			std::string lower = name;
			for (auto& c : lower) c = (char)std::tolower((unsigned char)c);
			if (lower.size() < 4 || lower.substr(lower.size() - 4) != ".exe") continue;

			info.installerName = std::move(name);
			if (urlIt != aobj.end() && urlIt->second.is<std::string>())
				info.installerUrl = urlIt->second.get<std::string>();
			if (sizeIt != aobj.end() && sizeIt->second.is<double>())
				info.installerSizeBytes = (uint64_t)sizeIt->second.get<double>();
			if (digestIt != aobj.end() && digestIt->second.is<std::string>()) {
				// "digest" is "sha256:hex". Strip the prefix.
				const std::string& d = digestIt->second.get<std::string>();
				const std::string prefix = "sha256:";
				if (d.size() > prefix.size() && d.compare(0, prefix.size(), prefix) == 0)
					info.installerSha256 = d.substr(prefix.size());
			}
			break;
		}
	}

	info.available = IsRemoteNewer(info.latestTag, std::string(SPACECAL_BUILD_STAMP));

	{
		std::lock_guard<std::mutex> lock(m_mutex);
		m_result = std::move(info);
	}
	m_state.store(State::HasResult);
}

} // namespace spacecal::updates
