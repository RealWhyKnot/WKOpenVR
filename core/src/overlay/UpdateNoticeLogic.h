#pragma once

#include <algorithm>
#include <array>
#include <cctype>
#include <cstdlib>
#include <string>
#include <string_view>

namespace openvr_pair::overlay {

inline bool IsLowerHexSha256(std::string_view value)
{
	if (value.size() != 64) return false;
	for (char ch : value) {
		if ((ch >= '0' && ch <= '9') || (ch >= 'a' && ch <= 'f')) continue;
		return false;
	}
	return true;
}

inline std::string LowerAscii(std::string_view value)
{
	std::string out(value);
	std::transform(out.begin(), out.end(), out.begin(),
	               [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
	return out;
}

inline std::string NormalizeSha256Digest(std::string_view digest)
{
	constexpr std::string_view prefix = "sha256:";
	std::string lower = LowerAscii(digest);
	std::string_view value(lower);
	if (value.size() == prefix.size() + 64 && value.substr(0, prefix.size()) == prefix) {
		value.remove_prefix(prefix.size());
	}
	return IsLowerHexSha256(value) ? std::string(value) : std::string();
}

inline std::string ExtractReleaseBodySha256(std::string_view body)
{
	constexpr std::string_view marker = "SHA256:";
	const size_t markerPos = body.find(marker);
	if (markerPos == std::string_view::npos) return {};

	size_t pos = markerPos + marker.size();
	while (pos < body.size() && (body[pos] == ' ' || body[pos] == '\t')) {
		++pos;
	}
	if (pos + 64 > body.size()) return {};
	return NormalizeSha256Digest(body.substr(pos, 64));
}

inline bool ReleaseBodySha256Matches(std::string_view body, std::string_view expectedSha256)
{
	const std::string bodySha = ExtractReleaseBodySha256(body);
	return !bodySha.empty() && bodySha == NormalizeSha256Digest(expectedSha256);
}

inline std::string ExpectedInstallerAssetName(std::string_view featureName, std::string_view version)
{
	std::string out = "WKOpenVR-";
	out.append(featureName);
	out.append("-v");
	out.append(version);
	out.append("-Setup.exe");
	return out;
}

inline bool ParseVersionStamp(std::string_view raw, std::array<int, 4>& out)
{
	std::string s(raw);
	if (!s.empty() && s[0] == 'v') s.erase(0, 1);
	const auto dash = s.find('-');
	if (dash != std::string::npos) s.resize(dash);

	std::array<int, 4> parsed = {0, 0, 0, 0};
	int idx = 0;
	const char* p = s.c_str();
	while (idx < 4 && *p) {
		char* end = nullptr;
		const long v = std::strtol(p, &end, 10);
		if (end == p) return false;
		parsed[static_cast<size_t>(idx++)] = static_cast<int>(v);
		p = end;
		if (*p == '.')
			++p;
		else if (*p != '\0')
			return false;
	}
	if (idx != 4) return false;
	out = parsed;
	return true;
}

inline bool IsRemoteVersionNewer(std::string_view remote, std::string_view local)
{
	std::array<int, 4> r = {0, 0, 0, 0};
	std::array<int, 4> l = {0, 0, 0, 0};
	if (!ParseVersionStamp(remote, r) || !ParseVersionStamp(local, l)) return false;
	for (size_t i = 0; i < r.size(); ++i) {
		if (r[i] > l[i]) return true;
		if (r[i] < l[i]) return false;
	}
	return false;
}

inline bool IsDevVersionStamp(std::string_view stamp)
{
	return stamp.find('-') != std::string_view::npos;
}

inline bool IsTrustedGitHubReleaseAssetUrl(std::string_view url, std::string_view repoName, std::string_view tagName,
                                           std::string_view assetName)
{
	constexpr std::string_view prefix = "https://github.com/RealWhyKnot/";
	if (url.substr(0, prefix.size()) != prefix) return false;

	std::string expected(prefix);
	expected.append(repoName);
	expected.append("/releases/download/");
	expected.append(tagName);
	expected.push_back('/');
	expected.append(assetName);
	return url == expected;
}

} // namespace openvr_pair::overlay
