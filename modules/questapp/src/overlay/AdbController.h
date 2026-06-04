#pragma once

#include <chrono>
#include <string>
#include <vector>

// Wrapper for launching adb.exe as a subprocess.
// Quest App installs platform-tools under the user's WKOpenVR data folder.
class AdbController
{
public:
	struct Result
	{
		int exitCode = -1;
		std::string out;
		std::string err;
		bool timedOut = false;
	};

	AdbController();
	explicit AdbController(std::string adbPath);
	virtual ~AdbController();

	// True if the resolved binary path exists on disk.
	// Virtual so test subclasses can override without requiring a real binary.
	virtual bool BinaryAvailable() const;
	// Absolute path that was resolved at construction time (may be empty on failure).
	std::string ResolvedBinaryPath() const;
	void RefreshResolvedBinaryPath();

	// Spawn adb with the given argument list. Each element is quoted if it
	// contains spaces. Returns after the process exits or timeout fires.
	// Virtual so test subclasses can inject stub outputs without a real binary.
	virtual Result Run(const std::vector<std::string>& args,
	                   std::chrono::milliseconds timeout = std::chrono::seconds(10));

	// Run `adb shell <cmd>`. Convenience wrapper around Run.
	Result Shell(const std::string& cmd, std::chrono::milliseconds timeout = std::chrono::seconds(10));

	// `adb connect <endpoint>`. Returns true on exit==0 AND stdout contains
	// "connected to" (idempotent -- already-connected reports the same string).
	// Virtual so test subclasses can inject results without spawning subprocesses.
	virtual bool Connect(const std::string& endpoint);
	virtual bool Connect(const std::string& endpoint, std::chrono::milliseconds timeout);

	bool LastConnectTimedOut() const;
	bool LastConnectWasRefused() const;

	// `adb get-state`. Returns true if exit==0 and stdout contains "device".
	virtual bool Connected();

	// `adb disconnect <endpoint>`. Passing an empty endpoint disconnects all TCP
	// endpoints known to the local adb server. Returns true if adb exits cleanly.
	virtual bool Disconnect(const std::string& endpoint = {});

	// `adb usb` or `adb -s <endpoint> usb`. This asks adbd to leave TCP/Wi-Fi
	// mode and listen on USB again. Returns true if adb exits cleanly.
	virtual bool DisableWirelessAdb(const std::string& endpoint = {});

	// `adb tcpip <port>` asks adbd to listen on Wi-Fi/TCP. This usually needs
	// an authorized USB connection first and does not survive headset reboot.
	virtual bool EnableWirelessAdb(int port = 5555);
	virtual bool EnableWirelessAdb(const std::string& serial, int port);

	// Opens an interactive cmd.exe window running `adb shell`.
	bool OpenInteractiveShell() const;
	bool OpenInteractiveShell(const std::string& serial) const;

	// Opens cmd.exe in the platform-tools directory for ad hoc adb commands.
	bool OpenToolsTerminal() const;

protected:
	// Resolved at construction from the Quest App platform-tools install.
	// Protected so test subclasses can substitute an alternative binary path
	// without requiring a real adb.exe to be present.
	std::string m_adbPath;

private:
	std::string m_targetSerial;
	bool m_lastConnectTimedOut = false;
	bool m_lastConnectWasRefused = false;

	// Quote an argument if it contains spaces or is empty, so CreateProcessW
	// sees it as a single token.
	static std::string QuoteArg(const std::string& arg);

	// Trim leading/trailing ASCII whitespace in place.
	static std::string TrimAscii(std::string s);
};
