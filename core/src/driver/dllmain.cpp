#include "Logging.h"

#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <windows.h>
#include <cstdio>

// Build-time version string. CMake reads version.txt and injects this macro
// via target_compile_definitions; the fallback only fires for an out-of-tree
// build that didn't go through the normal configure step.
#ifndef PAIRDRIVER_VERSION_STRING
#define PAIRDRIVER_VERSION_STRING "0.0.0.0-dev"
#endif

BOOL APIENTRY DllMain(HMODULE hModule, DWORD ul_reason_for_call, LPVOID lpReserved)
{
	switch (ul_reason_for_call) {
		case DLL_PROCESS_ATTACH:
			OpenLogFile();
			LOG("WKOpenVR " PAIRDRIVER_VERSION_STRING " loaded");
			break;
		case DLL_PROCESS_DETACH:
			LOG("WKOpenVR unloaded");
			CloseLogFile();
			break;
		case DLL_THREAD_ATTACH:
		case DLL_THREAD_DETACH:
			break;
	}
	return TRUE;
}
