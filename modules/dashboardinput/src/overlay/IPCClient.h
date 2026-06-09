#pragma once

#include "IpcClientBase.h"

class DashboardInputIPCClient : public openvr_pair::overlay::IpcClientBase
{
public:
	void Connect();
};
