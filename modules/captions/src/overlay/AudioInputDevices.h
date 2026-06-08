#pragma once

#include <string>
#include <vector>

namespace captions {

// One active capture endpoint: the stable WASAPI id (IMMDevice::GetId, what the
// host opens) plus the human-friendly name shown in the picker.
struct AudioInputDevice
{
	std::string id;
	std::string name;
};

// Enumerate active capture (microphone) endpoints. Returns an empty vector on
// failure. Cheap enough to call when the picker combo is opened; not meant for
// per-frame use. The "system default" choice is represented in the UI by an
// empty id and is not part of this list.
std::vector<AudioInputDevice> EnumerateCaptureDevices();

} // namespace captions
