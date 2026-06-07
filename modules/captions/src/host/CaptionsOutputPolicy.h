#pragma once

#include <string>

namespace captions {

inline bool ShouldPublishChatbox(bool chatboxEnabled, const std::string& text)
{
	return chatboxEnabled && !text.empty();
}

inline bool ShouldDrainQueuedChatbox(bool chatboxEnabled)
{
	return chatboxEnabled;
}

} // namespace captions
