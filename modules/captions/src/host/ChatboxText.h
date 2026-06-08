#pragma once

#include <cstddef>
#include <string>
#include <vector>

namespace captions {

inline size_t VrchatChatboxByteLimit()
{
	return 144;
}

inline bool ChatboxAsciiSpace(unsigned char ch)
{
	return ch == ' ' || ch == '\t' || ch == '\r' || ch == '\n';
}

inline bool ChatboxUtf8Continuation(unsigned char ch)
{
	return (ch & 0xC0u) == 0x80u;
}

inline size_t ClampChatboxUtf8Boundary(const std::string& text, size_t pos)
{
	if (pos >= text.size()) return text.size();
	while (pos > 0 && ChatboxUtf8Continuation(static_cast<unsigned char>(text[pos]))) {
		--pos;
	}
	return pos;
}

inline std::string TrimChatboxChunk(const std::string& text)
{
	size_t begin = 0;
	while (begin < text.size() && ChatboxAsciiSpace(static_cast<unsigned char>(text[begin]))) {
		++begin;
	}
	size_t end = text.size();
	while (end > begin && ChatboxAsciiSpace(static_cast<unsigned char>(text[end - 1]))) {
		--end;
	}
	return text.substr(begin, end - begin);
}

inline std::vector<std::string> SplitTextForChatbox(const std::string& text,
                                                    size_t byteLimit = VrchatChatboxByteLimit())
{
	std::vector<std::string> chunks;
	if (byteLimit == 0) return chunks;

	size_t offset = 0;
	while (offset < text.size()) {
		while (offset < text.size() && ChatboxAsciiSpace(static_cast<unsigned char>(text[offset]))) {
			++offset;
		}
		if (offset >= text.size()) break;

		if (text.size() - offset <= byteLimit) {
			std::string chunk = TrimChatboxChunk(text.substr(offset));
			if (!chunk.empty()) chunks.push_back(chunk);
			break;
		}

		size_t hardEnd = ClampChatboxUtf8Boundary(text, offset + byteLimit);
		if (hardEnd <= offset) {
			hardEnd = offset + byteLimit;
			if (hardEnd > text.size()) hardEnd = text.size();
			hardEnd = ClampChatboxUtf8Boundary(text, hardEnd);
		}
		if (hardEnd <= offset) break;

		size_t split = std::string::npos;
		for (size_t i = hardEnd; i > offset; --i) {
			if (ChatboxAsciiSpace(static_cast<unsigned char>(text[i - 1]))) {
				split = i - 1;
				break;
			}
		}

		const size_t chunkEnd = (split != std::string::npos && split > offset) ? split : hardEnd;
		std::string chunk = TrimChatboxChunk(text.substr(offset, chunkEnd - offset));
		if (!chunk.empty()) chunks.push_back(chunk);

		offset = (split != std::string::npos && split > offset) ? split + 1 : hardEnd;
	}

	return chunks;
}

inline std::string TruncateTextForChatbox(const std::string& text, size_t byteLimit = VrchatChatboxByteLimit())
{
	if (text.size() <= byteLimit) return text;
	const std::vector<std::string> chunks = SplitTextForChatbox(text, byteLimit);
	if (chunks.empty()) return {};
	if (byteLimit <= 3) return chunks.front().substr(0, byteLimit);
	if (chunks.front().size() <= byteLimit - 3) return chunks.front() + "...";

	size_t end = ClampChatboxUtf8Boundary(chunks.front(), byteLimit - 3);
	while (end > 0 && ChatboxAsciiSpace(static_cast<unsigned char>(chunks.front()[end - 1]))) {
		--end;
	}
	return chunks.front().substr(0, end) + "...";
}

} // namespace captions
