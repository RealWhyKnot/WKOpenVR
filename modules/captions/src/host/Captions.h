#pragma once

#include <memory>
#include <string>

// CTranslate2-based OPUS-MT translation wrapper.
//
// Load() takes the path to a converted ct2 model directory (e.g.
// "ct2-opus-mt-zh-en"). Translate() returns the translated string.
// When source and target language are the same, Translate() returns the
// input unchanged without running inference.
class Captions
{
public:
	Captions();
	~Captions();

	static bool RuntimeAvailable();

	// Load model directory. Returns false on failure.
	bool Load(const std::string& model_dir);

	void Unload();

	bool IsLoaded() const noexcept;

	// Translate `text` from src_lang to tgt_lang. If src_lang == tgt_lang or
	// the model is not loaded, returns `text` as-is.
	std::string Translate(const std::string& text, const std::string& src_lang, const std::string& tgt_lang);

private:
	struct Impl;
	std::unique_ptr<Impl> impl_;
};
