#define _CRT_SECURE_NO_DEPRECATE
#include "Captions.h"
#include "Logging.h"

#include <sstream>
#include <stdexcept>
#include <vector>
#include <algorithm>

#define WIN32_LEAN_AND_MEAN
#include <windows.h>

// CTranslate2 headers/import libs are available from lib/ctranslate2/ or the
// CMake build cache. The runtime DLL is installed on demand by the selected
// translation pack.
#ifdef HAVE_CT2
#include <ctranslate2/translator.h>
#endif
#if defined(HAVE_CT2) && defined(HAVE_SENTENCEPIECE)
#include <sentencepiece_processor.h>
#endif

#if defined(HAVE_CT2) && defined(HAVE_SENTENCEPIECE)

struct Captions::Impl
{
	std::unique_ptr<ctranslate2::Translator> ct2;
	std::unique_ptr<sentencepiece::SentencePieceProcessor> source_sp;
	std::unique_ptr<sentencepiece::SentencePieceProcessor> target_sp;
	std::string model_dir;
};

static std::string JoinModelPath(const std::string& dir, const char* leaf)
{
	if (dir.empty()) return leaf ? std::string(leaf) : std::string();
	const char last = dir.back();
	if (last == '\\' || last == '/') return dir + leaf;
	return dir + "\\" + leaf;
}

static std::string TrimAscii(std::string text)
{
	const auto not_space = [](unsigned char ch) {
		return ch != ' ' && ch != '\t' && ch != '\r' && ch != '\n';
	};
	const auto first = std::find_if(text.begin(), text.end(), not_space);
	if (first == text.end()) return {};
	const auto last = std::find_if(text.rbegin(), text.rend(), not_space).base();
	return std::string(first, last);
}

Captions::Captions() : impl_(std::make_unique<Impl>()) {}

Captions::~Captions()
{
	Unload();
}

bool Captions::RuntimeAvailable()
{
	HMODULE h = LoadLibraryW(L"ctranslate2.dll");
	if (!h) return false;
	FreeLibrary(h);
	return true;
}

bool Captions::Load(const std::string& model_dir)
{
	Unload();
	try {
		const std::string source_spm = JoinModelPath(model_dir, "source.spm");
		const std::string target_spm = JoinModelPath(model_dir, "target.spm");

		auto source_sp = std::make_unique<sentencepiece::SentencePieceProcessor>();
		auto target_sp = std::make_unique<sentencepiece::SentencePieceProcessor>();
		auto source_status = source_sp->Load(source_spm);
		if (!source_status.ok()) {
			TH_LOG("[captions] source tokenizer load failed: %s", source_status.ToString().c_str());
			return false;
		}
		auto target_status = target_sp->Load(target_spm);
		if (!target_status.ok()) {
			TH_LOG("[captions] target tokenizer load failed: %s", target_status.ToString().c_str());
			return false;
		}

		ctranslate2::Device device = ctranslate2::Device::CPU;
		ctranslate2::ComputeType compute = ctranslate2::ComputeType::INT8;
		auto translator = std::make_unique<ctranslate2::Translator>(model_dir, device, compute);
		impl_->source_sp = std::move(source_sp);
		impl_->target_sp = std::move(target_sp);
		impl_->ct2 = std::move(translator);
		impl_->model_dir = model_dir;
		TH_LOG("[captions] model loaded from '%s' with SentencePiece tokenizers", model_dir.c_str());
		return true;
	}
	catch (const std::exception& e) {
		TH_LOG("[captions] load failed: %s", e.what());
		return false;
	}
}

void Captions::Unload()
{
	impl_->ct2.reset();
	impl_->source_sp.reset();
	impl_->target_sp.reset();
	impl_->model_dir.clear();
}

bool Captions::IsLoaded() const noexcept
{
	return impl_->ct2 != nullptr;
}

std::string Captions::Translate(const std::string& text, const std::string& src_lang, const std::string& tgt_lang)
{
	if (text.empty()) return {};
	if (src_lang == tgt_lang) return text;
	if (!impl_->ct2 || !impl_->source_sp || !impl_->target_sp) return text;

	try {
		std::vector<std::string> tokens;
		auto encode_status = impl_->source_sp->Encode(text, &tokens);
		if (!encode_status.ok()) {
			TH_LOG("[captions] source tokenization failed: %s", encode_status.ToString().c_str());
			return {};
		}
		if (tokens.empty()) return {};

		// Marian OPUS-MT CTranslate2 models need an explicit source EOS token.
		// Without it, short inputs can repeat phrases until max_decoding_length.
		tokens.push_back("</s>");

		ctranslate2::TranslationOptions opts;
		opts.beam_size = 4;
		opts.no_repeat_ngram_size = 3;
		opts.disable_unk = true;
		opts.max_decoding_length = std::max<size_t>(32, std::min<size_t>(128, tokens.size() * 5 + 16));

		std::vector<std::vector<std::string>> batch = {tokens};
		auto results = impl_->ct2->translate_batch(batch, opts);

		if (results.empty() || results[0].hypotheses.empty()) return {};

		std::string out;
		auto decode_status = impl_->target_sp->Decode(results[0].hypotheses[0], &out);
		if (!decode_status.ok()) {
			TH_LOG("[captions] target detokenization failed: %s", decode_status.ToString().c_str());
			return {};
		}
		return TrimAscii(out);
	}
	catch (const std::exception& e) {
		TH_LOG("[captions] inference error: %s", e.what());
		return {};
	}
}

#else // !HAVE_CT2 || !HAVE_SENTENCEPIECE

// Build-without-translation stubs. The host links and runs; Translate() returns
// the input string unchanged so the chatbox still gets the user's words when no
// target language is requested. A first-call log line explains how to enable
// real translation.
struct Captions::Impl
{
};

Captions::Captions() : impl_(std::make_unique<Impl>()) {}
Captions::~Captions() = default;
bool Captions::RuntimeAvailable()
{
	return false;
}

bool Captions::Load(const std::string&)
{
	static bool logged = false;
	if (!logged) {
		TH_LOG("[captions] CTranslate2 and SentencePiece were not both available at build time. The captions host "
		       "built in translation-stub mode. Reconfigure with WKOPENVR_CAPTIONS_FETCH_CT2=ON and "
		       "WKOPENVR_CAPTIONS_FETCH_SENTENCEPIECE=ON, then rebuild.");
		logged = true;
	}
	return false;
}

void Captions::Unload() {}
bool Captions::IsLoaded() const noexcept
{
	return false;
}
std::string Captions::Translate(const std::string& text, const std::string&, const std::string&)
{
	return text;
}

#endif // HAVE_CT2 && HAVE_SENTENCEPIECE
