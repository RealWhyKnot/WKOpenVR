#include "OscWire.h"

#include <cstring>

namespace oscrouter {

// ---------------------------------------------------------------------------
// OSC 1.0 address pattern matching
//
// Recursive descent. OSC 1.0 path components are delimited by '/'; the
// matcher is applied to the full address string (with slashes), not
// per-component, which is sufficient for the router's use case (patterns
// like /avatar/parameters/* or /avatar/parameters/{JawOpen,LipOpen}).
//
// Spec: https://opensoundcontrol.stanford.edu/spec-1_0.html section 5.
// ---------------------------------------------------------------------------

// Forward declaration for mutual recursion in the alt-match branch.
static bool PatternMatchImpl(const char* p, const char* a);

// Match a character class '[...]'. On entry p points to the char after '['.
// Returns true if `ch` is in the class. Advances *pptr past the closing ']'.
static bool MatchCharClass(const char** pptr, char ch, bool& ok)
{
	ok = false;
	bool matched = false;
	bool negated = false;
	const char* p = *pptr;

	if (*p == '!') {
		negated = true;
		++p;
	}

	while (*p && *p != ']') {
		char c1 = *p++;
		if (*p == '-' && *(p + 1) && *(p + 1) != ']') {
			char c2 = *(p + 1);
			p += 2;
			if (ch >= c1 && ch <= c2) matched = true;
		}
		else {
			if (ch == c1) matched = true;
		}
	}

	if (*p != ']') {
		ok = false;
		return false;
	} // malformed
	*pptr = p + 1;
	ok = true;
	return negated ? !matched : matched;
}

// Match the comma-delimited alternatives '{a,b,c}' against the address
// starting at `a`. Returns true if any alternative matches the rest of
// the address when combined with the rest of the pattern `rest`.
// On entry `p` points to the char after '{'.
static bool MatchAlternates(const char* p, const char* a, const char* rest)
{
	// Collect each alternative and try PatternMatchImpl(alt + rest, a).
	char alt[256];
	const char* start = p;
	while (*p && *p != '}') {
		if (*p == ',') {
			// Try this alternative.
			size_t len = static_cast<size_t>(p - start);
			if (len >= sizeof(alt) - 1) return false;
			memcpy(alt, start, len);
			// Append the rest of the pattern after '}'.
			size_t rlen = strlen(rest);
			if (len + rlen >= sizeof(alt)) return false;
			memcpy(alt + len, rest, rlen + 1); // includes NUL
			if (PatternMatchImpl(alt, a)) return true;
			start = p + 1;
		}
		++p;
	}
	if (*p != '}') return false; // malformed, no closing brace

	// Try the last alternative (between start and p).
	size_t len = static_cast<size_t>(p - start);
	if (len >= sizeof(alt) - 1) return false;
	memcpy(alt, start, len);
	const char* after_brace = p + 1;
	size_t rlen = strlen(after_brace);
	if (len + rlen >= sizeof(alt)) return false;
	memcpy(alt + len, after_brace, rlen + 1);
	return PatternMatchImpl(alt, a);
}

static bool PatternMatchImpl(const char* p, const char* a)
{
	while (*p) {
		switch (*p) {
			case '?':
				if (!*a) return false;
				++p;
				++a;
				break;

			case '*':
				// Advance past consecutive '*' (they're equivalent to one).
				while (*p == '*')
					++p;
				if (!*p) return true; // pattern ends with '*', matches anything
				// Try anchoring at every position in the remainder of `a`.
				for (; *a; ++a) {
					if (PatternMatchImpl(p, a)) return true;
				}
				return false;

			case '[': {
				++p; // skip '['
				bool ok = false;
				bool hit = MatchCharClass(&p, *a, ok);
				if (!ok || !hit || !*a) return false;
				++a;
				break;
			}

			case '{': {
				++p; // skip '{'
				// Find the matching '}' to extract the rest of the pattern.
				const char* rest = p;
				int depth = 1;
				while (*rest && depth > 0) {
					if (*rest == '{')
						++depth;
					else if (*rest == '}')
						--depth;
					++rest;
				}
				// `rest` is now one past the closing '}', or end of string.
				return MatchAlternates(p, a, rest);
			}

			default:
				if (*p != *a) return false;
				++p;
				++a;
				break;
		}
	}
	return *a == '\0';
}

bool OscPatternMatch(const char* pattern, const char* address)
{
	if (!pattern || !address) return false;
	return PatternMatchImpl(pattern, address);
}

} // namespace oscrouter
