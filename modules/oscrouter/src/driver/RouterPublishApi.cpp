#include "RouterPublishApi.h"
#include "OscRouter.h"
#include "Logging.h"

#include <atomic>

namespace pairdriver::oscrouter {

bool PublishOsc(const char* source_id, const char* address, const char* typetag, const void* args, size_t arg_len)
{
	::oscrouter::OscRouter* router = ::oscrouter::g_activeRouter.load(std::memory_order_acquire);
	if (!router) {
		// One-shot per session. Every other module that calls PublishOsc
		// would otherwise drop packets silently when the router feature flag
		// is absent; this line tells the user (and the next bug report)
		// exactly why their face tracking / captions output is invisible.
		static std::atomic<bool> warned{false};
		bool expected = false;
		if (warned.compare_exchange_strong(expected, true, std::memory_order_acq_rel)) {
			OR_LOG("WARN: PublishOsc dropped from source='%s' addr='%s' "
			       "because g_activeRouter is null. Is enable_oscrouter.flag "
			       "present in the driver's resources folder?",
			       source_id ? source_id : "(null)", address ? address : "(null)");
		}
		return false;
	}
	return router->PublishOsc(source_id, address, typetag, args, arg_len);
}

} // namespace pairdriver::oscrouter
