#include "IpcSendQueue.h"

#include "DiagnosticsLog.h"

#include <utility>

namespace openvr_pair::overlay {

IpcSendQueue::~IpcSendQueue()
{
	Stop();
}

void IpcSendQueue::Start(std::string pipeName, IpcClientConnectOptions clientOptions)
{
	std::unique_lock<std::mutex> lock(mutex_);
	if (started_) return;
	pipeName_ = std::move(pipeName);
	clientOptions_ = std::move(clientOptions);
	stop_ = false;
	status_ = {};
	started_ = true;
	worker_ = std::thread([this] { Run(); });
	openvr_pair::common::DiagnosticLog("ipc-send-queue", "started pipe='%s'", pipeName_.c_str());
}

void IpcSendQueue::Stop()
{
	{
		std::unique_lock<std::mutex> lock(mutex_);
		if (!started_) return;
		stop_ = true;
		// Discard queued entries: they are periodic republish traffic. The
		// worker exits after at most its current in-flight operation, whose
		// duration the client's IO deadline already bounds.
		queue_.clear();
		status_.queueDepth = 0;
	}
	cv_.notify_all();
	if (worker_.joinable()) worker_.join();
	{
		std::unique_lock<std::mutex> lock(mutex_);
		started_ = false;
	}
	openvr_pair::common::DiagnosticLog("ipc-send-queue", "stopped pipe='%s'", pipeName_.c_str());
}

bool IpcSendQueue::IsRunning() const
{
	std::unique_lock<std::mutex> lock(mutex_);
	return started_ && !stop_;
}

void IpcSendQueue::Enqueue(const protocol::Request& request, std::string coalesceKey)
{
	{
		std::unique_lock<std::mutex> lock(mutex_);
		if (!started_ || stop_) {
			++status_.dropped;
			return;
		}
		if (!coalesceKey.empty()) {
			for (auto& entry : queue_) {
				if (entry.key == coalesceKey) {
					entry.request = request;
					++status_.coalesced;
					return;
				}
			}
		}
		if (queue_.size() >= kMaxQueue) {
			++status_.dropped;
			return;
		}
		queue_.push_back(Entry{request, std::move(coalesceKey)});
		status_.queueDepth = static_cast<uint32_t>(queue_.size());
	}
	cv_.notify_one();
}

IpcSendQueue::Status IpcSendQueue::GetStatus() const
{
	std::unique_lock<std::mutex> lock(mutex_);
	return status_;
}

void IpcSendQueue::Run()
{
	IpcClientBase client;
	for (;;) {
		Entry entry;
		{
			std::unique_lock<std::mutex> lock(mutex_);
			cv_.wait(lock, [this] { return stop_ || !queue_.empty(); });
			if (stop_) return;
			entry = std::move(queue_.front());
			queue_.pop_front();
			status_.queueDepth = static_cast<uint32_t>(queue_.size());
		}
		try {
			if (!client.IsConnected()) {
				client.Connect(pipeName_.c_str(), clientOptions_);
			}
			client.SendBlocking(entry.request);
			std::unique_lock<std::mutex> lock(mutex_);
			++status_.sent;
			status_.connected = true;
			status_.connectionGeneration = client.ConnectionGeneration();
		}
		catch (const std::exception& e) {
			// Connect backoff + the per-operation deadline bound the failure
			// path; the entry is dropped (periodic traffic re-sends). Keep
			// the diagnostic quiet: the connect/timeout paths already log.
			(void)e;
			std::unique_lock<std::mutex> lock(mutex_);
			++status_.sendFailures;
			status_.connected = client.IsConnected();
			status_.connectionGeneration = client.ConnectionGeneration();
		}
	}
}

} // namespace openvr_pair::overlay
