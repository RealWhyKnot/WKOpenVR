#include "IPCServer.h"
#include "FeatureFlags.h"
#include "Logging.h"
#include "ServerTrackedDeviceProvider.h"

#include <vector>

void IPCServer::HandleRequest(const protocol::Request& request, protocol::Response& response)
{
	if (!driver->HandleIpcRequest(featureMask, request, response)) {
		LOG("IPC[%s]: invalid request type %d", pipeName.c_str(), request.type);
		response.type = protocol::ResponseInvalid;
	}
}

IPCServer::~IPCServer()
{
	Stop();
}

void IPCServer::Run()
{
	LOG("IPC[%s] starting server thread feature_mask=0x%08x", pipeName.c_str(), (unsigned)featureMask);
	mainThread = std::thread(RunThread, this);
}

void IPCServer::Stop()
{
	TRACE("IPCServer::Stop()");
	LOG("IPC[%s] stop requested running=%d", pipeName.c_str(), running ? 1 : 0);
	// Signal and join even if `running` has already been cleared by the
	// ThreadGuard (early-exit path). A joinable thread whose destructor fires
	// without a join calls std::terminate.
	if (mainThread.joinable()) {
		stop = true;
		// Signal the event only if RunThread hasn't already closed it. The
		// ThreadGuard swaps connectEvent to INVALID_HANDLE_VALUE before
		// CloseHandle so we can distinguish a live handle from a gone one.
		HANDLE ev = connectEvent;
		if (ev && ev != INVALID_HANDLE_VALUE) SetEvent(ev);
		mainThread.join();
	}
	// running and connectEvent are cleared by the ThreadGuard destructor in
	// RunThread before join() returns; nothing left to clean up here.

	TRACE("IPCServer::Stop() finished");
	LOG("IPC[%s] stop finished", pipeName.c_str());
}

IPCServer::PipeInstance* IPCServer::CreatePipeInstance(HANDLE pipe)
{
	// Value-init: PipeInstance contains an OVERLAPPED whose hEvent must be NULL
	// for the completion-routine variants of WriteFileEx/ReadFileEx. Default-init
	// (`new PipeInstance`) leaves OVERLAPPED with heap-residual garbage; some
	// resident bytes in hEvent cause the API to fail.
	auto pipeInst = new PipeInstance{};
	pipeInst->pipe = pipe;
	pipeInst->server = this;
	pipes.insert(pipeInst);
	return pipeInst;
}

void IPCServer::ClosePipeInstance(PipeInstance* pipeInst)
{
	DisconnectNamedPipe(pipeInst->pipe);
	CloseHandle(pipeInst->pipe);
	pipes.erase(pipeInst);
	delete pipeInst;
}

void IPCServer::RunThread(IPCServer* _this)
{
	_this->running = true;
	LOG("IPC[%s] server thread entered feature_mask=0x%08x", _this->pipeName.c_str(), (unsigned)_this->featureMask);

	// RAII guard: on any exit path (normal or error) drain any still-open
	// PipeInstance objects, clear `running`, and close `connectEvent` once.
	// This prevents Stop() from deadlocking on join() when the thread exits
	// early due to a WaitForSingleObjectEx or GetOverlappedResult failure,
	// and it stops the early-return paths from leaking PipeInstance heap
	// allocations + their kernel pipe handles.
	struct ThreadGuard
	{
		IPCServer* server;
		~ThreadGuard()
		{
			LOG("IPC[%s] server thread exiting open_pipes=%zu stop=%d", server->pipeName.c_str(), server->pipes.size(),
			    server->stop ? 1 : 0);
			// Snapshot then close: ClosePipeInstance erases from `pipes`,
			// which invalidates iterators on std::set during a range-for.
			std::vector<PipeInstance*> snapshot(server->pipes.begin(), server->pipes.end());
			server->pipes.clear();
			for (auto* p : snapshot) {
				DisconnectNamedPipe(p->pipe);
				CloseHandle(p->pipe);
				delete p;
			}

			server->running = false;
			// Close and null-out connectEvent using INVALID_HANDLE_VALUE as a
			// sentinel so Stop() -- which may race here -- does not double-close.
			HANDLE ev = server->connectEvent;
			if (ev && ev != INVALID_HANDLE_VALUE) {
				server->connectEvent = INVALID_HANDLE_VALUE;
				CloseHandle(ev);
			}
		}
	} guard{_this};

	HANDLE connectEvent = _this->connectEvent = CreateEvent(0, TRUE, TRUE, 0);
	if (!connectEvent) {
		LOG("CreateEvent failed in RunThread. Error: %d", GetLastError());
		return;
	}
	LOG("IPC[%s] listen event created", _this->pipeName.c_str());

	OVERLAPPED connectOverlap;
	connectOverlap.hEvent = connectEvent;

	HANDLE nextPipe;
	BOOL connectPending = _this->CreateAndConnectInstance(&connectOverlap, nextPipe);

	while (!_this->stop) {
		DWORD wait = WaitForSingleObjectEx(connectEvent, INFINITE, TRUE);

		if (_this->stop) {
			break;
		}
		else if (wait == 0) {
			// When connectPending is false, the last call to CreateAndConnectInstance
			// picked up a connected client and triggered this event, so we can simply
			// create a new pipe instance for it. If true, the client was still pending
			// connection when CreateAndConnectInstance returned, so this event was triggered
			// internally and we need to flush out the result, or something like that.
			if (connectPending) {
				DWORD bytesConnect;
				BOOL success = GetOverlappedResult(nextPipe, &connectOverlap, &bytesConnect, FALSE);
				if (!success) {
					LOG("GetOverlappedResult failed in RunThread. Error: %d", GetLastError());
					// Close the still-pending pipe handle before bailing --
					// neither CreatePipeInstance (which would have taken
					// ownership) nor any other path runs on this branch.
					// Without this, the kernel handle leaks every time the
					// overlapped wait fails (rare in practice but the leak
					// is cumulative across the driver's entire load lifetime).
					CloseHandle(nextPipe);
					nextPipe = INVALID_HANDLE_VALUE;
					return;
				}
			}

			LOG("IPC[%s] client connected (our_protocol=%u)", _this->pipeName.c_str(), (unsigned)protocol::Version);

			auto pipeInst = _this->CreatePipeInstance(nextPipe);
			CompletedWriteCallback(0, sizeof protocol::Response, (LPOVERLAPPED)pipeInst);

			connectPending = _this->CreateAndConnectInstance(&connectOverlap, nextPipe);
		}
		else if (wait != WAIT_IO_COMPLETION) {
			LOG("WaitForSingleObjectEx failed in RunThread. Error: %d", GetLastError());
			return;
		}
	}
	// Pipe-instance cleanup runs in the ThreadGuard destructor on all exit
	// paths (normal + error returns above).
}

BOOL IPCServer::CreateAndConnectInstance(LPOVERLAPPED overlap, HANDLE& pipe)
{
	pipe = CreateNamedPipeA(pipeName.c_str(), PIPE_ACCESS_DUPLEX | FILE_FLAG_OVERLAPPED,
	                        PIPE_TYPE_MESSAGE | PIPE_READMODE_MESSAGE | PIPE_WAIT, PIPE_UNLIMITED_INSTANCES,
	                        sizeof protocol::Request, sizeof protocol::Response, 1000, 0);

	if (pipe == INVALID_HANDLE_VALUE) {
		LOG("CreateNamedPipe(%s) failed. Error: %d", pipeName.c_str(), GetLastError());
		return FALSE;
	}

	ConnectNamedPipe(pipe, overlap);

	switch (GetLastError()) {
		case ERROR_IO_PENDING:
			// Mark a pending connection by returning true, and when the connection
			// completes an event will trigger automatically.
			LOG("IPC[%s] waiting for client", pipeName.c_str());
			return TRUE;

		case ERROR_PIPE_CONNECTED:
			// Signal the event loop that a client is connected.
			if (SetEvent(overlap->hEvent)) {
				LOG("IPC[%s] client was already connected at pipe creation", pipeName.c_str());
				return FALSE;
			}
	}

	// Pipe handle was created above but neither path took ownership of it. Close
	// before returning failure so we don't leak a kernel pipe handle every retry.
	LOG("ConnectNamedPipe(%s) failed. Error: %d", pipeName.c_str(), GetLastError());
	CloseHandle(pipe);
	pipe = INVALID_HANDLE_VALUE;
	return FALSE;
}

void IPCServer::CompletedReadCallback(DWORD err, DWORD bytesRead, LPOVERLAPPED overlap)
{
	PipeInstance* pipeInst = (PipeInstance*)overlap;
	BOOL success = FALSE;

	if (err == 0 && bytesRead == sizeof protocol::Request) {
		pipeInst->server->HandleRequest(pipeInst->request, pipeInst->response);
		success = WriteFileEx(pipeInst->pipe, &pipeInst->response, sizeof protocol::Response, overlap,
		                      (LPOVERLAPPED_COMPLETION_ROUTINE)CompletedWriteCallback);
	}
	else if (err == 0 && bytesRead > 0) {
		// Partial message on a PIPE_TYPE_MESSAGE pipe means the peer wrote
		// fewer bytes than sizeof(Request). The Request union has overlapping
		// payloads and an unread tail would be uninitialized memory; dispatching
		// HandleRequest in that state can do anything. Drop and disconnect.
		LOG("IPC[%s] short read: got=%u expected=%zu, disconnecting client", pipeInst->server->pipeName.c_str(),
		    bytesRead, sizeof protocol::Request);
	}

	if (!success) {
		if (err == ERROR_BROKEN_PIPE) {
			LOG("IPC[%s] client disconnecting normally", pipeInst->server->pipeName.c_str());
		}
		else {
			LOG("IPC[%s] client disconnecting due to error (CompletedReadCallback): err=%d bytesRead=%d",
			    pipeInst->server->pipeName.c_str(), err, bytesRead);
		}
		pipeInst->server->ClosePipeInstance(pipeInst);
	}
}

void IPCServer::CompletedWriteCallback(DWORD err, DWORD bytesWritten, LPOVERLAPPED overlap)
{
	PipeInstance* pipeInst = (PipeInstance*)overlap;
	BOOL success = FALSE;

	if (err == 0 && bytesWritten == sizeof protocol::Response) {
		success = ReadFileEx(pipeInst->pipe, &pipeInst->request, sizeof protocol::Request, overlap,
		                     (LPOVERLAPPED_COMPLETION_ROUTINE)CompletedReadCallback);
	}

	if (!success) {
		LOG("IPC[%s] client disconnecting due to error (CompletedWriteCallback): err=%d bytesWritten=%d",
		    pipeInst->server->pipeName.c_str(), err, bytesWritten);
		pipeInst->server->ClosePipeInstance(pipeInst);
	}
}
