/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2013 Dr. Chat / Erwin Coumans  http://bulletphysics.com

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

#include "btWin32ThreadPool.h"

#ifdef USE_WIN32_THREADING
#include <Windows.h>

// MSVC-Specific thread name setting
#ifdef _MSC_VER
// Stolen from http://msdn.microsoft.com/en-us/library/xcb2z8hs%28v=vs.110%29.aspx
const DWORD MS_VC_EXCEPTION=0x406D1388;

#pragma pack(push,8)
typedef struct tagTHREADNAME_INFO
{
   DWORD dwType; // Must be 0x1000.
   LPCSTR szName; // Pointer to name (in user addr space).
   DWORD dwThreadID; // Thread ID (-1=caller thread).
   DWORD dwFlags; // Reserved for future use, must be zero.
} THREADNAME_INFO;
#pragma pack(pop)

static void SetThreadName(DWORD dwThreadID, const char* threadName)
{
	THREADNAME_INFO info;
	info.dwType = 0x1000;
	info.szName = threadName;
	info.dwThreadID = dwThreadID;
	info.dwFlags = 0;

	__try
	{
		RaiseException(MS_VC_EXCEPTION, 0, sizeof(info)/sizeof(ULONG_PTR), (ULONG_PTR*)&info);
	}
	__except(EXCEPTION_EXECUTE_HANDLER)
	{
	}
}
#endif // _MSC_VER

static DWORD WINAPI ThreadFunc(LPVOID lpParam)
{
	if (!lpParam)
		return 1;

	btWin32ThreadStatus *status = (btWin32ThreadStatus *)lpParam;

	while (true)
	{
		status->m_status = btWin32ThreadStatus::STATUS_READY;

		WaitForSingleObject(status->m_pStartEvent, INFINITE);

		switch (status->m_command)
		{
			case btWin32ThreadStatus::COMMAND_RUNTASK:
				status->m_status = btWin32ThreadStatus::STATUS_BUSY;

				status->m_threadFunc(status->m_userTaskId, status->m_pUserPointer);
				status->m_status = btWin32ThreadStatus::STATUS_READY;

				SetEvent(status->m_pCompletedEvent);
				break;
			case btWin32ThreadStatus::COMMAND_EXIT:
				status->m_status = btWin32ThreadStatus::STATUS_FINISHED;
				SetEvent(status->m_pCompletedEvent);

				return 0;
				break;
		}
	}

	return 0;
}

class btWin32CriticalSection : public btCriticalSection
{
	public:
		btWin32CriticalSection()
		{
			InitializeCriticalSection(&m_criticalSection);
		}

		~btWin32CriticalSection()
		{
			DeleteCriticalSection(&m_criticalSection);
		}

		bool trylock()
		{
			return TryEnterCriticalSection(&m_criticalSection) == TRUE;
		}

		void lock()
		{
			EnterCriticalSection(&m_criticalSection);
		}

		void unlock()
		{
			LeaveCriticalSection(&m_criticalSection);
		}

	private:
		CRITICAL_SECTION m_criticalSection;
};

btWin32ThreadPool::btWin32ThreadPool() :
	m_bThreadsRunning(false)
{
}

btWin32ThreadPool::~btWin32ThreadPool()
{
	if (m_bThreadsRunning)
	{
		stopThreads();
	}
}

void btWin32ThreadPool::startThreads(const btThreadPoolInfo &info)
{
	btAssert(!m_bThreadsRunning);
	m_activeThreadStatus.resize(m_threadInfo.m_numThreads);
	m_completeHandles.resize(m_threadInfo.m_numThreads);
	m_threadInfo = info;

	for (uint32_t i = 0; i < m_threadInfo.m_numThreads; i++)
	{
		btWin32ThreadStatus &status = m_activeThreadStatus[i];

		status.m_pStartEvent = CreateEventA(NULL, false, false, NULL);
		status.m_pCompletedEvent = CreateEventA(NULL, false, false, NULL);
		m_completeHandles[i] = status.m_pCompletedEvent;

		status.m_pHandle = CreateThread(NULL, m_threadInfo.m_threadStackSize, ThreadFunc, &status, CREATE_SUSPENDED, 0);
		SetThreadPriority(status.m_pHandle, THREAD_PRIORITY_HIGHEST);
		
#ifdef _MSC_VER
		// For easier debugging.
		char threadName[32];
		sprintf_s(threadName, "%s-%d", m_threadInfo.m_pUniqueName, i);
		SetThreadName(GetThreadId(status.m_pHandle), threadName);
#endif

		status.m_status = btWin32ThreadStatus::STATUS_INVALID;
		status.m_command = btWin32ThreadStatus::COMMAND_INVALID;
		status.m_threadFunc = m_threadInfo.m_threadFunc;
		status.m_pUserPointer = NULL;

		// Set the thread free
		ResumeThread(status.m_pHandle);
	}

	m_bThreadsRunning = true;
}

void btWin32ThreadPool::stopThreads()
{
	btAssert(m_bThreadsRunning);

	for (uint32_t i = 0; i < m_threadInfo.m_numThreads; i++)
	{
		btWin32ThreadStatus &status = m_activeThreadStatus[i];

		status.m_command = btWin32ThreadStatus::COMMAND_EXIT;
		SetEvent(status.m_pStartEvent);

		// Wait 30 seconds
		DWORD dwRet = WaitForSingleObject(status.m_pCompletedEvent, 30000);
		if (dwRet == WAIT_TIMEOUT || dwRet == WAIT_FAILED)
		{
			// Should we warn the user that we had to terminate this thread?
			TerminateThread(status.m_pHandle, 1);
		}

		CloseHandle(status.m_pStartEvent);
		CloseHandle(status.m_pCompletedEvent);
	}

	m_bThreadsRunning = false;
}

void btWin32ThreadPool::sendRequest(int taskId, void *pArg)
{
	btAssert(m_bThreadsRunning);

	for (uint32_t i = 0; i < m_threadInfo.m_numThreads; i++)
	{
		btWin32ThreadStatus &status = m_activeThreadStatus[i];

		status.m_command = btWin32ThreadStatus::COMMAND_RUNTASK;
		status.m_pUserPointer = pArg;
		status.m_userTaskId = taskId;
		SetEvent(status.m_pStartEvent);
	}
}

bool btWin32ThreadPool::isTaskCompleted(uint32_t waitMs)
{
	btAssert(m_bThreadsRunning);

	DWORD dwRet = WaitForMultipleObjects(m_completeHandles.size(), &m_completeHandles[0], TRUE, waitMs);
	if (dwRet != WAIT_FAILED && dwRet != WAIT_TIMEOUT)
	{
		return true;
	}

	return false;
}

void btWin32ThreadPool::waitForResponse()
{
	btAssert(m_bThreadsRunning);

	DWORD dwRet = WaitForMultipleObjects(m_completeHandles.size(), &m_completeHandles[0], TRUE, INFINITE);
	btAssert(dwRet != WAIT_FAILED);
}

btCriticalSection *btWin32ThreadPool::createCriticalSection()
{
	void *mem = btAlloc(sizeof(btCriticalSection));
	return new(mem) btWin32CriticalSection();
}

void btWin32ThreadPool::destroyCriticalSection(btCriticalSection *pCritSection)
{
	pCritSection->~btCriticalSection();
	btFree(pCritSection);
}

#endif // USE_WIN32_THREADING