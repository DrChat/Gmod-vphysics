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

#ifndef BT_WIN32_THREAD_POOL_H
#define BT_WIN32_THREAD_POOL_H

#include "btIThreadPool.h"
#include <LinearMath/btAlignedObjectArray.h>

#ifdef USE_WIN32_THREADING

struct btWin32ThreadStatus
{
	enum Status
	{
		STATUS_INVALID = -1,
		STATUS_READY = 0,	// Ready to do any tasks
		STATUS_BUSY,		// Currently processing a task
		STATUS_FINISHED		// Thread is closing/has been closed.
	};

	enum Command
	{
		COMMAND_INVALID = -1,
		COMMAND_RUNTASK = 0,
		COMMAND_EXIT,
	};

	btThreadFunc	m_threadFunc;
	void *			m_pUserPointer;
	int				m_userTaskId;

	int				m_status;
	int				m_command;	// Internal command (exit, run task, etc...)

	void *			m_pHandle;
	void *			m_pStartEvent;
	void *			m_pCompletedEvent;
};

class btWin32ThreadPool : public btIThreadPool
{
	public:
		btWin32ThreadPool();
		~btWin32ThreadPool();

		// Starts threads with the specified construction info
		void startThreads(const btThreadPoolInfo &info);
		void stopThreads();

		// send a request, optionally with your own supplied task id that'll be passed through
		// to your thread function
		void sendRequest(int taskId, void *arg);

		bool isTaskCompleted(uint32_t waitMs);
		void waitForResponse();

		uint32_t getNumThreads()
		{
			return m_threadInfo.m_numThreads;
		}

		const btThreadPoolInfo &getThreadPoolInfo() const
		{
			return m_threadInfo;
		}

		// These really shouldn't be here
		btCriticalSection *createCriticalSection();
		void destroyCriticalSection(btCriticalSection *pCritSection);
	private:
		// FIXME: We shouldn't need an aligned object array here.
		btAlignedObjectArray<btWin32ThreadStatus>	m_activeThreadStatus;
		btAlignedObjectArray<void *>				m_completeHandles;

		btThreadPoolInfo				m_threadInfo;
		bool							m_bThreadsRunning;
};

#endif // USE_WIN32_THREADING

#endif // BT_WIN32_THREAD_POOL_H