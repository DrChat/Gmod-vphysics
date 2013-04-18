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

#ifndef BT_ITHREADPOOL_H
#define BT_ITHREADPOOL_H

#include <LinearMath/btDefines.h>
#include "PlatformDefinitions.h"

typedef int (*btThreadFunc)(int taskId, void *pArg);

struct btThreadPoolInfo
{
	btThreadPoolInfo()
	{
		m_pUniqueName = NULL;
		m_threadFunc = NULL;
		m_numThreads = 0;
		m_threadStackSize = 0;
	}

	btThreadPoolInfo(const char *uniqueName, btThreadFunc threadFunc, uint32_t numThreads, uint32_t threadStackSize = 65535) :
		m_pUniqueName(uniqueName),
		m_threadFunc(threadFunc),
		m_numThreads(numThreads),
		m_threadStackSize(threadStackSize)
	{
	}

	const char *	m_pUniqueName; // Mainly for debugging.
	btThreadFunc	m_threadFunc;
	uint32_t		m_numThreads;
	uint32_t		m_threadStackSize;
};

class btCriticalSection
{
	public:
		virtual ~btCriticalSection();

		// tries to lock the critical section, returns false if it couldn't acquire the lock.
		virtual bool trylock();
		virtual void lock();
		virtual void unlock();
};

// this critical section class will only last through function scope
// useful for functions that may encounter an exception and need this
// critical section to be unlocked
class btScopeCriticalSection
{
	public:
		btScopeCriticalSection(btCriticalSection *critSection) :
			m_critSection(critSection)
		{
			m_critSection->lock();
		}

		~btScopeCriticalSection()
		{
			m_critSection->unlock();
		}

	private:
		btCriticalSection *m_critSection;
};

class btIThreadPool
{
	public:
		virtual ~btIThreadPool();

		virtual void		startThreads(const btThreadPoolInfo &info) = 0;
		virtual void		stopThreads() = 0;

		// send a request, optionally with your own supplied task id that'll be passed through
		// to your thread function
		virtual void		sendRequest(int taskId, void *arg) = 0;

		// waits the specified milliseconds to see if all threads have completed their task (specify 0 for no wait)
		virtual bool		isTaskCompleted(uint32_t waitMs) = 0;
		// blocks until all of the threads have completed the task.
		virtual void		waitForResponse() = 0;

		virtual uint32_t	getNumThreads() = 0;

		virtual const btThreadPoolInfo &getThreadPoolInfo() const = 0;

		// These really shouldn't be here
		virtual btCriticalSection *createCriticalSection() = 0;
		virtual void destroyCriticalSection(btCriticalSection *pCritSection) = 0;
};

#endif // BT_ITHREADPOOL_H