#include "btThreadPool.h"

#include "LinearMath/btDefines.h"

#include <stdio.h>

static void ThreadFunc(void *pArg) {
	btThreadPoolInfo *pThreadInfo = (btThreadPoolInfo *)pArg;
	btThreadPool *pThreadPool = pThreadInfo->pThreadPool;

	while (true) {
		// Get the next task, or wait until a task is available.
		// If it returns NULL, that means the wait was cancelled because we're exiting.
		btIThreadTask *pTask = pThreadPool->getNextTask(pThreadInfo->pIdleEvent);
		
		// returns NULL if we're exiting
		if (pTask) {
			pTask->run();
			pTask->destroy();
		} else {
			break;
		}
	}
}

btThreadPool::btThreadPool() {
	m_bThreadsStarted = false;
	m_pThreadInfo = NULL;
	m_bThreadsShouldExit = false;

	m_pTaskCritSection = btCreateCriticalSection();
	m_pNewTaskCondVar = btCreateConditionalVariable();
}

btThreadPool::~btThreadPool() {
	if (m_bThreadsStarted)
		stopThreads();

	btDeleteCriticalSection(m_pTaskCritSection);
	btDeleteConditionalVariable(m_pNewTaskCondVar);
}

void btThreadPool::startThreads(int numThreads) {
	btAssert(numThreads > 0);

	m_bThreadsStarted = true;
	m_numThreads = numThreads;

	m_pThreadInfo = (btThreadPoolInfo **)btAlloc(numThreads * sizeof(void *));

	for (int i = 0; i < numThreads; i++) {
		btIThread *pThread = btCreateThread();

		m_pThreadInfo[i] = (btThreadPoolInfo *)btAlloc(sizeof(btThreadPoolInfo));

		m_pThreadInfo[i]->threadId = i;
		m_pThreadInfo[i]->pThread = pThread;
		m_pThreadInfo[i]->pIdleEvent = btCreateEvent(true);
		m_pThreadInfo[i]->pThreadPool = this;
		pThread->setThreadFunc(ThreadFunc);

		// Set the name for debugging purposes
		char name[128];
		sprintf(name, "btThreadPool thread %d", i); // Possible buffer overflow?
		pThread->setThreadName(name);

		pThread->run(m_pThreadInfo[i]);
	}
}

void btThreadPool::stopThreads() {
	m_bThreadsStarted = false;
	m_bThreadsShouldExit = true;
	m_pNewTaskCondVar->wakeAll(); // Wake up all of the threads from their sleep and make them exit.

	for (int i = 0; i < m_numThreads; i++) {
		m_pThreadInfo[i]->pThread->waitForExit();

		btDeleteThread(m_pThreadInfo[i]->pThread);
		btDeleteEvent(m_pThreadInfo[i]->pIdleEvent);
		btFree(m_pThreadInfo[i]);
	}

	btFree(m_pThreadInfo);
}

int btThreadPool::getNumThreads() {
	return m_numThreads;
}

void btThreadPool::addTask(btIThreadTask *pTask) {
	m_pTaskCritSection->lock();

	m_taskArray.push_back(pTask);

	m_pTaskCritSection->unlock();
}

void btThreadPool::runTasks() {
	m_pTaskCritSection->lock();

	// Reset the idle events here (if threads call this, main thread may be past waitIdle already)
	for (int i = 0; i < m_numThreads; i++) {
		m_pThreadInfo[i]->pIdleEvent->reset();
	}

	m_pTaskCritSection->unlock();

	m_pNewTaskCondVar->wakeAll();
}

void btThreadPool::waitIdle() {
	if (!m_bThreadsStarted) return;

	for (int i = 0; i < m_numThreads; i++) {
		m_pThreadInfo[i]->pIdleEvent->wait();
	}
}

btIThreadTask *btThreadPool::getNextTask(btIEvent *pIdleEvent) {
	m_pTaskCritSection->lock();

	// Task array is empty! Wait until new tasks are added (or the pool is exiting)
	// While loop because cond vars are subject to spurious wakeups
	while (m_taskArray.size() <= 0) {
		// Trigger the idle event (tell everyone we're currently waiting)
		pIdleEvent->trigger();
		m_pNewTaskCondVar->wait(m_pTaskCritSection); // Wait for new tasks (and release the crit section, will be reacquired when we wake)

		if (m_bThreadsShouldExit) {
			m_pTaskCritSection->unlock();
			return NULL;
		}
	}

	btAssert(m_taskArray.size() > 0);

	// Send the new task back.
	// Although we should never reach here if the task array size is 0, check anyways
	btIThreadTask *ret = m_taskArray[m_taskArray.size() - 1];
	m_taskArray.pop_back();

	m_pTaskCritSection->unlock();

	return ret;
}
