#ifndef BT_THREADPOOL_H
#define BT_THREADPOOL_H

#include "btThreading.h"
#include "LinearMath/btAlignedObjectArray.h"

class btIThreadTask {
	public:
		virtual void run() = 0;
		virtual void destroy() {}; // Destroys this task. Called after run() (not on main thread)
};

class btThreadPool;

struct btThreadPoolInfo {
	btIThread *pThread;
	btIEvent *pIdleEvent;
	btIEvent *pStartEvent;
	btThreadPool *pThreadPool;
	int threadId;

	// Task information
	btIThreadTask **pTaskArr;
	int numTasks;
};

class btThreadPool {
	public:
		btThreadPool();
		~btThreadPool();

		void startThreads(int numThreads);
		void stopThreads();

		int getNumThreads();

		void addTask(btIThreadTask *pTask);
		void clearTasks(); // Clear task queue
		void runTasks(); // Runs the threads until task pool is empty
		void waitIdle(); // Waits until thread pool is idle (no more tasks)

		// Internal functions (do not call these)

		void threadFunction(btThreadPoolInfo *pInfo);

	private:
		btAlignedObjectArray<btThreadPoolInfo *> m_pThreadInfo;
		//btThreadPoolInfo **	m_pThreadInfo;
		int					m_numThreads;
		bool				m_bThreadsStarted;
		bool				m_bThreadsShouldExit;

		btICriticalSection *m_pTaskCritSection;
		btIConditionalVariable *m_pNewTaskCondVar;
		btAlignedObjectArray<btIThreadTask *> m_taskArray; // FIXME: We don't need an aligned array.
};

#endif // BT_THREADPOOL_H