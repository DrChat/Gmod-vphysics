/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2006 Erwin Coumans  http://continuousphysics.com/Bullet/

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose,
including commercial applications, and to alter it and redistribute it freely,
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

#include "btAlignedAllocator.h"

#if defined(_MSC_VER) && defined(BT_DEBUG_MEMORY_ALLOCATIONS)
	#include <crtdbg.h>
#endif

int gNumAlignedAllocs = 0;
int gNumAlignedFree = 0;
int gTotalBytesAlignedAllocs = 0;//detect memory leaks

/************************
* ALLOCATOR FUNCTIONS
************************/

#if defined(BT_DEBUG_MEMORY_ALLOCATIONS)

static void *btDbgAllocDefault(size_t size, int blockType, const char *pFile, int line)
{
	return _malloc_dbg(size, blockType, pFile, line);
}

static void btDbgFreeDefault(void *ptr, int blockType)
{
	return _free_dbg(ptr, blockType);
}

static btDbgAllocFunc *sDbgAllocFunc = btDbgAllocDefault;
static btDbgFreeFunc *sDbgFreeFunc = btDbgFreeDefault;

#else

static void *btAllocDefault(size_t size)
{
	return malloc(size);
}

static void btFreeDefault(void *ptr)
{
	free(ptr);
}

static btAllocFunc *sAllocFunc = btAllocDefault;
static btFreeFunc *sFreeFunc = btFreeDefault;

#endif

/*****************************
* ALIGNED ALLOCATOR FUNCTIONS
*****************************/

#if defined(BT_DEBUG_MEMORY_ALLOCATIONS)

static inline void *btDbgAlignedAllocDefault(size_t size, int alignment, int blockType, const char *fileName, int line)
{
	void *ret;
	char *real;
	real = (char *)sDbgAllocFunc(size + sizeof(void *) + (alignment-1), blockType, fileName, line);

	if (real)
	{
		ret = btAlignPointer(real + sizeof(void *), alignment);
		*((void **)(ret)-1) = (void *)(real);
	}
	else
	{
		ret = (void *)(real);
	}

	return (ret);
}

static inline void btDbgAlignedFreeDefault(void *ptr, int blockType)
{
	void* real;

	if (ptr) {
		real = *((void **)(ptr)-1);
		sDbgFreeFunc(real, blockType);
	}
}

static btDbgAlignedAllocFunc *sDbgAlignedAllocFunc = btDbgAlignedAllocDefault;
static btDbgAlignedFreeFunc *sDbgAlignedFreeFunc = btDbgAlignedFreeDefault;

#else // !BT_DEBUG_MEMORY_ALLOCATIONS

#if defined (BT_HAS_ALIGNED_ALLOCATOR)

#include <malloc.h>
static void *btAlignedAllocDefault(size_t size, int alignment)
{
	return _aligned_malloc(size, (size_t)alignment);
}

static void btAlignedFreeDefault(void *ptr)
{
	_aligned_free(ptr);
}
#elif defined(__CELLOS_LV2__)
#include <stdlib.h>

static inline void *btAlignedAllocDefault(size_t size, int alignment)
{
	return memalign(alignment, size);
}

static inline void btAlignedFreeDefault(void *ptr)
{
	free(ptr);
}

#else // !BT_HAS_ALIGNED_ALLOCATOR && !__CELLOS_LV2__

static inline void *btAlignedAllocDefault(size_t size, int alignment)
{
	void *ret;
	char *real;
	real = (char *)sAllocFunc(size + sizeof(void *) + (alignment-1));

	if (real)
	{
		ret = btAlignPointer(real + sizeof(void *), alignment);
		*((void **)(ret)-1) = (void *)(real);
	}
	else
	{
		ret = (void *)(real);
	}

	return (ret);
}

static inline void btAlignedFreeDefault(void *ptr)
{
	void* real;

	if (ptr) {
		real = *((void **)(ptr)-1);
		sFreeFunc(real);
	}
}

#endif // !BT_HAS_ALIGNED_ALLOCATOR && !__CELLOS_LV2__

#endif // !BT_DEBUG_MEMORY_ALLOCATIONS

// this generic allocator provides the total allocated number of bytes

#ifdef BT_DEBUG_MEMORY_ALLOCATIONS
// Debug versions of the aligned allocator for CRT debugging, etc.

void *btDbgAlignedAllocInternal(size_t size, int alignment, int blockType, const char *fileName, int line)
{
	gNumAlignedAllocs++;
	void *ptr;
	ptr = sDbgAlignedAllocFunc(size, alignment, blockType, fileName, line);
	return ptr;
}

void btDbgAlignedFreeInternal(void *ptr, int blockType)
{
	if (!ptr)
		return;

	gNumAlignedFree++;
	sDbgAlignedFreeFunc(ptr, blockType);
}

#else // BT_DEBUG_MEMORY_ALLOCATIONS

static btAlignedAllocFunc *sAlignedAllocFunc = btAlignedAllocDefault;
static btAlignedFreeFunc *sAlignedFreeFunc = btAlignedFreeDefault;

void btAlignedAllocSetCustomAligned(btAlignedAllocFunc *allocFunc, btAlignedFreeFunc *freeFunc)
{
	sAlignedAllocFunc = allocFunc ? allocFunc : btAlignedAllocDefault;
	sAlignedFreeFunc = freeFunc ? freeFunc : btAlignedFreeDefault;
}

void btAlignedAllocSetCustom(btAllocFunc *allocFunc, btFreeFunc *freeFunc)
{
	sAllocFunc = allocFunc ? allocFunc : btAllocDefault;
	sFreeFunc = freeFunc ? freeFunc : btFreeDefault;
}

void *btAlignedAllocInternal(size_t size, int alignment)
{
	gNumAlignedAllocs++;
	void* ptr;
	ptr = sAlignedAllocFunc(size, alignment);
//	printf("btAlignedAllocInternal %d, %x\n", size, ptr);
	return ptr;
}

void btAlignedFreeInternal(void* ptr)
{
	if (!ptr)
	{
		return;
	}

	gNumAlignedFree++;
//	printf("btAlignedFreeInternal %x\n", ptr);
	sAlignedFreeFunc(ptr);
}

#endif