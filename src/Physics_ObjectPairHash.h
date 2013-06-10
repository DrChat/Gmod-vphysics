#ifndef PHYSICS_OBJECTPAIRHASH_H
#define PHYSICS_OBJECTPAIRHASH_H

#include <vphysics/object_hash.h>

struct pair_hash_list {
	void *object0;
	void *object1;

	pair_hash_list *previous;
	pair_hash_list *next;
};

class CPhysicsObjectPairHash : public IPhysicsObjectPairHash {
	public:
		CPhysicsObjectPairHash();

		virtual void	AddObjectPair(void *pObject0, void *pObject1);
		virtual void	RemoveObjectPair(void *pObject0, void *pObject1);
		virtual bool	IsObjectPairInHash(void *pObject0, void *pObject1);
		virtual void	RemoveAllPairsForObject(void *pObject0);
		virtual bool	IsObjectInHash(void *pObject0);

		virtual int		GetPairCountForObject(void *pObject0);
		virtual int		GetPairListForObject(void *pObject0, int nMaxCount, void **ppObjectList);

	private:
		pair_hash_list *m_pHashList[256];
};

#endif // PHYSICS_OBJECTPAIRHASH_H
