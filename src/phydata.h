#ifndef PHYDATA_H
#define PHYDATA_H

// Various structures used in ivp mesh parsing.
#include <tier1/byteswap.h>

// 12 bytes
struct collideheader_t {
	int		size;
	int		vphysicsID;		// Generally the ASCII for "VPHY" in newer files
	short	version;
	short	modelType;
};

// 20 bytes
struct compactsurfaceheader_t {
	int		surfaceSize;
	Vector	dragAxisAreas;
	int		axisMapSize;
};

// 4 bytes
struct moppsurfaceheader_t {
	int		moppSize;
};

// 48 bytes
// Just like a btCompoundShape.
struct ivpcompactsurface_t {
	float	mass_center[3];
	float	rotation_inertia[3];
	float	upper_limit_radius;
	int		max_deviation : 8;
	int		byte_size : 24;
	int		offset_ledgetree_root;
	int		dummy[3]; 			// dummy[2] is "IVPS" or 0

	void byteSwap() {
		for (int i = 0; i < 3; i++) {
			// FIXME: Bullet returns an int because the OS may "fix" invalid floats! We need to do the same.
			mass_center[i] = (float)btSwapEndianFloat(mass_center[i]);
			rotation_inertia[i] = (float)btSwapEndianFloat(rotation_inertia[i]);
		}

		upper_limit_radius = (float)btSwapEndianFloat(upper_limit_radius);
		// FIXME: How do we swap max_deviation and byte_size?
	}

	// Recursively byteswap through the ledge tree
	void byteSwapAll() {
		
	}
};

struct ivpcompactmopp_t {
	float	mass_center[3];
	float	rotation_inertia[3];
	float	upper_limit_radius;
	int		max_deviation : 8;
	int		byte_size : 24;
	int		offset_ledgetree_root; // offset to root node of internal ledgetree
	int		offset_ledges; // offset to the ledges
	int		size_convex_hull;
	int		dummy; // 16byte memory align
};

// 16 bytes
// Just like a btTriangleMesh (although we use btConvexHullShape)
struct ivpcompactledge_t {
	int		c_point_offset; // byte offset from 'this' to (ledge) point array
	union {
		int	ledgetree_node_offset;
		int	client_data;	// if indicates a non terminal ledge
	};
	uint	has_chilren_flag:2;
	int		is_compact_flag:2;  // if false than compact ledge uses points outside this piece of memory
	uint	dummy:4;
	uint	size_div_16:24; 
	short	n_triangles;
	short	for_future_use;
};

// 4 bytes
struct ivpcompactedge_t {
	uint	start_point_index:16;		// point index
	int		opposite_index:15;			// rel to this // maybe extra array, 3 bits more than tri_index/pierce_index
	uint	is_virtual:1;
};

// 16 bytes (4 bytes + 12 bytes edge array)
struct ivpcompacttriangle_t {
	uint	tri_index : 12; // used for upward navigation
	uint	pierce_index : 12;
	uint	material_index : 7;
	uint	is_virtual : 1;
	ivpcompactedge_t c_three_edges[3];
};

// 18 bytes
// IVP has a ledge tree after the vertex data in every solid.
// If this is terminal, offset_compact_ledge is a ledge. Otherwise, it is shape hull.
struct ivpcompactledgenode_t {
	int		offset_right_node; // (if != 0 than children
	int		offset_compact_ledge; // (if != 0, pointer to hull that contains all subelements
	float	center[3];	// in object_coords
	float	radius; // size of sphere
	unsigned char box_sizes[3];
	unsigned char free_0;

	// Functions

	const ivpcompactledge_t *GetCompactLedge() const {
		Assert(this->offset_right_node == 0);
		return (ivpcompactledge_t *)((char *)this + this->offset_compact_ledge);
	}

	const ivpcompactledgenode_t *GetLeftSon() const {
		Assert(this->offset_right_node);
		return this + 1;
	}

	const ivpcompactledgenode_t *GetRightSon() const {
		Assert(this->offset_right_node);
		return (ivpcompactledgenode_t *)((char *)this + this->offset_right_node);
	}

	bool IsTerminal() const {
		return (this->offset_right_node == 0);
	}

	const ivpcompactledge_t *GetCompactHull() const {
		if (this->offset_compact_ledge)
			return (ivpcompactledge_t *)((char *)this + this->offset_compact_ledge);
		else
			return NULL;
	}
};

#endif // PHYDATA_H
