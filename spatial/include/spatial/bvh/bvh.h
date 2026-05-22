/**
 * @file bvh.h
 * @author khalilhenoud@gmail.com
 * @brief non-general bvh for simplistic collision detection.
 * @version 0.1
 * @date 2023-10-07
 *
 * @copyright Copyright (c) 2023
 *
 */
#ifndef SPATIAL_BVH_BVH_H
#define SPATIAL_BVH_BVH_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <spatial/internal/module.h>
#include <library/containers/cvector.h>
#include <math/vector3f.h>


////////////////////////////////////////////////////////////////////////////////
//| bvh_t, '*' = bvh
//|=============================================================================
//| OPERATION                   | SUPPORTED
//|=============================================================================
//|    *_def                    | YES
//|    *_is_def                 | YES
//|    *_replicate              |
//|    *_fullswap               |
//|    *_serialize              | YES
//|    *_deserialize            | YES
//|    *_hash                   |
//|    *_is_equal               |
//|    *_type_size              | YES
//|    *_type_alignment         |
//|    *_type_id_count          |
//|    *_type_ids               |
//|    *_owns_alloc             | YES
//|    *_get_alloc              | YES
//|    *_cleanup                | YES
////////////////////////////////////////////////////////////////////////////////

typedef struct allocator_t allocator_t;
typedef struct binary_stream_t binary_stream_t;

typedef
enum bvh_construction_method_t {
  BVH_CONSTRUCT_NAIVE,
  BVH_CONSTRUCT_COUNT
} bvh_construction_method_t;

typedef
struct bvh_aabb_t {
  point3f min_max[2];
} bvh_aabb_t;

// NOTE: left_first meaning depends on the tri_count. If tri_count is 0, the
// node is an interior node and left_first will be an offset into the node
// array, otherwise we are dealing with a leaf and left_first will be an offset
// into the triangle array.
typedef
struct bvh_node_t {
  bvh_aabb_t bounds;
  uint32_t left_first, tri_count;
} bvh_node_t;

// NOTE: the max size of node array is max: 2 * (count + 1) - 1.
// We also allocate an extra unused node that is left unused so we can adhere to
// cache alignment in the future.
typedef
struct bvh_t {
  cvector_t faces;          // face_t
  cvector_t normals;        // vector3f
  cvector_t bounds;         // bhv_aabb_t
  cvector_t nodes;          // bvh_node_t
} bvh_t;

SPATIAL_API
void
bvh_def(void *ptr);

SPATIAL_API
uint32_t
bvh_is_def(const void *ptr);

SPATIAL_API
void
bvh_serialize(
  const void *src,
  binary_stream_t *stream);

SPATIAL_API
void
bvh_deserialize(
  void *dst,
  const allocator_t *allocator,
  binary_stream_t* stream);

SPATIAL_API
size_t
bvh_type_size(void);

SPATIAL_API
uint32_t
bvh_owns_alloc(void);

SPATIAL_API
const allocator_t *
bvh_get_alloc(const void *ptr);

SPATIAL_API
void
bvh_cleanup(
  void *ptr,
  const allocator_t* allocator);

////////////////////////////////////////////////////////////////////////////////
// NOTE: we use multidimensional arrays, to take into account multiple meshes
// contributing to the bvh.
// TODO: cache align the nodes.
SPATIAL_API
bvh_t*
bvh_create(
  float **vertices,
  uint32_t **indices,
  uint32_t *indices_count,
  uint32_t multi_count,
  const allocator_t *allocator,
  bvh_construction_method_t method);

inline
uint32_t
get_max_node_count(uint32_t count)
{
  return 2 * (count + 1) - 1;
}

SPATIAL_API
void
merge_aabb(
  bvh_aabb_t* dst,
  const bvh_aabb_t* a,
  const bvh_aabb_t* b);

SPATIAL_API
void
merge_aabb_inplace(
  bvh_aabb_t* dst,
  const bvh_aabb_t* b);

SPATIAL_API
uint32_t
get_bvh_primitives_per_leaf(void);

SPATIAL_API
int32_t
bounds_intersect(const bvh_aabb_t* left, const bvh_aabb_t* right);

// NOTE: A max 256 leaf indices returned, if used is 0 no intersection.
SPATIAL_API
void
query_intersection_fixed_256(
  bvh_t* bvh,
  bvh_aabb_t* bound,
  uint32_t array[256],
  uint32_t* used);

#ifdef __cplusplus
}
#endif

#endif