#ifndef __PERLS_COMMON_CACHE_H__
#define __PERLS_COMMON_CACHE_H__

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef void * (*CacheValueCopyFunc) (const void *value);

typedef void (*CacheValueDestroyFunc) (void *value);

typedef struct _cache cache_t;

/* Creates a new fixed size cache data structure useful for holding
 * items in memory for quick access.  Each time an item is accessed
 * from the cache using cache_pop(), it's expiration date gets reset.
 * When an item is pushed onto the cache and exceeds the cache's
 * maximum length, then the item with the oldest expiration date falls
 * off the cache and is destroyed using the user supplied
 * value_destroy_func().  Items are indexed by int64_t keys
 * (e.g. utime) using a balanced binary tree for efficient retrieval.
 *
 * max_nodes          - maximum number of nodes, once the cache exceeds this, least used items
 *                      are pushed off
 * value_destroy_func - function handle that frees memory associated with value
 * value_copy_func    - function handle that clones value in return of cache_pop(), set to NULL
 *                      to have the return of cache_pop() point directly at value in the cache.
 *                      Note that the value in the cache can be destroyed at any time with a 
 *                      call to cache_push(), hence why you may want to copy the data rather
 *                      than point directly at it.
 *
 * Note: in the case of value_copy_func == NULL, the user should not free the memory associated with
 *       the value returned by cache_pop(), as cache_push() will do this internally.
 */
cache_t *
cache_new (size_t max_nnodes, CacheValueCopyFunc value_copy_func, CacheValueDestroyFunc value_destroy_func);

/* Destroys the cache and purges all elements.
 */
void
cache_destroy (cache_t *cache);

/* Returns the current number of nodes in the cache
 */
size_t
cache_nnodes (cache_t *cache);

/* Returns the maximum number of nodes the cache can hold
 */
size_t
cache_maxnodes (cache_t *cache);


/* Adds value and key to the top of the cache
 */
void
cache_push (cache_t *cache, int64_t key, void *value);

/* Returns a pointer to the value associated with key and resets the key's expiration in the cache
 */
void *
cache_pop (cache_t *cache, int64_t key);


#ifdef __cplusplus
}
#endif

#endif //__PERLS_COMMON_CACHE_H__
