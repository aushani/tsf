#include <stdio.h>
#include <stdlib.h>
#include <inttypes.h>
#include <glib.h>

#include "cache.h"

struct _cache {
    GTree *tree;
    CacheValueCopyFunc value_copy_func;
    CacheValueDestroyFunc value_destroy_func; // CacheValueDestroyFunc == GDestroyNotify
    size_t max_nnodes;
    size_t refcount;
    GStaticMutex mutex;
};

typedef struct cache_node cache_node_t;
struct cache_node {
    gpointer value;
    size_t   refcount;
    GDestroyNotify value_destroy_func;
};

typedef struct cache_search cache_search_t;
struct cache_search {
    int64_t key;
    size_t  refcount;
};

static gint
_cache_key_compare (gconstpointer a, gconstpointer b, gpointer user_data)
{
    const int64_t *keyA=a, *keyB=b;
    if (*keyA > *keyB)
        return 1;
    else if (*keyA == *keyB)
        return 0;
    else
        return -1;
}

static gboolean
_cache_search_func (gpointer key, gpointer value, gpointer data)
{
    cache_search_t *oldest = data;
    cache_node_t *node = value;
    if (node->refcount < oldest->refcount) {
        oldest->refcount = node->refcount;
        oldest->key = *((int64_t*)key);        
    }
    return FALSE;
}

static void
_cache_key_destroy (gpointer key)
{
    free (key);
}

static void
_cache_node_destroy (gpointer data)
{
    cache_node_t *node = data;
    node->value_destroy_func (node->value);
    free (node);
}

cache_t *
cache_new (size_t max_nnodes, CacheValueCopyFunc value_copy_func, CacheValueDestroyFunc value_destroy_func)
{
    cache_t *cache = malloc (sizeof (*cache));
    cache->tree = g_tree_new_full (&_cache_key_compare, NULL, &_cache_key_destroy, &_cache_node_destroy);
    cache->value_destroy_func = value_destroy_func;
    cache->value_copy_func = value_copy_func;
    cache->max_nnodes = max_nnodes;
    cache->refcount = 0;
    g_static_mutex_init (&cache->mutex);
    return cache;
}

void
cache_destroy (cache_t *cache)
{
    g_static_mutex_lock (&cache->mutex);
    g_tree_destroy (cache->tree);
    g_static_mutex_unlock (&cache->mutex);
    g_static_mutex_free (&cache->mutex);
    free (cache);
}

size_t
cache_nnodes (cache_t *cache)
{
    g_static_mutex_lock (&cache->mutex);
    size_t nnodes = g_tree_nnodes (cache->tree);
    g_static_mutex_unlock (&cache->mutex);
    return nnodes;
}

size_t
cache_maxnodes (cache_t *cache)
{
    return cache->max_nnodes;
}

void
cache_push (cache_t *cache, int64_t key, void *value)
{
    // key
    int64_t *k = malloc (sizeof (*k));
    *k = key;

    // node
    cache_node_t *node = malloc (sizeof (*node));
    node->value = value;
    node->refcount = ++cache->refcount;
    node->value_destroy_func = cache->value_destroy_func;

    // push key/node onto tree and check if anything should fall off
    // the tree, prioritized by last use
    g_static_mutex_lock (&cache->mutex);
    g_tree_insert (cache->tree, k, node);
    size_t nnodes = g_tree_nnodes (cache->tree);
    while (nnodes > cache->max_nnodes) {
        cache_search_t oldest = {.refcount = cache->refcount};
        g_tree_foreach (cache->tree, &_cache_search_func, &oldest);
        g_tree_remove (cache->tree, &oldest.key);
        nnodes--;
    }
    g_static_mutex_unlock (&cache->mutex);
}

void *
cache_pop (cache_t *cache, int64_t key)
{
    void *ret_value = NULL;
    g_static_mutex_lock (&cache->mutex);
    cache_node_t *node = g_tree_lookup (cache->tree, &key);
    if (node) {// found it
        node->refcount = ++cache->refcount;
        if (cache->value_copy_func)
            ret_value = cache->value_copy_func (node->value);
        else
            ret_value = node->value;
    } 
    g_static_mutex_unlock (&cache->mutex);
    return ret_value;
}
