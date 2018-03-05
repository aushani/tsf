#ifndef __PERLS_MATH_MESH_H__
#define __PERLS_MATH_MESH_H__

/**
 * @defgroup PerlsMathMesh mesh
 * @brief Meshing routines for 3D surface point clouds
 * @ingroup PerlsMath
 * 
 * @{
 */

#include "gsl/gsl_matrix.h"
#include "perls-math/gsl_util_index.h"

#define MESH_QHULL_DELAUNAY_FLAGS "qhull d Qt Qbb Qc"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * triangular-based mesh.  Each triangle has three vertices: va, vb, and vc 
 */
typedef struct _trimesh_t {
    gslu_index *va;
    gslu_index *vb;
    gslu_index *vc;
} trimesh_t;

/**
 * @brief free a trimesh_t struct
 */
void
trimesh_t_free (trimesh_t *mesh);

/**
 * @brief copy a trimesh_t struct
 */
trimesh_t *
trimesh_t_copy (const trimesh_t *tri);

/**
 * @brief allocate a trimesh_t struct containing num_tri triangles
 */
trimesh_t *
trimesh_t_alloc (int num_tri);

void
trimesh_t_printf (const trimesh_t *mesh, FILE *fp);

/**
 * @brief perform n-dimensional delaunay triangulation on points.  A clone of matlab
 * 2007's delaunayn command
 *
 * @param points nXN matrix where n is dimension of data (typically 2) and N is the number of points
 *
 * @note Currently the only value of n that is supported is 2.
 */
trimesh_t *
mesh_delaunay_alloc (const gsl_matrix *points);

/**
 * @brief Select a single triangle from the mesh
 */
void
mesh_seltri (const trimesh_t *tri, const gsl_matrix *points, size_t ind, gsl_matrix *tri_points);

/**
 * @brief compute the normals for point cloud and triangular mesh
 *
 * @param points 3xN point cloud, where N is number of 3D points
 * @param tri trimesh_t containing the triangle indeces (ie, from delaunay triangulation)
 *
 * @note Currently this only supports 3D point clouds
 */
void
mesh_normals (const gsl_matrix *points, const trimesh_t *tri, gsl_matrix *normals);

/**
 * @brief allocation version of mesh_normals
 */
static inline gsl_matrix *
mesh_normals_alloc (const gsl_matrix *points, const trimesh_t *tri)
{
    gsl_matrix *normals = gsl_matrix_calloc (3, tri->va->size);
    mesh_normals (points, tri, normals);
    return normals;
}

/**
 * @brief save point cloud as obj file, which can be loaded into meshlab
 *
 * @param points 3xN point cloud, where N is number of 3D points
 * @param tri trimesh_t containing the triangle indeces (ie, from delaunay triangulation)
 * @param filename filename of obj file
 *
 * @return EXIT_SUCCESS on success, EXIT_FAILURE on failure
 */
int
mesh_save_to_obj (const gsl_matrix *points, const trimesh_t *tri, const char *filename);

/**
 * @brief load point cloud from obj file
 *
 * @param points pointer to resulting 3xN point cloud, where N is number of 3D points
 * @param tri pointer to resulting trimesh_t containing the triangle indeces
 * @param filename filename of obj file
 *
 * @return EXIT_SUCCESS on succes, EXIT_FAILURE on failure
 *
 * @note points and tri must be freed by the user
 */
int
mesh_load_from_obj_alloc (gsl_matrix **points, trimesh_t **tri, const char *filename);

#ifdef __cplusplus
}
#endif

#endif /* __PERLS_MATH_MESH_H__ */
