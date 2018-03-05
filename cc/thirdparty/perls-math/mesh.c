#include "mesh.h"
#include "gsl_util_matrix.h"

#include "perls-common/error.h"
#include <assert.h>

#include <qhull/qhull_a.h>

static coordT *
_gsl_matrix_to_coordT_alloc (const gsl_matrix *points)
{
    int dim = points->size1;
    int num_points = points->size2;

    coordT *points_coordT = calloc (dim*num_points, sizeof (*points_coordT));

    for (int col=0; col<num_points; col++)
        for (int row=0; row<dim; row++)
            points_coordT[col*dim + row] = gsl_matrix_get (points, row, col);

    return points_coordT;
}

static void
_print_summary (void) {
    facetT *facet;
    vertexT *vertex, **vertexp;
    setT *vertices;

    printf ("Found %d Delaunay Regions\n", qh num_good);

    FORALLfacets {
        if (facet->good) {
            vertices = qh_facet3vertex (facet);
            FOREACHvertex_ (vertices)
                printf ("%d ", qh_pointid (vertex->point));
            printf ("\n");
            qh_settempfree(&vertices);
        }
    }
}

static int
_get_num_delaunay_regions ()
{
    int n = 0;
    facetT *facet;

    FORALLfacets {
        if (facet->good) {
            n++;
        }
    }

    return n;
}

static void
_set_trimesh (trimesh_t **mesh)
{
    facetT *facet;
    vertexT *vertex, **vertexp;
    setT *vertices;
    int n = 0;
    int _dim = 0;

    int numGood = _get_num_delaunay_regions ();
    *mesh = trimesh_t_alloc (numGood);

    FORALLfacets {
        if (facet->good) {
            vertices = qh_facet3vertex (facet);

            _dim = 0;
            FOREACHvertex_ (vertices) {
                switch (_dim) {
                case 0:
                    gslu_index_set ((*mesh)->va, n, qh_pointid (vertex->point));
                    break;
                case 1:
                    gslu_index_set ((*mesh)->vb, n, qh_pointid (vertex->point));
                    break;
                case 2:
                    gslu_index_set ((*mesh)->vc, n, qh_pointid (vertex->point));
                    break;
                default:
                    printf ("[mesh]\tTrying to assign higher-dimensional vertex to a triangular mesh!\n");
                }
                _dim++;
            }
            n++;
            qh_settempfree(&vertices);
        }
    }
}

trimesh_t *
trimesh_t_alloc (int num_tri)
{
    trimesh_t *mesh = calloc (1, sizeof (*mesh));
    mesh->va = gslu_index_calloc (num_tri);
    mesh->vb = gslu_index_calloc (num_tri);
    mesh->vc = gslu_index_calloc (num_tri);
    return mesh;
}

void
trimesh_t_free (trimesh_t *mesh)
{
    if (mesh) {
        gslu_index_free (mesh->va);
        gslu_index_free (mesh->vb);
        gslu_index_free (mesh->vc);
    }
    free (mesh);
}

trimesh_t *
trimesh_t_copy (const trimesh_t *tri)
{
    assert (tri->va->size == tri->vb->size && tri->va->size == tri->vc->size);

    trimesh_t *copy = calloc (1, sizeof (*copy));
    copy->va = gslu_index_calloc (tri->va->size);
    copy->vb = gslu_index_calloc (tri->vb->size);
    copy->vc = gslu_index_calloc (tri->vc->size);

    gslu_index_memcpy (copy->va, tri->va);
    gslu_index_memcpy (copy->vb, tri->vb);
    gslu_index_memcpy (copy->vc, tri->vc);

    return copy;
}

void
trimesh_t_printf (const trimesh_t *mesh, FILE *fp)
{
    if (!fp)
        fp = stdout;

    int numPoints = mesh->va->size;
    for (int i=0; i<numPoints; i++)
        fprintf (fp, "%lu %lu %lu\n", gslu_index_get (mesh->va, i), gslu_index_get (mesh->vb, i), gslu_index_get (mesh->vc, i));
}

trimesh_t *
mesh_delaunay_alloc (const gsl_matrix *points)
{
    trimesh_t *del = NULL;
    int numpoints = points->size2;
    int dim = points->size1;
    coordT *points_coordT = NULL;
    boolT ismalloc = False;
    char flags[256];
    int curlong, totlong;     /* memory remaining after qh_memfreeshort */
    sprintf (flags, MESH_QHULL_DELAUNAY_FLAGS);

#if 1
    if (dim != 2) {
        printf ("[mesh]\tYou provided %dD data. Only 2D triangulation is supported!\n", dim);
        return NULL;
    }
#endif

    points_coordT = _gsl_matrix_to_coordT_alloc (points);

    /* For some reason, to get same ordering as matlab, you need to provide a non-null file to qhull! */
    FILE *fp = fopen ("/dev/null", "w");
    int exit_code = qh_new_qhull (dim, numpoints, points_coordT, ismalloc, flags, fp, NULL);
    fclose (fp);

    if (!exit_code)
        _set_trimesh (&del);

    qh_freeqhull(!qh_ALL);                 /* free long memory */
    qh_memfreeshort (&curlong, &totlong);  /* free short memory and memory allocator */
    if (curlong || totlong)
        printf ("[mesh]\tqhull internal warning: did not free %d bytes of long memory (%d pieces)\n", totlong, curlong);

    /* clean up */
    free (points_coordT);

    return del;
}

void
mesh_normals (const gsl_matrix *points, const trimesh_t *tri, gsl_matrix *normals)
{
    assert (normals->size1 == 3 && normals->size2 == tri->va->size);

    gsl_matrix *pointsA = gslu_matrix_selcol_alloc (points, tri->va);
    gsl_matrix *pointsB = gslu_matrix_selcol_alloc (points, tri->vb);
    gsl_matrix *pointsC = gslu_matrix_selcol_alloc (points, tri->vc);

    for (size_t i=0; i<tri->va->size; i++) {
        double pointAX = gsl_matrix_get (pointsA, 0, i);
        double pointAY = gsl_matrix_get (pointsA, 1, i);
        double pointAZ = gsl_matrix_get (pointsA, 2, i);

        double pointBX = gsl_matrix_get (pointsB, 0, i);
        double pointBY = gsl_matrix_get (pointsB, 1, i);
        double pointBZ = gsl_matrix_get (pointsB, 2, i);

        double pointCX = gsl_matrix_get (pointsC, 0, i);
        double pointCY = gsl_matrix_get (pointsC, 1, i);
        double pointCZ = gsl_matrix_get (pointsC, 2, i);

        double Ux = pointBX - pointAX;
        double Uy = pointBY - pointAY;
        double Uz = pointBZ - pointAZ;

        double Vx = pointCX - pointAX;
        double Vy = pointCY - pointAY;
        double Vz = pointCZ - pointAZ;

        double Nx = Uy*Vz - Uz*Vy;
        double Ny = Uz*Vx - Ux*Vz;
        double Nz = Ux*Vy - Uy*Vx;

        double lambda = sqrt (Nx*Nx + Ny*Ny + Nz*Nz);
        Nx /= lambda;
        Ny /= lambda;
        Nz /= lambda;

        gsl_matrix_set (normals, 0, i, Nx);
        gsl_matrix_set (normals, 1, i, Ny);
        gsl_matrix_set (normals, 2, i, Nz);
    }

    gsl_matrix_free (pointsA);
    gsl_matrix_free (pointsB);
    gsl_matrix_free (pointsC);
}

int
mesh_save_to_obj (const gsl_matrix *points, const trimesh_t *tri, const char *filename)
{
    FILE *fp = fopen (filename, "w");
    if (!fp) {
        PERROR ("fopen()");
        return EXIT_FAILURE;
    }

    assert (points->size1 == 3);

    /* compute normals */
    gsl_matrix *normals = mesh_normals_alloc (points, tri);

    /* header */
    char header[512];
    time_t now = time (NULL);
    sprintf (header, "# PeRL Mesh generated on %s", ctime (&now));
    fprintf (fp, "%s", header);

    /* save point cloud to file */
    for (int i=0; i<points->size2; i++) {
        fprintf (fp, "v %f %f %f\n",
                 gsl_matrix_get (points, 0, i),
                 gsl_matrix_get (points, 1, i),
                 gsl_matrix_get (points, 2, i));
    }

    /* save face normals */
    for (int i=0; i<normals->size2; i++) {
        fprintf (fp, "vn %f %f %f\n",
                 gsl_matrix_get (normals, 0, i),
                 gsl_matrix_get (normals, 1, i),
                 gsl_matrix_get (normals, 2, i));
    }

    /* save triangle and normal indeces to file */
    for (long unsigned int i=0; i<tri->va->size; i++) {
        fprintf (fp, "f %lu//%lu %lu//%lu %lu//%lu\n",
                 gslu_index_get (tri->va, i)+1,
                 i+1,
                 gslu_index_get (tri->vb, i)+1,
                 i+1,
                 gslu_index_get (tri->vc, i)+1,
                 i+1);
    }

    /* clean up */
    fclose (fp);
    gsl_matrix_free (normals);

    return EXIT_SUCCESS;
}

static void
_countInFile (FILE *fp, size_t *numVertices, size_t *numFaces, size_t *numNormals)
{
    *numVertices = 0;
    *numFaces = 0;
    *numNormals = 0;

    fseek (fp, 0, SEEK_SET);

    size_t len = 1024;
    char line[len];
    while (fgets (line, len, fp)) {
        if (strncmp (line, "v\t", 2) == 0)
            (*numVertices)++;
        else if (strncmp (line, "v ", 2) == 0)
            (*numVertices)++;
        else if (strncmp (line, "f\t", 2) == 0)
            (*numFaces)++;
        else if (strncmp (line, "f ", 2) == 0)
            (*numFaces)++;
        else if (strncmp (line, "vn\t", 3) == 0)
            (*numNormals)++;
        else if (strncmp (line, "vn ", 3) == 0)
            (*numNormals)++;
    }

    fseek (fp, 0, SEEK_SET);
}

static void
_loadVertexLine (gsl_matrix *points, char *line, int i)
{
    /* DO NOT USE SSCANF FOR FLOATS - results in rounding errors */

    char *delim = " ";
    /* "v " portion */
    char *token;
    token = strtok (line, delim);

    /* vx portion */
    token = strtok (NULL, delim);
    gsl_matrix_set (points, 0, i, atof (token));

    /* vy portion */
    token = strtok (NULL, delim);
    gsl_matrix_set (points, 1, i, atof (token));

    /* vz portion */
    token = strtok (NULL, delim);
    gsl_matrix_set (points, 2, i, atof (token));
}

static void
_loadFaceLine (trimesh_t *tri, char *line, int i)
{
    size_t faceInds[3];
    size_t dummy;
    char *format = "f %ld//%ld %ld//%ld %ld//%ld\n";

    sscanf (line, format, faceInds, &dummy, faceInds+1, &dummy, faceInds+2, &dummy);

    gslu_index_set (tri->va, i, faceInds[0]-1);
    gslu_index_set (tri->vb, i, faceInds[1]-1);
    gslu_index_set (tri->vc, i, faceInds[2]-1);
}

int
mesh_load_from_obj_alloc (gsl_matrix **points, trimesh_t **tri, const char *filename)
{
    FILE *fp = fopen (filename, "r");
    if (!fp) {
        PERROR ("fopen()");
        return EXIT_FAILURE;
    }

    size_t numVertices, numFaces, numNormals;
    _countInFile (fp, &numVertices, &numFaces, &numNormals);

    *points = gsl_matrix_calloc (3, numVertices);
    *tri = trimesh_t_alloc (numFaces);

    /* begin parsing data */
    size_t len = 1024;
    char line[len];
    size_t vertexCounter = 0;
    size_t faceCounter = 0;
    while (fgets (line, len, fp)) {

        if (strncmp (line, "#", 1) == 0)
            continue;

        if (strncmp (line, "v\t", 2) == 0
            || strncmp (line, "v ", 2) == 0)
            _loadVertexLine (*points, line, vertexCounter++);

        if (strncmp (line, "f\t", 2) == 0
            || strncmp (line, "f ", 2) == 0)
            _loadFaceLine (*tri, line, faceCounter++);

    }

    fclose (fp);

    return EXIT_SUCCESS;
}

void
mesh_seltri (const trimesh_t *tri, const gsl_matrix *points, size_t ind, gsl_matrix *tri_points)
{
    assert (points->size1 == 3);
    assert (tri_points->size1 == 3 && tri_points->size2 == tri_points->size1);

    size_t indA = gslu_index_get (tri->va, ind);
    size_t indB = gslu_index_get (tri->vb, ind);
    size_t indC = gslu_index_get (tri->vc, ind);

    gsl_vector_const_view pointA = gsl_matrix_const_column (points, indA);
    gsl_vector_const_view pointB = gsl_matrix_const_column (points, indB);
    gsl_vector_const_view pointC = gsl_matrix_const_column (points, indC);

    gsl_vector_view triPointA = gsl_matrix_column (tri_points, 0);
    gsl_vector_view triPointB = gsl_matrix_column (tri_points, 1);
    gsl_vector_view triPointC = gsl_matrix_column (tri_points, 2);

    gsl_vector_memcpy (&triPointA.vector, &pointA.vector);
    gsl_vector_memcpy (&triPointB.vector, &pointB.vector);
    gsl_vector_memcpy (&triPointC.vector, &pointC.vector);
}
