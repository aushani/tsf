#include <assert.h>
#include <gsl/gsl_vector.h>
#include <gsl/gsl_matrix.h>
#include <gsl/gsl_blas.h>
#include <gsl/gsl_complex_math.h>

#include "gsl_util_blas.h"

int
gslu_blas_mmm (gsl_matrix *D, const gsl_matrix *A, const gsl_matrix *B, const gsl_matrix *C, gsl_matrix *_work)
{
    assert (D->size1 == A->size1 && D->size2 == C->size2 &&
            A->size2 == B->size1 && B->size2 == C->size1);
    
    gsl_matrix *work = _work;
    if (work)
        assert (work->size1 == A->size1 && work->size2 == B->size2);
    else
        work = gsl_matrix_alloc (A->size1, B->size2);

    gsl_blas_dgemm (CblasNoTrans, CblasNoTrans, 1.0, A, B, 0.0, work);
    int ret = gsl_blas_dgemm (CblasNoTrans, CblasNoTrans, 1.0, work, C, 0.0, D);
    if (work != _work)
        gsl_matrix_free (work);

    return ret;
}

int
gslu_blas_mmmT (gsl_matrix *D, const gsl_matrix *A, const gsl_matrix *B, const gsl_matrix *C, gsl_matrix *_work)
{
    assert (D->size1 == A->size1 && D->size2 == C->size1 && 
            A->size2 == B->size1 && B->size2 == C->size2);

    gsl_matrix *work = _work;
    if (work)
        assert (work->size1 == A->size1 && work->size2 == B->size2);
    else
        work = gsl_matrix_alloc (A->size1, B->size2);

    gsl_blas_dgemm (CblasNoTrans, CblasNoTrans, 1.0, A, B, 0.0, work);
    int ret = gsl_blas_dgemm (CblasNoTrans, CblasTrans, 1.0, work, C, 0.0, D);
    if (work != _work)
        gsl_matrix_free (work);

    return ret;
}


int
gslu_blas_mmTm (gsl_matrix *D, const gsl_matrix *A, const gsl_matrix *B, const gsl_matrix *C, gsl_matrix *_work)
{
    assert (D->size1 == A->size1 && D->size2 == C->size2 && 
            A->size2 == B->size2 && B->size1 == C->size1);

    gsl_matrix *work = _work;
    if (work)
        assert (work->size1 == A->size1 && work->size2 == B->size1);
    else
        work = gsl_matrix_alloc (A->size1, B->size1);

    gsl_blas_dgemm (CblasNoTrans, CblasTrans, 1.0, A, B, 0.0, work);
    int ret = gsl_blas_dgemm (CblasNoTrans, CblasNoTrans, 1.0, work, C, 0.0, D);
    if (work != _work)
        gsl_matrix_free (work);

    return ret;
}

int
gslu_blas_mTmm (gsl_matrix *D, const gsl_matrix *A, const gsl_matrix *B, const gsl_matrix *C, gsl_matrix *_work)
{
    assert (D->size1 == A->size2 && D->size2 == C->size2 && 
            A->size1 == B->size1 && B->size2 == C->size1);

    gsl_matrix *work = _work;
    if (work)
        assert (work->size1 == A->size2 && work->size2 == B->size2);
    else
        work = gsl_matrix_alloc (A->size2, B->size2);

    gsl_blas_dgemm (CblasTrans, CblasNoTrans, 1.0, A, B, 0.0, work);
    int ret = gsl_blas_dgemm (CblasNoTrans, CblasNoTrans, 1.0, work, C, 0.0, D);
    if (work != _work)
        gsl_matrix_free (work);

    return ret;
}

int
gslu_blas_mmTmT (gsl_matrix *D, const gsl_matrix *A, const gsl_matrix *B, const gsl_matrix *C, gsl_matrix *_work)
{
    assert (D->size1 == A->size1 && D->size2 == C->size1 && 
            A->size2 == B->size2 && B->size1 == C->size2);

    gsl_matrix *work = _work;
    if (work)
        assert (work->size1 == A->size1 && work->size2 == B->size1);
    else
        work = gsl_matrix_alloc (A->size1, B->size1);

    gsl_blas_dgemm (CblasNoTrans, CblasTrans, 1.0, A, B, 0.0, work);
    int ret = gsl_blas_dgemm (CblasNoTrans, CblasTrans, 1.0, work, C, 0.0, D);
    if (work != _work)
        gsl_matrix_free (work);

    return ret;
}

int
gslu_blas_mTmTm (gsl_matrix *D, const gsl_matrix *A, const gsl_matrix *B, const gsl_matrix *C, gsl_matrix *_work)
{
    assert (D->size1 == A->size2 && D->size2 == C->size2 && 
            A->size1 == B->size2 && B->size1 == C->size1);

    gsl_matrix *work = _work;
    if (work)
        assert (work->size1 == A->size2 && work->size2 == B->size1);
    else
        work = gsl_matrix_alloc (A->size2, B->size1);

    gsl_blas_dgemm (CblasTrans, CblasTrans, 1.0, A, B, 0.0, work);
    int ret = gsl_blas_dgemm (CblasNoTrans, CblasNoTrans, 1.0, work, C, 0.0, D);
    if (work != _work)
        gsl_matrix_free (work);

    return ret;
}

int
gslu_blas_mTmmT (gsl_matrix *D, const gsl_matrix *A, const gsl_matrix *B, const gsl_matrix *C, gsl_matrix *_work)
{
    assert (D->size1 == A->size2 && D->size2 == C->size1 && 
            A->size1 == B->size1 && B->size2 == C->size2);

    gsl_matrix *work = _work;
    if (work)
        assert (work->size1 == A->size2 && work->size2 == B->size2);
    else
        work = gsl_matrix_alloc (A->size2, B->size2);

    gsl_blas_dgemm (CblasTrans, CblasNoTrans, 1.0, A, B, 0.0, work);
    int ret = gsl_blas_dgemm (CblasNoTrans, CblasTrans, 1.0, work, C, 0.0, D);
    if (work != _work)
        gsl_matrix_free (work);

    return ret;
}

int
gslu_blas_mTmTmT (gsl_matrix *D, const gsl_matrix *A, const gsl_matrix *B, const gsl_matrix *C, gsl_matrix *_work)
{
    assert (D->size1 == A->size2 && D->size2 == C->size1 && 
            A->size1 == B->size2 && B->size1 == C->size2);

    gsl_matrix *work = _work;
    if (work)
        assert (work->size1 == A->size2 && work->size2 == B->size1);
    else
        work = gsl_matrix_alloc (A->size2, B->size1);

    gsl_blas_dgemm (CblasTrans, CblasTrans, 1.0, A, B, 0.0, work);
    int ret = gsl_blas_dgemm (CblasNoTrans, CblasTrans, 1.0, work, C, 0.0, D);
    if (work != _work)
        gsl_matrix_free (work);

    return ret;
}

int
gslu_blas_vvT (gsl_matrix *C, const gsl_vector *a, const gsl_vector *b)
{
    // used to require a and b same size
    //assert (C->size1 == C->size2 && a->size == b->size && C->size1 == a->size);
    assert (C->size1 == a->size && C->size2 == b->size);

    const size_t M = C->size1;
    const size_t N = C->size2;
    const size_t MA = a->size;
    //const size_t NA = 1;
    const size_t MB = b->size;
    //const size_t NB = 1;

    if (M == MA && N == MB) {  /* [MxN] = [MAxNA][MBxNB]^T = [MAxNA][NBxMB] */
        cblas_dgemm (CblasRowMajor, CblasNoTrans, CblasTrans, M, N, 1,
                     1.0, a->data, a->stride, b->data, b->stride, 0.0,
                     C->data, C->tda);
        return GSL_SUCCESS;
    }
    else
        GSL_ERROR ("invalid length", GSL_EBADLEN);
}

int
gslu_blas_mmm_complex (gsl_matrix_complex *D, const gsl_matrix_complex *A, const gsl_matrix_complex *B, 
                  const gsl_matrix_complex *C, gsl_matrix_complex *_work)
{
    assert (D->size1 == A->size1 && D->size2 == C->size2 &&
            A->size2 == B->size1 && B->size2 == C->size1);
    
    gsl_complex alpha = gsl_complex_rect (1.0, 0.0);
    gsl_complex beta = gsl_complex_rect (0.0, 0.0);

    gsl_matrix_complex *work = _work;
    if (work)
        assert (work->size1 == A->size1 && work->size2 == B->size2);
    else
        work = gsl_matrix_complex_alloc (A->size1, B->size2);

    gsl_blas_zgemm (CblasNoTrans, CblasNoTrans, alpha, A, B, beta, work);
    int ret = gsl_blas_zgemm (CblasNoTrans, CblasNoTrans, alpha, work, C, beta, D);
    if (work != _work)
        gsl_matrix_complex_free (work);

    return ret;
}

int
gslu_blas_mmmH_complex (gsl_matrix_complex *D, const gsl_matrix_complex *A, const gsl_matrix_complex *B, 
                   const gsl_matrix_complex *C, gsl_matrix_complex *_work)
{
    assert (D->size1 == A->size1 && D->size2 == C->size1 && 
            A->size2 == B->size1 && B->size2 == C->size2);

    gsl_complex alpha = gsl_complex_rect (1.0, 0.0);
    gsl_complex beta = gsl_complex_rect (0.0, 0.0);

    gsl_matrix_complex *work = _work;
    if (work)
        assert (work->size1 == A->size1 && work->size2 == B->size2);
    else
        work = gsl_matrix_complex_alloc (A->size1, B->size2);

    gsl_blas_zgemm (CblasNoTrans, CblasNoTrans, alpha, A, B, beta, work);
    int ret = gsl_blas_zgemm (CblasNoTrans, CblasConjTrans, alpha, work, C, beta, D);
    if (work != _work)
        gsl_matrix_complex_free (work);

    return ret;
}


int
gslu_blas_mmHm_complex (gsl_matrix_complex *D, const gsl_matrix_complex *A, const gsl_matrix_complex *B,
                   const gsl_matrix_complex *C, gsl_matrix_complex *_work)
{
    assert (D->size1 == A->size1 && D->size2 == C->size2 && 
            A->size2 == B->size2 && B->size1 == C->size1);

    gsl_complex alpha = gsl_complex_rect (1.0, 0.0);
    gsl_complex beta = gsl_complex_rect (0.0, 0.0);

    gsl_matrix_complex *work = _work;
    if (work)
        assert (work->size1 == A->size1 && work->size2 == B->size1);
    else
        work = gsl_matrix_complex_alloc (A->size1, B->size1);

    gsl_blas_zgemm (CblasNoTrans, CblasConjTrans, alpha, A, B, beta, work);
    int ret = gsl_blas_zgemm (CblasNoTrans, CblasNoTrans, alpha, work, C, beta, D);
    if (work != _work)
        gsl_matrix_complex_free (work);

    return ret;
}

int
gslu_blas_mHmm_complex (gsl_matrix_complex *D, const gsl_matrix_complex *A, const gsl_matrix_complex *B,
                   const gsl_matrix_complex *C, gsl_matrix_complex *_work)
{
    assert (D->size1 == A->size2 && D->size2 == C->size2 && 
            A->size1 == B->size1 && B->size2 == C->size1);

    gsl_complex alpha = gsl_complex_rect (1.0, 0.0);
    gsl_complex beta = gsl_complex_rect (0.0, 0.0);

    gsl_matrix_complex *work = _work;
    if (work)
        assert (work->size1 == A->size2 && work->size2 == B->size2);
    else
        work = gsl_matrix_complex_alloc (A->size2, B->size2);

    gsl_blas_zgemm (CblasConjTrans, CblasNoTrans, alpha, A, B, beta, work);
    int ret = gsl_blas_zgemm (CblasNoTrans, CblasNoTrans, alpha, work, C, beta, D);
    if (work != _work)
        gsl_matrix_complex_free (work);

    return ret;
}

int
gslu_blas_mmHmH_complex (gsl_matrix_complex *D, const gsl_matrix_complex *A, const gsl_matrix_complex *B,
                    const gsl_matrix_complex *C, gsl_matrix_complex *_work)
{
    assert (D->size1 == A->size1 && D->size2 == C->size1 && 
            A->size2 == B->size2 && B->size1 == C->size2);

    gsl_complex alpha = gsl_complex_rect (1.0, 0.0);
    gsl_complex beta = gsl_complex_rect (0.0, 0.0);

    gsl_matrix_complex *work = _work;
    if (work)
        assert (work->size1 == A->size1 && work->size2 == B->size1);
    else
        work = gsl_matrix_complex_alloc (A->size1, B->size1);

    gsl_blas_zgemm (CblasNoTrans, CblasConjTrans, alpha, A, B, beta, work);
    int ret = gsl_blas_zgemm (CblasNoTrans, CblasConjTrans, alpha, work, C, beta, D);
    if (work != _work)
        gsl_matrix_complex_free (work);

    return ret;
}


int
gslu_blas_mHmHm_complex (gsl_matrix_complex *D, const gsl_matrix_complex *A, const gsl_matrix_complex *B,
                    const gsl_matrix_complex *C, gsl_matrix_complex *_work)
{
    assert (D->size1 == A->size2 && D->size2 == C->size2 && 
            A->size1 == B->size2 && B->size1 == C->size1);

    gsl_complex alpha = gsl_complex_rect (1.0, 0.0);
    gsl_complex beta = gsl_complex_rect (0.0, 0.0);

    gsl_matrix_complex *work = _work;
    if (work)
        assert (work->size1 == A->size2 && work->size2 == B->size1);
    else
        work = gsl_matrix_complex_alloc (A->size2, B->size1);

    gsl_blas_zgemm (CblasConjTrans, CblasConjTrans, alpha, A, B, beta, work);
    int ret = gsl_blas_zgemm (CblasNoTrans, CblasNoTrans, alpha, work, C, beta, D);
    if (work != _work)
        gsl_matrix_complex_free (work);

    return ret;
}


int
gslu_blas_mHmmH_complex (gsl_matrix_complex *D, const gsl_matrix_complex *A, const gsl_matrix_complex *B,
                    const gsl_matrix_complex *C, gsl_matrix_complex *_work)
{
    assert (D->size1 == A->size2 && D->size2 == C->size1 && 
            A->size1 == B->size1 && B->size2 == C->size2);

    gsl_complex alpha = gsl_complex_rect (1.0, 0.0);
    gsl_complex beta = gsl_complex_rect (0.0, 0.0);

    gsl_matrix_complex *work = _work;
    if (work)
        assert (work->size1 == A->size2 && work->size2 == B->size2);
    else
        work = gsl_matrix_complex_alloc (A->size2, B->size2);

    gsl_blas_zgemm (CblasConjTrans, CblasNoTrans, alpha, A, B, beta, work);
    int ret = gsl_blas_zgemm (CblasNoTrans, CblasConjTrans, alpha, work, C, beta, D);
    if (work != _work)
        gsl_matrix_complex_free (work);

    return ret;
}

int
gslu_blas_mHmHmH_complex (gsl_matrix_complex *D, const gsl_matrix_complex *A, const gsl_matrix_complex *B,
                     const gsl_matrix_complex *C, gsl_matrix_complex *_work)
{
    assert (D->size1 == A->size2 && D->size2 == C->size1 && 
            A->size1 == B->size2 && B->size1 == C->size2);

    gsl_complex alpha = gsl_complex_rect (1.0, 0.0);
    gsl_complex beta = gsl_complex_rect (0.0, 0.0);

    gsl_matrix_complex *work = _work;
    if (work)
        assert (work->size1 == A->size2 && work->size2 == B->size1);
    else
        work = gsl_matrix_complex_alloc (A->size2, B->size1);

    gsl_blas_zgemm (CblasConjTrans, CblasConjTrans, alpha, A, B, beta, work);
    int ret = gsl_blas_zgemm (CblasNoTrans, CblasConjTrans, alpha, work, C, beta, D);
    if (work != _work)
        gsl_matrix_complex_free (work);

    return ret;
}
