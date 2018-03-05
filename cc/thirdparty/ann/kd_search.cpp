//----------------------------------------------------------------------
// File:			kd_search.cpp
// Programmer:		Sunil Arya and David Mount
// Description:		Standard kd-tree search
// Last modified:	01/04/05 (Version 1.0)
//----------------------------------------------------------------------
// Copyright (c) 1997-2005 University of Maryland and Sunil Arya and
// David Mount.  All Rights Reserved.
//
// This software and related documentation is part of the Approximate
// Nearest Neighbor Library (ANN).  This software is provided under
// the provisions of the Lesser GNU Public License (LGPL).  See the
// file ../ReadMe.txt for further information.
//
// The University of Maryland (U.M.) and the authors make no
// representations about the suitability or fitness of this software for
// any purpose.  It is provided "as is" without express or implied
// warranty.
//----------------------------------------------------------------------
// History:
//	Revision 0.1  03/04/98
//		Initial release
//	Revision 1.0  04/01/05
//		Changed names LO, HI to ANN_LO, ANN_HI
//----------------------------------------------------------------------

#include "kd_search.h"					// kd-search declarations

//----------------------------------------------------------------------
//	Approximate nearest neighbor searching by kd-tree search
//		The kd-tree is searched for an approximate nearest neighbor.
//		The point is returned through one of the arguments, and the
//		distance returned is the squared distance to this point.
//
//		The method used for searching the kd-tree is an approximate
//		adaptation of the search algorithm described by Friedman,
//		Bentley, and Finkel, ``An algorithm for finding best matches
//		in logarithmic expected time,'' ACM Transactions on Mathematical
//		Software, 3(3):209-226, 1977).
//
//		The algorithm operates recursively.  When first encountering a
//		node of the kd-tree we first visit the child which is closest to
//		the query point.  On return, we decide whether we want to visit
//		the other child.  If the box containing the other child exceeds
//		1/(1+eps) times the current best distance, then we skip it (since
//		any point found in this child cannot be closer to the query point
//		by more than this factor.)  Otherwise, we visit it recursively.
//		The distance between a box and the query point is computed exactly
//		(not approximated as is often done in kd-tree), using incremental
//		distance updates, as described by Arya and Mount in ``Algorithms
//		for fast vector quantization,'' Proc.  of DCC '93: Data Compression
//		Conference, eds. J. A. Storer and M. Cohn, IEEE Press, 1993,
//		381-390.
//
//		The main entry points is annkSearch() which sets things up and
//		then call the recursive routine ann_search().  This is a recursive
//		routine which performs the processing for one node in the kd-tree.
//		There are two versions of this virtual procedure, one for splitting
//		nodes and one for leaves.  When a splitting node is visited, we
//		determine which child to visit first (the closer one), and visit
//		the other child on return.  When a leaf is visited, we compute
//		the distances to the points in the buckets, and update information
//		on the closest points.
//
//		Some trickery is used to incrementally update the distance from
//		a kd-tree rectangle to the query point.  This comes about from
//		the fact that which each successive split, only one component
//		(along the dimension that is split) of the squared distance to
//		the child rectangle is different from the squared distance to
//		the parent rectangle.
//----------------------------------------------------------------------

//----------------------------------------------------------------------
//		To keep argument lists short, a number of global variables
//		are maintained which are common to all the recursive calls.
//		These are given below.
//----------------------------------------------------------------------

/* AKU These aren't thread safe!
int				ANNkdDim;				// dimension of space
ANNpoint		ANNkdQ;					// query point
double			ANNkdMaxErr;			// max tolerable squared error
ANNpointArray	ANNkdPts;				// the points
ANNmin_k		*ANNkdPointMK;			// set of k closest points
*/

//----------------------------------------------------------------------
//	annkSearch - search for the k nearest neighbors
//----------------------------------------------------------------------

void ANNkd_tree::annkSearch(
	ANNpoint			q,				// the query point
	int					k,				// number of near neighbors to return
	ANNidxArray			nn_idx,			// nearest neighbor indices (returned)
	ANNdistArray		dd,				// the approximate nearest neighbor
	double				eps)			// the error bound
{

	/* AKU
	ANNkdDim = dim;						// copy arguments to static equivs
	ANNkdQ = q;
	ANNkdPts = pts;
	ANNptsVisited = 0;					// initialize count of points visited
	*/

	int n_pts_visited = 0; // AKU
	if (k > n_pts) {					// too many near neighbors?
		annError("Requesting more near neighbors than data points", ANNabort);
	}

	// AKU ANNkdMaxErr = ANN_POW(1.0 + eps);
	double kdMaxErr = ANN_POW(1.0 + eps);
	ANN_FLOP(2)							// increment floating op count

	//AKU ANNkdPointMK = new ANNmin_k(k);		// create set for closest k points
	ANNmin_k *aku_ANNkdPointMK = new ANNmin_k(k);
										// search starting at the root
	//AKU root->ann_search(annBoxDistance(q, bnd_box_lo, bnd_box_hi, dim));
	root->ann_search(annBoxDistance(q, bnd_box_lo, bnd_box_hi, dim), q, kdMaxErr, aku_ANNkdPointMK, pts, dim);

	for (int i = 0; i < k; i++) {		// extract the k-th closest points
		//AKU dd[i] = ANNkdPointMK->ith_smallest_key(i);
		dd[i] = aku_ANNkdPointMK->ith_smallest_key(i);
		//AKU nn_idx[i] = ANNkdPointMK->ith_smallest_info(i);
		nn_idx[i] = aku_ANNkdPointMK->ith_smallest_info(i);
	}
	//AKU delete ANNkdPointMK;				// deallocate closest point set
	delete aku_ANNkdPointMK;
}

//----------------------------------------------------------------------
//	kd_split::ann_search - search a splitting node
//----------------------------------------------------------------------

void ANNkd_split::ann_search(ANNdist box_dist, ANNpoint &kdq, double &kdMaxErr, ANNmin_k *kdPointMK, ANNpointArray &kdPts, int &dim)
{
										// check dist calc term condition
	if (ANNmaxPtsVisited != 0 && ANNptsVisited > ANNmaxPtsVisited) return;

										// distance to cutting plane
	//AKU ANNcoord cut_diff = ANNkdQ[cut_dim] - cut_val;
	ANNcoord cut_diff = kdq[cut_dim] - cut_val;

	if (cut_diff < 0) {					// left of cutting plane
		//AKU child[ANN_LO]->ann_search(box_dist);// visit closer child first
		child[ANN_LO]->ann_search(box_dist, kdq, kdMaxErr, kdPointMK, kdPts, dim);// visit closer child first

		//AKU ANNcoord box_diff = cd_bnds[ANN_LO] - ANNkdQ[cut_dim];
		ANNcoord box_diff = cd_bnds[ANN_LO] - kdq[cut_dim];
		if (box_diff < 0)				// within bounds - ignore
			box_diff = 0;
										// distance to further box
		box_dist = (ANNdist) ANN_SUM(box_dist,
				ANN_DIFF(ANN_POW(box_diff), ANN_POW(cut_diff)));

										// visit further child if close enough
		//AKU if (box_dist * ANNkdMaxErr < ANNkdPointMK->max_key())
		if (box_dist * kdMaxErr < kdPointMK->max_key())
			//AKU child[ANN_HI]->ann_search(box_dist);
			child[ANN_HI]->ann_search(box_dist, kdq, kdMaxErr, kdPointMK, kdPts, dim);

	}
	else {								// right of cutting plane
		//AKU child[ANN_HI]->ann_search(box_dist);// visit closer child first
		child[ANN_HI]->ann_search(box_dist, kdq, kdMaxErr, kdPointMK, kdPts, dim);// visit closer child first

		//AKU ANNcoord box_diff = ANNkdQ[cut_dim] - cd_bnds[ANN_HI];
		ANNcoord box_diff = kdq[cut_dim] - cd_bnds[ANN_HI];
		if (box_diff < 0)				// within bounds - ignore
			box_diff = 0;
										// distance to further box
		box_dist = (ANNdist) ANN_SUM(box_dist,
				ANN_DIFF(ANN_POW(box_diff), ANN_POW(cut_diff)));

										// visit further child if close enough
		//AKU if (box_dist * ANNkdMaxErr < ANNkdPointMK->max_key())
		if (box_dist * kdMaxErr < kdPointMK->max_key())
			//AKU child[ANN_LO]->ann_search(box_dist);
			child[ANN_LO]->ann_search(box_dist, kdq, kdMaxErr, kdPointMK, kdPts, dim);

	}
	ANN_FLOP(10)						// increment floating ops
	ANN_SPL(1)							// one more splitting node visited
}

//----------------------------------------------------------------------
//	kd_leaf::ann_search - search points in a leaf node
//		Note: The unreadability of this code is the result of
//		some fine tuning to replace indexing by pointer operations.
//----------------------------------------------------------------------

void ANNkd_leaf::ann_search(ANNdist box_dist, ANNpoint &kdQ, double &kdMaxErr, ANNmin_k *kdPointMK, ANNpointArray &kdPts, int &kdDim)
{
	register ANNdist dist;				// distance to data point
	register ANNcoord* pp;				// data coordinate pointer
	register ANNcoord* qq;				// query coordinate pointer
	register ANNdist min_dist;			// distance to k-th closest point
	register ANNcoord t;
	register int d;

	//AKU min_dist = ANNkdPointMK->max_key(); // k-th smallest distance so far
	min_dist = kdPointMK->max_key(); // k-th smallest distance so far

	for (int i = 0; i < n_pts; i++) {	// check points in bucket

		//AKU pp = ANNkdPts[bkt[i]];			// first coord of next data point
		pp = kdPts[bkt[i]];			// first coord of next data point
		//AKU qq = ANNkdQ;					// first coord of query point
		qq = kdQ;					// first coord of query point
		dist = 0;

		//AKU for(d = 0; d < ANNkdDim; d++) {
		for(d = 0; d < kdDim; d++) {
			ANN_COORD(1)				// one more coordinate hit
			ANN_FLOP(4)					// increment floating ops

			t = *(qq++) - *(pp++);		// compute length and adv coordinate
										// exceeds dist to k-th smallest?
			if( (dist = ANN_SUM(dist, ANN_POW(t))) > min_dist) {
				break;
			}
		}

		//AKU if (d >= ANNkdDim &&					// among the k best?
		if (d >= kdDim &&					    // among the k best?
		   (ANN_ALLOW_SELF_MATCH || dist!=0)) { // and no self-match problem
												// add it to the list
			//AKU ANNkdPointMK->insert(dist, bkt[i]);
			kdPointMK->insert(dist, bkt[i]);
			//AKU min_dist = ANNkdPointMK->max_key();
			min_dist = kdPointMK->max_key();
		}
	}
	ANN_LEAF(1)							// one more leaf node visited
	ANN_PTS(n_pts)						// increment points visited
	ANNptsVisited += n_pts;				// increment number of points visited
}
