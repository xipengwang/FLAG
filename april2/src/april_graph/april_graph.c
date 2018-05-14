/* Copyright (C) 2013-2016, The Regents of The University of Michigan.
All rights reserved.

This software was developed in the APRIL Robotics Lab under the
direction of Edwin Olson, ebolson@umich.edu. This software may be
available under alternative licensing terms; contact the address above.

This library is free software; you can redistribute it and/or
modify it under the terms of the GNU Lesser General Public
License as published by the Free Software Foundation; either
version 2.1 of the License, or (at your option) any later version.

This library is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public
License along with this library; if not, see <http://www.gnu.org/licenses/>.
*/

#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <float.h>

#include "april_graph.h"
#include "common/math_util.h"
#include "common/string_util.h"
#include "common/timeprofile.h"
#include "common/smatd.h"
#include "common/doubles.h"

int *exact_minimum_degree_ordering(smatd_t *mat);

double alt_mod2pi(double v)
{
    while (v > M_PI)
        v -= 2*M_PI;
    while (v < -M_PI)
        v += 2*M_PI;
    return v;
}

struct tok_feeder
{
    zarray_t *toks;
    int pos;
};

static int tf_int(struct tok_feeder *tf)
{
    char *tok;
    zarray_get(tf->toks, tf->pos++, &tok);
    return atoi(tok);
}

static double tf_double(struct tok_feeder *tf)
{
    char *tok;
    zarray_get(tf->toks, tf->pos++, &tok);
    return atof(tok);
}

static char* tf_string(struct tok_feeder *tf)
{
    char *tok;
    zarray_get(tf->toks, tf->pos++, &tok);
    return tok;
}

april_graph_t *april_graph_create_from_file_old(const char *path)
{
    april_graph_t *graph = april_graph_create();

    FILE *f = fopen(path, "r");
    char line[1024];

    while (fgets(line, sizeof(line), f) != NULL) {

        zarray_t *toks = str_split(line, " ");
        struct tok_feeder tf = { .toks = toks, .pos = 0 };

        char *type = tf_string(&tf);

        if (!strcmp(type, "xytnode") && zarray_size(toks)==10) {

            double *state = (double[]) { tf_double(&tf), tf_double(&tf), tf_double(&tf) };
            double  *init = (double[]) { tf_double(&tf), tf_double(&tf), tf_double(&tf) };
            double *truth = (double[]) { tf_double(&tf), tf_double(&tf), tf_double(&tf) };

            april_graph_node_t *n = april_graph_node_xyt_create(state, init, truth);

            zarray_add(graph->nodes, &n);

        } else if (!strcmp(type, "xytedge") && zarray_size(toks)==15) {

            int a = tf_int(&tf), b = tf_int(&tf);
            double *z      = (double[]) { tf_double(&tf), tf_double(&tf), tf_double(&tf) };
            double *ztruth = (double[]) { tf_double(&tf), tf_double(&tf), tf_double(&tf) };

            matd_t *P = matd_create_data(3, 3,
                                         (double[]) { tf_double(&tf), tf_double(&tf), tf_double(&tf),
                                                 0,              tf_double(&tf), tf_double(&tf),
                                                 0,              0,              tf_double(&tf) });
            for (int i = 0; i < 3; i++)
                for (int j = i+1; j < 3; j++)
                    MATD_EL(P, j, i) = MATD_EL(P, i, j);

            matd_t *W = matd_op("M^-1", P);

            april_graph_factor_t *f = april_graph_factor_xyt_create(a, b,
                                                                    z, ztruth,
                                                                    W);

            matd_destroy(P);
            matd_destroy(W);

            zarray_add(graph->factors, &f);

        } else {
            printf("Unknown type '%s' with %d tokens\n", type, zarray_size(toks));
        }

        zarray_vmap(toks, free);
        zarray_destroy(toks);
    }

    fclose(f);

    return graph;
}

void april_graph_destroy(april_graph_t *graph)
{
    for (int i = 0; i < zarray_size(graph->nodes); i++) {
        april_graph_node_t *n;
        zarray_get(graph->nodes, i, &n);

        n->destroy(n);
    }

    for (int i = 0; i < zarray_size(graph->factors); i++) {
        april_graph_factor_t *factor;
        zarray_get(graph->factors, i, &factor);

        factor->destroy(factor);
    }

    zarray_destroy(graph->nodes);
    zarray_destroy(graph->factors);

    free(graph);
}

int april_graph_dof(april_graph_t *graph)
{
    int factor_dof = 0, state_dof = 0;

    for (int i = 0; i < zarray_size(graph->nodes); i++) {
        april_graph_node_t *n;
        zarray_get(graph->nodes, i, &n);

        state_dof += n->length;
    }

    for (int i = 0; i < zarray_size(graph->factors); i++) {
        april_graph_factor_t *factor;
        zarray_get(graph->factors, i, &factor);

        factor_dof += factor->length;
    }

    return factor_dof - state_dof;
}

double april_graph_chi2(april_graph_t *graph)
{
    double chi2 = 0;

    for (int i = 0; i < zarray_size(graph->factors); i++) {
        april_graph_factor_t *factor;
        zarray_get(graph->factors, i, &factor);
        april_graph_factor_eval_t *eval = factor->eval(factor, graph, NULL);

        chi2 += eval->chi2;

        april_graph_factor_eval_destroy(eval);
    }

    return chi2;
}

void april_graph_postscript(april_graph_t *graph, const char *path)
{
    FILE *f = fopen(path, "w");
    fprintf(f, "%%!PS\n\n");

    fprintf(f, "0 setlinewidth\n");

    double x0 = FLT_MAX, x1 = -FLT_MAX, y0 = FLT_MAX, y1 = -FLT_MAX;

    double page_width = 612, page_height = 792;

    for (int i = 0; i < zarray_size(graph->nodes); i++) {
        april_graph_node_t *n;
        zarray_get(graph->nodes, i, &n);

        if (n->type == APRIL_GRAPH_NODE_XYT_TYPE) {
            x0 = fmin(x0, n->state[0]);
            x1 = fmax(x1, n->state[0]);
            y0 = fmin(y0, n->state[1]);
            y1 = fmax(y1, n->state[1]);
        }
    }

    double s = fmin(page_width / (x1-x0), page_height / (y1-y0));

    fprintf(f, "/node {\n"); // invocation: x y theta node
    fprintf(f, "gsave\n");
    fprintf(f, "3 1 roll\n"); // convert tx ty theta => theta tx ty
    fprintf(f, "translate\n");
    fprintf(f, "57.296 mul rotate\n");
    fprintf(f, ".2 .2 scale\n"); // size of robot
    fprintf(f, ".5 setgray\n");
    fprintf(f, "newpath\n-1 -1 moveto\n2 0 lineto\n-1 1 lineto\nclosepath\nfill\n");
    fprintf(f, "grestore\n");
    fprintf(f, "} def\n");
    fprintf(f, "\n");

    fprintf(f, "/edge {\n"); // invocation: x0 y0 x1 y1 dposes edge
    fprintf(f, "gsave\n");
//    fprintf(f, "pop\n");
    fprintf(f, "1 eq { 0 0 1 setrgbcolor } { 0 1 1 setrgbcolor } ifelse\n");
    fprintf(f, "moveto lineto stroke\n");
    fprintf(f, "grestore\n");
    fprintf(f, "} def\n");
    fprintf(f, "\n");

    fprintf(f, "%f %f translate\n", page_width / 2, page_height / 2);
    fprintf(f, "%f %f scale\n", s, s);
    fprintf(f, "%f %f translate\n", -(x0+x1)/2, -(y0+y1)/2);

    for (int i = 0; i < zarray_size(graph->factors); i++) {
        april_graph_factor_t *factor;
        zarray_get(graph->factors, i, &factor);

        if (factor->type != APRIL_GRAPH_FACTOR_XYT_TYPE)
            continue;

        april_graph_node_t *na, *nb;
        zarray_get(graph->nodes, factor->nodes[0], &na);
        zarray_get(graph->nodes, factor->nodes[1], &nb);

        fprintf(f, "%f %f %f %f %d edge\n",
                na->state[0], na->state[1], nb->state[0], nb->state[1],
                abs(factor->nodes[1] - factor->nodes[0]));
    }

    for (int i = 0; i < zarray_size(graph->nodes); i++) {
        april_graph_node_t *n;
        zarray_get(graph->nodes, i, &n);

        if (n->type != APRIL_GRAPH_NODE_XYT_TYPE)
            continue;

        fprintf(f, "%f %f %f node\n", n->state[0], n->state[1], n->state[2]);
    }

    fclose(f);
}

////////////////////////////////////////////////////////////////////////////////////////

void april_graph_factor_eval_destroy(april_graph_factor_eval_t *eval)
{
    if (!eval)
        return;

    for (int i = 0; i < eval->length; i++) {
        if (!eval->jacobians[i])
            break;

        matd_destroy(eval->jacobians[i]);
    }

    free(eval->jacobians);
    free(eval->r);
    matd_destroy(eval->W);
    free(eval);
}

/////////////////////////////////////////////////////////////////////////////////////////
// Cholesky Solver
void april_graph_cholesky_param_init(april_graph_cholesky_param_t *param)
{
    memset(param, 0, sizeof(april_graph_cholesky_param_t));

    param->ordering = NULL;
    param->tikhanov = 0.0001;
    param->show_timing = 0;
}

// Compute a Gauss-Newton update on the graph, using the specified
// node ordering. NULL can be passed in for parameters.
void april_graph_cholesky(april_graph_t *graph, april_graph_cholesky_param_t *_param)
{
    april_graph_cholesky_param_t param;
    april_graph_cholesky_param_init(&param);

    // nothing to do
    if (zarray_size(graph->nodes) == 0 || zarray_size(graph->factors) == 0)
        return;

    if (_param) {
        memcpy(&param, _param, sizeof(april_graph_cholesky_param_t));
    }

    timeprofile_t *tp = timeprofile_create();
    timeprofile_stamp(tp, "begin");

    int *allocated_ordering = NULL; // we'll free this one.
    int *use_ordering = param.ordering; // this could be from .param or one we create.

    if (use_ordering == NULL) {
        // make symbolic matrix for variable reordering.
        smatd_t *Asym = smatd_create(zarray_size(graph->nodes), zarray_size(graph->nodes));
        for (int fidx = 0; fidx < zarray_size(graph->factors); fidx++) {
            april_graph_factor_t *factor;
            zarray_get(graph->factors, fidx, &factor);

            for (int i = 0; i < factor->nnodes; i++) {
                for (int j = 0; j < factor->nnodes; j++) {
                    smatd_set(Asym, factor->nodes[i], factor->nodes[j], 1);
                }
            }
        }

        timeprofile_stamp(tp, "make symbolic");

        allocated_ordering = exact_minimum_degree_ordering(Asym);
        use_ordering = allocated_ordering;
        smatd_destroy(Asym);
    }

    // idxs[j]: what index in x do the state variables for node j start at?
    int *idxs = calloc(zarray_size(graph->nodes), sizeof(int));
    int xlen = 0;
    for (int i = 0; i < zarray_size(graph->nodes); i++) {
        april_graph_node_t *node;
        zarray_get(graph->nodes, use_ordering[i], &node);
        idxs[use_ordering[i]] = xlen;
        xlen += node->length;
    }

    assert(xlen > 0);

    if (allocated_ordering)
        free(allocated_ordering);

    timeprofile_stamp(tp, "compute ordering");

    // we'll solve normal equations, Ax = B
    smatd_t *A = smatd_create(xlen, xlen);
    double  *B = calloc(xlen, sizeof(double));

    for (int i = 0; i < zarray_size(graph->factors); i++) {
        april_graph_factor_t *factor;
        zarray_get(graph->factors, i, &factor);
        april_graph_factor_eval_t *eval = factor->eval(factor, graph, NULL);

        for (int z0 = 0; z0 < factor->nnodes; z0++) {
            int n0 = factor->nodes[z0];

            matd_t *JatW = matd_op("M'*M", eval->jacobians[z0], eval->W);

            for (int z1 = 0; z1 < factor->nnodes; z1++) {
                int n1 = factor->nodes[z1];

                matd_t *JatWJb = matd_op("M*M", JatW, eval->jacobians[z1]);

                for (int row = 0; row < JatWJb->nrows; row++) {
                    for (int col = 0; col < JatWJb->ncols; col++) {
                        double v = smatd_get(A, row+idxs[n0], col+idxs[n1]);
                        smatd_set(A, row+idxs[n0], col+idxs[n1], v + MATD_EL(JatWJb, row, col));
                    }
                }

                matd_destroy(JatWJb);
            }

            matd_t *R = matd_create_data(eval->length, 1, eval->r);
            matd_t *JatWr = matd_op("M*M", JatW, R);
            for (int row = 0; row < JatWr->nrows; row++)
                B[idxs[n0]+row] += MATD_EL(JatWr, row, 0);

            matd_destroy(R);
            matd_destroy(JatW);
            matd_destroy(JatWr);
        }

        april_graph_factor_eval_destroy(eval);
    }

    // tikhanov regularization
    // Ensure a maximum condition number of no more than maxcond.
    // trace(A) = sum of eigenvalues. worst-case scenario is that
    // we're rank 1 and that one eigenvalue is trace(A). Thus,
    // ensure all other eigenvalues are at least trace(A)/maxcond.
    if (param.tikhanov > 0) {
/*
        double trace = 0;
        for (int i = 0; i < xlen; i++) {
            double v = smatd_get(A, i, i);
            trace += v;
        }

        if (trace == 0) {
            printf("warning: trace is zero!");
//            trace = 1;
        }
*/
        // XXX
        double lambda = param.tikhanov; //  trace / param.max_cond;

        for (int i = 0; i < xlen; i++) {
            double v = smatd_get(A, i, i);
            smatd_set(A, i, i, v + lambda);
        }
    }

    timeprofile_stamp(tp, "build A, B");

    smatd_chol_t *chol = smatd_chol(A);
    double *x = calloc(xlen, sizeof(double));
    smatd_chol_solve(chol, B, x);

    for (int i = 0; i < zarray_size(graph->nodes); i++) {
        april_graph_node_t *node;
        zarray_get(graph->nodes, i, &node);

        node->update(node, &x[idxs[i]]);
    }

    timeprofile_stamp(tp, "solve");

    smatd_chol_destroy(chol);
    smatd_destroy(A);

    free(B);
    free(x);
    free(idxs);

    if (param.show_timing)
        timeprofile_display(tp);
    timeprofile_destroy(tp);
}

april_graph_gauss_seidel_info_t *april_graph_gauss_seidel_info_create(april_graph_t *graph)
{
    april_graph_gauss_seidel_info_t *info = calloc(1, sizeof(april_graph_gauss_seidel_info_t));

    int nnodes = zarray_size(graph->nodes);
    int nfactors = zarray_size(graph->factors);

    /////////////////////////////////////////////////////
    // make a list of the factors that affect each node.
    zarray_t *allneighbors = zarray_create(sizeof(zarray_t*));
    for (int nodeidx = 0; nodeidx < nnodes; nodeidx++) {
        zarray_t *neighbors = zarray_create(sizeof(int));
        zarray_add(allneighbors, &neighbors);
    }

    for (int factoridx = 0; factoridx < nfactors; factoridx++) {
        april_graph_factor_t *factor;
        zarray_get(graph->factors, factoridx, &factor);

        for (int z0 = 0; z0 < factor->nnodes; z0++) {
            int n0 = factor->nodes[z0];
            zarray_t *neighbors;
            zarray_get(allneighbors, n0, &neighbors);
            zarray_add(neighbors, &factoridx);
        }
    }

    info->allneighbors = allneighbors;
    info->nevals = nfactors;
    info->evals = calloc(nfactors, sizeof(april_graph_factor_eval_t*));

    return info;
}

void april_graph_gauss_seidel_info_destroy(april_graph_gauss_seidel_info_t *info)
{
    if (!info)
        return;

    for (int i = 0; i < zarray_size(info->allneighbors); i++) {
        zarray_t *neighbors;
        zarray_get(info->allneighbors, i, &neighbors);
        zarray_destroy(neighbors);
    }
    zarray_destroy(info->allneighbors);

    for (int i = 0; i < info->nevals; i++)
        april_graph_factor_eval_destroy(info->evals[i]);
    free(info->evals);
    free(info);
}

void april_graph_gauss_seidel(april_graph_t *graph, april_graph_gauss_seidel_info_t *info)
{
    int nnodes = zarray_size(graph->nodes);
    int nfactors = zarray_size(graph->factors);

    zarray_t *allneighbors = info->allneighbors;

    /////////////////////////////////////////////////////
    // TODO: What is a better order for evaluating nodes?
    for (int nodeidx = 0; nodeidx < nnodes; nodeidx++) {
        april_graph_node_t *node;
        zarray_get(graph->nodes, nodeidx, &node);

        matd_t *JtWJ = matd_create(node->length, node->length);
        matd_t *JtWr = matd_create(node->length, 1);

        zarray_t *neighbors;
        zarray_get(allneighbors, nodeidx, &neighbors);

        // it won't be solvable if there are no constraints. This is a
        // sufficient but not necessary condition to eliminate
        // under-constrained solves.
        if (zarray_size(neighbors) == 0)
            continue;

        int nn = min(100, zarray_size(neighbors));

        for (int neighboridx = 0; neighboridx < nn; neighboridx++) {
            int factoridx;
            zarray_get(neighbors, neighboridx, &factoridx);
            april_graph_factor_t *factor;
            zarray_get(graph->factors, factoridx, &factor);

            for (int z0 = 0; z0 < factor->nnodes; z0++) {
                int n0 = factor->nodes[z0];
                if (n0 == nodeidx) {

                    // try to recycle "eval" objects as much as we can, noting that
                    // we can only recycle eval objects on factors of the same type.
                    april_graph_factor_eval_t *eval = info->evals[factoridx];
                    eval = factor->eval(factor, graph, eval);
                    info->evals[factoridx] = eval;

                    if (0) {
                        matd_t *thisJtW = matd_op("M'*M", eval->jacobians[z0], eval->W);
                        matd_t *thisJtWJ = matd_op("M*M", thisJtW, eval->jacobians[z0]);
                        matd_t *thisR = matd_create_data(eval->length, 1, eval->r);
                        matd_t *thisJtWr = matd_op("M*M", thisJtW, thisR);

                        matd_add_inplace(JtWJ, thisJtWJ);
                        matd_add_inplace(JtWr, thisJtWr);

                        matd_destroy(thisJtW);
                        matd_destroy(thisJtWJ);
                        matd_destroy(thisR);
                        matd_destroy(thisJtWr);
                    } else {
                        // M: observation dimension
                        // N: state dimension
                        //
                        // J: M x N,  J': N x M
                        // W: M x M
                        // R: M x 1
                        // J'W: N x M
                        // J'WR: N x 1
                        // J'WJ: N x N
                        int N = node->length;
                        int M = factor->length;

                        double thisJtW[N*M], thisJtWJ[N*N], thisJtWr[N];

                        doubles_mat_AtB(eval->jacobians[z0]->data, M, N, eval->W->data, M, M,
                                        thisJtW, N, M);
                        doubles_mat_AB(thisJtW, N, M, eval->jacobians[z0]->data, M, N,
                                       thisJtWJ, N, N);
                        doubles_mat_Ab(thisJtW, N, M, eval->r, M, thisJtWr, N);

                        doubles_mat_add(JtWJ->data, N, N, thisJtWJ, N, N,
                                        JtWJ->data, N, N);
                        doubles_mat_add(JtWr->data, N, 1, thisJtWr, N, 1,
                                        JtWr->data, N, 1);
                    }
                }
            }
        }

        matd_t *dx = matd_solve(JtWJ, JtWr);

        // over-relaxation
        if (0) {
            for (int i = 0; i < dx->nrows; i++)
                MATD_EL(dx, i, 0) *= 1.1;
        }

        node->update(node, dx->data);
        matd_destroy(dx);

        matd_destroy(JtWJ);
        matd_destroy(JtWr);
    }
}

void april_graph_gradient_descent(april_graph_t *graph)
{
    // map index of graph->nodes to index in state vector
    int state_index[zarray_size(graph->nodes) + 1];

    state_index[0] = 0;
    for (int i = 0; i < zarray_size(graph->nodes); i++) {
        april_graph_node_t *node;
        zarray_get(graph->nodes, i, &node);
        state_index[i+1] = state_index[i] + node->length;
    }

    // how many states?
    int N = state_index[zarray_size(graph->nodes)];

    //  d = J'*P'*W*P*r;
    double d[N];
    memset(d, 0, sizeof(double) * N);

    for (int fidx = 0; fidx < zarray_size(graph->factors); fidx++) {

        april_graph_factor_t *factor;
        zarray_get(graph->factors, fidx, &factor);
        april_graph_factor_eval_t *eval = factor->eval(factor, graph, NULL);

        matd_t *r = matd_create_data(eval->length, 1, eval->r);

        for (int z0 = 0; z0 < factor->nnodes; z0++) {

            matd_t *JtWr = matd_op("M'*M*M", eval->jacobians[z0], eval->W, r);

            for (int i = 0; i < JtWr->nrows; i++)
                d[state_index[factor->nodes[z0]] + i] += MATD_EL(JtWr, i, 0);

            matd_destroy(JtWr);
        }

        matd_destroy(r);
    }

//     lambda = d'*J'*P'*W*P*r / (d'*J'*P'*W*P*J*d);

    // lambda numerator
    double lambda_N = 0;
    for (int i = 0; i < N; i++)
        lambda_N = d[i]*d[i];

    // lambda denominator
    double lambda_D = 0;
    for (int fidx = 0; fidx < zarray_size(graph->factors); fidx++) {
        april_graph_factor_t *factor;
        zarray_get(graph->factors, fidx, &factor);
        april_graph_factor_eval_t *eval = factor->eval(factor, graph, NULL);

        for (int z0 = 0; z0 < factor->nnodes; z0++) {
            int off0 = state_index[factor->nodes[z0]];
            int off1 = state_index[factor->nodes[z0]+1];
            int sz = off1 - off0;
            matd_t *di = matd_create_data(sz, 1, &d[off0]);
            matd_t *Jdi = matd_op("M*M", eval->jacobians[z0], di);
            matd_t *tmp = matd_op("M'*M*M", Jdi, eval->W, Jdi);
            assert(tmp->nrows == 1 && tmp->ncols == 1);
            lambda_D = MATD_EL(tmp, 0, 0);
            matd_destroy(tmp);
            matd_destroy(Jdi);
            matd_destroy(di);
        }
        april_graph_factor_eval_destroy(eval);
    }

    double lambda = lambda_N / lambda_D;
    //printf("%e %e: %e\n", lambda_N, lambda_D, lambda);

    double maxT = 0;
    double maxR = 0;
    for (int i = 0; i < zarray_size(graph->nodes); i++) {
        double *dx = &d[state_index[i]];
        maxT = fmax(maxT, dx[0]*dx[0] + dx[1]*dx[1]);
        maxR = fmax(maxR, fabs(dx[2]));
    }
    maxT = sqrt(maxT);

//    lambda = fmax(lambda_N / lambda_D, fmin(1.0 / maxT, 10.0 * (M_PI / 180.0) / maxR));
    lambda = fmax(lambda_N / lambda_D, 1.0 / maxT);
    for (int i = 0; i < N; i++) {
        d[i] *= lambda;
        //printf("%4d %15f\n", i, d[i]);
    }

    //printf("%f %f %f\n", maxT, maxR, lambda);

    // perform step
    for (int i = 0; i < zarray_size(graph->nodes); i++) {
        april_graph_node_t *node;
        zarray_get(graph->nodes, i, &node);
        node->update(node, &d[state_index[i]]);
    }
}

// take hash pointer pointer so we can initialize if NULL
static void attr_add(april_graph_attr_t **attr, const stype_t *stype, const char *key, void *value)
{
    // XXX race
    if (*attr == NULL)
        *attr = april_graph_attr_create();

    zhash_t *hash = (*attr)->hash;

    struct april_graph_attr_record record = { .stype = stype,
                                              .value = value };
    char *keycopy = strdup(key);

    char *oldkey;
    struct april_graph_attr_record oldrecord;

    if (zhash_put(hash, &keycopy, &record, &oldkey, &oldrecord)) {
        // free the old value
        if (oldrecord.value == NULL) {
            // nothing to do when adding a value when only a NULL
            // exited before.
        } else if (oldrecord.value != value) {
            // they've changed the value. This application is
            // responsible for correctly removing the old one.
        }

        // free the old key
        free(oldkey);
    }
}

static void *attr_get(april_graph_attr_t *attr, const char *key)
{
    if (attr == NULL)
        return NULL;

    struct april_graph_attr_record record;
    if (zhash_get(attr->hash, &key, &record))
        return record.value;

    return NULL;
}

void april_graph_attr_put(april_graph_t *graph, const stype_t *type, const char *key, void *data)
{
    attr_add(&graph->attr, type, key, data);
}

void* april_graph_attr_get(april_graph_t * graph,
                           const char * key)
{
    return attr_get(graph->attr, key);
}

void april_graph_factor_attr_put(april_graph_factor_t *factor, const stype_t *type,
                                 const char *key, void *data)
{
    attr_add(&factor->attr, type, key, data);
}

void* april_graph_factor_attr_get(april_graph_factor_t * factor,
                                  const char * key)
{
    return attr_get(factor->attr, key);
}

void april_graph_node_attr_put(april_graph_node_t *node, const stype_t *type,
                               const char *key, void *data)
{
    attr_add(&node->attr, type, key, data);
}

void* april_graph_node_attr_get(april_graph_node_t *node,
                                const char *key)
{
    return attr_get(node->attr, key);
}

static void april_graph_attr_encode(const stype_t *stype, uint8_t *data, uint32_t *datapos, const void *obj)
{
    const april_graph_attr_t *attr = obj;

    zhash_iterator_t zit;
    zhash_iterator_init(attr->hash, &zit);
    char *name;
    struct april_graph_attr_record record;
    while (zhash_iterator_next(&zit, &name, &record)) {
        if (record.stype) {
            encode_u8(data, datapos, 1); // pompeil flag
            encode_string_u32(data, datapos, name);
            stype_encode_object(data, datapos, record.stype, record.value);
        } else {
//            printf("graph attribute %s doesn't have a type\n", name);
        }
    }

    encode_u8(data, datapos, 0);
}

static void *april_graph_attr_decode(const stype_t *stype, const uint8_t *data, uint32_t *datapos, uint32_t datalen)
{
    april_graph_attr_t *attr = april_graph_attr_create();

    while (decode_u8(data, datapos, datalen)) {
        struct april_graph_attr_record record;
        char *key = decode_string_u32(data, datapos, datalen);
        record.value = stype_decode_object(data, datapos, datalen, &record.stype);

        zhash_put(attr->hash, &key, &record, NULL, NULL);
    }

    return attr;
}

const stype_t stype_april_graph_attr = { .name = "april_graph_attr_t",
                                         .encode = april_graph_attr_encode,
                                         .decode = april_graph_attr_decode,
                                         .copy = NULL };

april_graph_attr_t *april_graph_attr_create()
{
    april_graph_attr_t *attr = calloc(1, sizeof(april_graph_attr_t));
    attr->stype = &stype_april_graph_attr;
    attr->hash = zhash_create(sizeof(char*), sizeof(struct april_graph_attr_record),
                              zhash_str_hash, zhash_str_equals);
    return attr;
}

void april_graph_attr_destroy(april_graph_attr_t *attr)
{
    if (!attr)
        return;

    zhash_destroy(attr->hash);
    free(attr);
}

void april_graph_encode(const stype_t *stype, uint8_t *data, uint32_t *datapos, const void *_obj)
{
    const april_graph_t *graph = _obj;

    for (int i = 0; i < zarray_size(graph->nodes); i++) {
        april_graph_node_t *node;
        zarray_get(graph->nodes, i, &node);

        if (node->stype) {
            encode_u8(data, datapos, 1); // encoding a node
            stype_encode_object(data, datapos, node->stype, node);
        } else {
            printf("node without stype\n");
        }
    }

    for (int i = 0; i < zarray_size(graph->factors); i++) {
        april_graph_factor_t *factor;
        zarray_get(graph->factors, i, &factor);

        if (factor->stype) {
            encode_u8(data, datapos, 2); // encoding a factor
            stype_encode_object(data, datapos, factor->stype, factor);
        } else {
            printf("factor without stype\n");
        }
    }

    encode_u8(data, datapos, 0); // EOF

    april_graph_attr_t *attr = graph->attr;
    stype_encode_object(data, datapos, attr ? attr->stype : NULL, attr);
}

void *april_graph_decode(const stype_t *stype, const uint8_t *data, uint32_t *datapos, uint32_t datalen)
{
    april_graph_t *graph = april_graph_create();

    while (1) {
        uint8_t op = decode_u8(data, datapos, datalen);
        if (op == 0)
            break;

        switch (op) {

            case 1: { // node
                april_graph_node_t *node = stype_decode_object(data, datapos, datalen, NULL);
                if (node != NULL) {
                    zarray_add(graph->nodes, &node);
                }
                break;
            }

            case 2: { // factor
                april_graph_factor_t *factor = stype_decode_object(data, datapos, datalen, NULL);
                if (factor != NULL) {
                    zarray_add(graph->factors, &factor);
                }
                break;
            }

            default: {
                printf("bad opcode %d\n", op);
                assert(0);
                break;
            }
        }
    }

    graph->attr = stype_decode_object(data, datapos, datalen, NULL);
    return graph;
}

const stype_t stype_april_graph = { .name = "april_graph_t",
                                    .encode = april_graph_encode,
                                    .decode = april_graph_decode,
                                    .copy = NULL };


april_graph_t *april_graph_create()
{
    april_graph_t *graph = calloc(1, sizeof(april_graph_t));

    graph->factors = zarray_create(sizeof(april_graph_factor_t*));
    graph->nodes = zarray_create(sizeof(april_graph_node_t*));
    graph->stype = &stype_april_graph;

    return graph;
}

void april_graph_xy_stype_init();
void april_graph_xyz_stype_init();
void april_graph_xyt_stype_init();
void april_graph_max_stype_init();
void april_graph_xypos_stype_init();
void april_graph_xytpos_stype_init();
void april_graph_xyzr_stype_init();
void april_graph_xyzt_stype_init();
void april_graph_r_stype_init();
void april_graph_r_fixed_stype_init();
void april_graph_xyzpos_stype_init();
void april_graph_xyr_stype_init();

void april_graph_stype_init()
{
    stype_register(&stype_april_graph);
    stype_register(&stype_april_graph_attr);
    april_graph_xy_stype_init();
    april_graph_xyz_stype_init();
    april_graph_xyt_stype_init();
    april_graph_max_stype_init();
    april_graph_xypos_stype_init();
    april_graph_xytpos_stype_init();
    april_graph_xyzr_stype_init();
    april_graph_xyzt_stype_init();
    april_graph_r_fixed_stype_init();
    april_graph_r_stype_init();
    april_graph_xyzpos_stype_init();
    april_graph_xyr_stype_init();
}

int april_graph_save(april_graph_t *graph, const char *path)
{
    uint32_t datalen = 0;
    stype_encode_object(NULL, &datalen, graph->stype, graph);

    uint8_t *tmp = malloc(datalen);
    uint32_t tmplen = 0;
    stype_encode_object(tmp, &tmplen, graph->stype, graph);

    FILE *f = fopen(path, "w");
    if(f == NULL)
    {
        printf("failed to open %s\n", path);
        return(0);
    }
    size_t res = fwrite(tmp, 1, tmplen, f);
    free(tmp);
    fclose(f);
    if (res == tmplen)
        return 1;
    return 0;
}

april_graph_t *april_graph_create_from_file(const char *path)
{
    april_graph_t *graph = NULL;

    FILE *f = fopen(path, "r");
    if (f == NULL)
        return NULL;

    fseek(f, 0L, SEEK_END);
    long len = ftell(f);
    fseek(f, 0L, SEEK_SET);

    uint8_t *tmp = malloc(len);

    size_t sz = fread(tmp, 1, len, f);
    if (sz != len)
        goto cleanup;

    uint32_t pos = 0;
    graph = stype_decode_object(tmp, &pos, len, NULL);

  cleanup:

    free(tmp);
    fclose(f);
    return graph;
}
