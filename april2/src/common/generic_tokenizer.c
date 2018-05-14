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

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <assert.h>

#include "generic_tokenizer.h"
#include "zarray.h"
#include "zstack.h"
#include "zset.h"
#include "string_util.h"
#include "freepool.h"

#define MIN_CHAR 0
#define EOF_CHAR 127
#define MAX_CHAR EOF_CHAR

#define max(a, b) (((a) > (b)) ? (a) : (b))

struct nstate
{
    char *type;
    int priority;
    int ignore;
    zarray_t *out; // type: nedge
    int id;
};

struct nedge
{
    uint8_t c0, c1;
    int epsilon;
    struct nstate *destination;
};

struct dstate
{
    zset_t *nstates_closure; // type: nstate
    int id;
    zarray_t *out; // type: dedge
    char *type;
    int ignore;
};

struct dedge
{
    char c0, c1;
    struct dstate *destination;
};

/*
struct gt_string_feeder
{
    const char *s;
    int pos;
    int line, col;
};
*/

struct generic_tokenizer
{
    struct nstate *nroot;
    struct dstate *droot;
    zarray_t *dstates;
    int compiled;
    char *error_type;

    int next_nstate_id, next_nstate_priority;
    int next_dstate_id;

    freepool_t *fp;

    // all_nstates is only needed for _debug.
    zarray_t *all_nstates;
};

struct generic_tokenizer_feeder
{
    zarray_t *tokens;
    int pos;
};

static void nstate_destroy(struct nstate *ns)
{
    zarray_destroy(ns->out);
    if (ns->type != NULL)
        free(ns->type);
    memset(ns, 0, sizeof(struct nstate));
    free(ns);
}

static struct nstate *nstate_create(generic_tokenizer_t *gt)
{
    struct nstate *ns = (struct nstate*) calloc(1, sizeof(struct nstate));
    ns->out = zarray_create(sizeof(struct nedge*));
    ns->id = gt->next_nstate_id++;
    zarray_add(gt->all_nstates, &ns);
    freepool_add(gt->fp, ns, nstate_destroy);
    return ns;
}

static struct nedge *nedge_create(generic_tokenizer_t *gt)
{
    struct nedge *ne = (struct nedge*) calloc(1, sizeof(struct nedge));
    freepool_add(gt->fp, ne, free);
    return ne;
}

static struct dstate *dstate_create(generic_tokenizer_t *gt)
{
    struct dstate *ds = (struct dstate*) calloc(1, sizeof(struct dstate));
    ds->out = zarray_create(sizeof(struct dedge*));
    ds->id = gt->next_dstate_id++;

    freepool_add(gt->fp, ds->out, zarray_destroy);
    freepool_add(gt->fp, ds, free);
    return ds;
}

static struct dedge *dedge_create(generic_tokenizer_t *gt)
{
    struct dedge *de = (struct dedge*) calloc(1, sizeof(struct dedge));
    freepool_add(gt->fp, de, free);
    return de;
}

static void set_accept_flags_for_control_char(int accepts[], char c)
{
    if (c=='n')
        accepts[(int) '\n'] = 1;
    else if (c=='t')
        accepts[(int) '\t'] = 1;
    else if (c=='r')
        accepts[(int) '\r'] = 1;
    else if (c=='s') {
        accepts[(int) ' '] = 1;
        accepts[(int) '\n'] = 1;
        accepts[(int) '\r'] = 1;
        accepts[(int) '\t'] = 1;
    } else if (c=='$') {
        accepts[EOF_CHAR] = 1;
    } else {
        accepts[(int) c] = 1;
    }
}

static void create_nstates(generic_tokenizer_t *gt, string_feeder_t *sf, struct nstate *sinit, struct nstate *sexit)
{
    // sinit: where should we go back to if we encounter a |. It's
    // always the beginning of an expression or the beginning of the
    // inner most parethetical block.

    // where should we go back to if we encounter a *, +, ? It's
    // "back" one atom.
    struct nstate *sback = sinit;

    // the state representing our current working position.
    struct nstate *stail = sinit;

    if (1) {
        // Provide a dummy link to insulate this sub-expression from
        // the previous sub-expression.  Consider the regex:
        // "A+B|AC". Without this insulation, both OR branches will
        // have a common root, and the epsilon transition resulting
        // from the "+" will go back to this common root. As a
        // consequence, the regular expression would erroneously match
        // "AAAAAC".  We provide a similar extra link when handling
        // "|" below.

        struct nstate *s = nstate_create(gt);
        struct nedge *e = nedge_create(gt);
        e->epsilon = 1;
        e->destination = s;
        zarray_add(sinit->out, &e);

        sback = s;
        stail = s;
    }

    while (string_feeder_has_next(sf)) {

        char c = string_feeder_next(sf);

        switch (c) {
            case ')': {
                struct nedge *e = nedge_create(gt);
                e->epsilon = 1;
                e->destination = sexit;
                zarray_add(stail->out, &e);
                return;
            }

            case '|': {
                // Terminate this part of the OR clause by connecting to the exit state.
                struct nedge *e = nedge_create(gt);
                e->epsilon = 1;
                e->destination = sexit;
                zarray_add(stail->out, &e);

                // Create a new OR clause with an insulating dummy state.
                struct nstate *s = nstate_create(gt);
                struct nedge *e2 = nedge_create(gt);
                e2->epsilon = 1;
                e2->destination = s;
                zarray_add(sinit->out, &e2);

                sback = s;
                stail = s;
                break;
            }

            case '(': {
                struct nstate *snew = nstate_create(gt);
                create_nstates(gt, sf, stail, snew);
                sback = stail;
                stail = snew;
                break;
            }

            case '.': {
                struct nstate *snew = nstate_create(gt);
                struct nedge *e = nedge_create(gt);
                e->c0 = MIN_CHAR;
                e->c1 = EOF_CHAR - 1;
                e->destination = snew;
                zarray_add(stail->out, &e);

                sback = stail;
                stail = snew;
                break;
            }

                // NB: control characters can map to multiple accepted
                // characters (e.g. \\s), so handle these two cases
                // the same.
            case '\\':
            case '[': {

                struct nstate *snew = nstate_create(gt);

                int accepts[MAX_CHAR+1];
                memset(accepts, 0, sizeof(accepts));

                if (c == '\\') {
                    c = string_feeder_next(sf);

                    set_accept_flags_for_control_char(accepts, c);

                } else {
                    // it's a [
                    int invert = 0;

                    if (string_feeder_peek(sf)=='^') {
                        invert = 1;
                        (void) string_feeder_next(sf);
                    }

                    while (string_feeder_peek(sf)!=']') {
                        int c0 = string_feeder_next(sf);
                        if (c0 == '\\') {
                            c0 = string_feeder_next(sf);
                            set_accept_flags_for_control_char(accepts, c0);
                            continue;
                        }

                        int c1 = c0;
                        if (string_feeder_has_next(sf) && string_feeder_peek(sf)=='-') {
                            string_feeder_next(sf);
                            c1 = string_feeder_next(sf);
                        }

                        for (int i = c0; i <= c1; i++)
                            accepts[i] = 1;
                    }
                    string_feeder_next(sf); // consume ']'

                    if (invert)
                        for (int i = 0; i <= MAX_CHAR; i++)
                            accepts[i] ^= 1;
                }

                struct nedge *lastEdge = NULL;

                for (int i = MIN_CHAR; i <= MAX_CHAR; i++) {
                    if (accepts[i]) {
                        if (lastEdge != NULL && i > 0 && accepts[i-1]) {
                            lastEdge->c1++;
                            continue;
                        }

                        lastEdge = nedge_create(gt);
                        lastEdge->c0 = i;
                        lastEdge->c1 = i;
                        lastEdge->destination = snew;
                        zarray_add(stail->out, &lastEdge);
                    }
                }

                sback = stail;
                stail = snew;
                break;
            }

            case '*': {
                // epsilon edge backwards
                struct nedge *e = nedge_create(gt);
                e->epsilon = 1;
                e->destination = sback;
                zarray_add(stail->out, &e);

                // add edge so we can also skip over last atom...
                e = nedge_create(gt);
                e->epsilon = 1;
                e->destination = stail;
                zarray_add(sback->out, &e);
                break;
            }

            case '+': {
                // epsilon edge backwards
                struct nedge *e = nedge_create(gt);
                e->epsilon = 1;
                e->destination = sback;
                zarray_add(stail->out, &e);
                break;
            }

            case '?': {
                // add edge so we can also skip over last atom...
                struct nedge *e = nedge_create(gt);
                e = nedge_create(gt);
                e->epsilon = 1;
                e->destination = stail;
                zarray_add(sback->out, &e);
                break;
            }

            default: {
                // it's an ordinary character.
                struct nstate *snew = nstate_create(gt);
                struct nedge *e = nedge_create(gt);
                e->c0 = c;
                e->c1 = c;
                e->destination = snew;
                zarray_add(stail->out, &e);

                sback = stail;
                stail = snew;
                break;
            }
        }
    }

    // If out of regex input, create a transition to the exit state.
    struct nedge *e = nedge_create(gt);
    e->epsilon = 1;
    e->destination = sexit;
    zarray_add(stail->out, &e);
}

generic_tokenizer_t *generic_tokenizer_create(const char *error_type)
{
    generic_tokenizer_t *gt = (generic_tokenizer_t*) calloc(1, sizeof(generic_tokenizer_t));
    gt->fp = freepool_create();

    gt->all_nstates = zarray_create(sizeof(struct nstate*));

    gt->dstates = zarray_create(sizeof(struct dstate*));
    gt->error_type = strdup(error_type);
    gt->nroot = nstate_create(gt);

    freepool_add(gt->fp, gt->all_nstates, zarray_destroy);
    freepool_add(gt->fp, gt->dstates, zarray_destroy);
    freepool_add(gt->fp, gt->error_type, free);
    return gt;
}

static void generic_tokenizer_add_impl(generic_tokenizer_t *gt, const char *type, const char *regex, int ignore)
{
    assert(!gt->compiled);

    if (ignore)
        assert(type == NULL);
    if (!ignore)
        assert(type != NULL);

    struct nstate *terminalNstate = nstate_create(gt);
    if (type != NULL) {
        terminalNstate->type = strdup(type);
        terminalNstate->priority = gt->next_nstate_priority++;
    }

    terminalNstate->ignore = ignore;

    struct nstate *initialNstate = nstate_create(gt);
    struct nedge *e = nedge_create(gt);
    e->epsilon = 1;
    e->destination = initialNstate;
    zarray_add(gt->nroot->out, &e);

    string_feeder_t *sf = string_feeder_create(regex);
    create_nstates(gt, sf, initialNstate, terminalNstate);
    string_feeder_destroy(sf);
}

void generic_tokenizer_add_escape(generic_tokenizer_t *gt, const char *type, const char *regex)
{
    string_buffer_t *sb = string_buffer_create();

    for (int i = 0; i < strlen(regex); i++) {
        char c = regex[i];

        switch (c)
        {
            case '(':
            case ')':
            case '^':
            case '.':
            case '$':
            case '|':
            case '\\':
            case '+':
            case '-':
            case '?':
            case '*':
            case '[':
            case ']':
                string_buffer_append(sb, '\\');
                string_buffer_append(sb, c);
                break;
            case ' ':
                string_buffer_append(sb, '|');
                break;
            default:
                string_buffer_append(sb, c);
                break;
        }
    }

    char *s = string_buffer_to_string(sb);
    generic_tokenizer_add_impl(gt, type, s, 0);
    free(s);
    string_buffer_destroy(sb);
}

void generic_tokenizer_add(generic_tokenizer_t *gt, const char *type, const char *regex)
{
    generic_tokenizer_add_impl(gt, type, regex, 0);
}

void generic_tokenizer_add_ignore(generic_tokenizer_t *gt, const char *regex)
{
    generic_tokenizer_add_impl(gt, NULL, regex, 1);
}

// takes a set of nstates as input, returns the set of reachable
// nstates via epsilon transitions.
static void epsilon_closure_impl(generic_tokenizer_t *gt, zset_t *closure, zstack_t *queue)
{
    while (!zstack_empty(queue)) {
        struct nstate *s = NULL;
        zstack_pop(queue, &s);

        if (zset_contains(closure, &s))
            continue;

        zset_add(closure, &s, NULL);
        for (int eidx = 0; eidx < zarray_size(s->out); eidx++) {
            struct nedge *e = NULL;
            zarray_get(s->out, eidx, &e);
            if (e->epsilon)
                zstack_push(queue, &e->destination);
        }
    }
}

static zset_t *epsilon_closure_state(generic_tokenizer_t *gt, struct nstate *s)
{
    zset_t *closure = zset_create(sizeof(struct nstate*), zhash_ptr_hash, zhash_ptr_equals);
    zstack_t *queue = zstack_create(sizeof(struct nstate*));

    zstack_push(queue, &s);
    epsilon_closure_impl(gt, closure, queue);
    zstack_destroy(queue);

    freepool_add(gt->fp, closure, zset_destroy);
    return closure;
}

static zset_t *epsilon_closure_set(generic_tokenizer_t *gt, zset_t *instates)
{
    zset_t *closure = zset_create(sizeof(struct nstate*), zhash_ptr_hash, zhash_ptr_equals);
    zstack_t *queue = zstack_create(sizeof(struct nstate*));

    zset_iterator_t vit;
    zset_iterator_init(instates, &vit);
    struct nstate *ns;
    while (zset_iterator_next(&vit, &ns)) {
        zstack_push(queue, &ns);
    }

    epsilon_closure_impl(gt, closure, queue);
    zstack_destroy(queue);

    freepool_add(gt->fp, closure, zset_destroy);
    return closure;
}

static struct dstate *get_dstate(generic_tokenizer_t *gt, zset_t *closure)
{
    for (int i = 0; i < zarray_size(gt->dstates); i++) {
        struct dstate *s = NULL;
        zarray_get(gt->dstates, i, &s);
        struct nstate *scratch;
        if (zset_equals(s->nstates_closure, closure, &scratch))
            return s;
    }

    struct dstate *ds = dstate_create(gt);
    ds->nstates_closure = closure;

    int priority = INT32_MAX;
    zset_iterator_t it;
    zset_iterator_init(ds->nstates_closure, &it);

    struct nstate *ns = NULL;
    while (zset_iterator_next(&it, &ns)) {
        if ((ns->type != NULL || ns->ignore) && ns->priority < priority) {
            ds->type = ns->type;
            ds->ignore = ns->ignore;
            priority = ns->priority;
        }
    }

    zarray_add(gt->dstates, &ds);
    return ds;
}

static void compile_impl(generic_tokenizer_t *gt, zstack_t *queue, zset_t *visited)
{
    while (!zstack_empty(queue)) {
        struct dstate *ds = NULL;
        zstack_pop(queue, &ds);

        if (zset_contains(visited, &ds))
            continue;

        zset_add(visited, &ds, NULL);

        zarray_t *edges = zarray_create(sizeof(struct nedge*));
        zset_iterator_t it;
        zset_iterator_init(ds->nstates_closure, &it);

        struct nstate *ns;
        while (zset_iterator_next(&it, &ns)) {
            for (int i = 0; i < zarray_size(ns->out); i++) {
                struct nedge *ne = NULL;
                zarray_get(ns->out, i, &ne);
                zarray_add(edges, &ne);
            }
        }

        zset_t *last_destinations_closure = NULL;
        struct dedge *last_dedge = NULL;

        for (int i = MIN_CHAR; i <= MAX_CHAR; i++) {

            zset_t *destinations = zset_create(sizeof(struct nstate*), zhash_ptr_hash, zhash_ptr_equals);
            freepool_add(gt->fp, destinations, zset_destroy);

            for (int eidx = 0; eidx < zarray_size(edges); eidx++) {
                struct nedge *ne = NULL;
                zarray_get(edges, eidx, &ne);
                if (!ne->epsilon && i >= ne->c0 && i <= ne->c1)
                    zset_add(destinations, &ne->destination, NULL);
            }

            zset_t *destinations_closure = epsilon_closure_set(gt, destinations);
            struct nstate *scratch;

            if (last_destinations_closure != NULL && zset_equals(destinations_closure, last_destinations_closure, &scratch)) {
                assert(last_dedge->c1 == (i-1));
                assert(last_dedge->destination == get_dstate(gt, destinations_closure));
                last_dedge->c1 = i;
                continue;
            }

            if (zset_size(destinations_closure) > 0) {
                last_dedge = dedge_create(gt);
                last_dedge->c0 = i;
                last_dedge->c1 = i;
                last_dedge->destination = get_dstate(gt, destinations_closure);
                zstack_push(queue, &last_dedge->destination);
                zarray_add(ds->out, &last_dedge);
                last_destinations_closure = destinations_closure;
            } else {
                last_destinations_closure = NULL; // XXX leak?
            }
        }

        zarray_destroy(edges);
    }
}

static void compile(generic_tokenizer_t *gt)
{
    assert(!gt->compiled);
    gt->compiled = 1;

    zstack_t *queue = zstack_create(sizeof(struct dstate*));
    zset_t *rootClosure = epsilon_closure_state(gt, gt->nroot);
    gt->droot = get_dstate(gt, rootClosure);
    zstack_push(queue, &gt->droot);

    zset_t *visited = zset_create(sizeof(struct dstate*), zhash_ptr_hash, zhash_ptr_equals);

    compile_impl(gt, queue, visited);

    zstack_destroy(queue);
    zset_destroy(visited);
}

void generic_tokenizer_debug(generic_tokenizer_t *gt)
{
    if (!gt->compiled)
        compile(gt);

    for (int idx = 0; idx < zarray_size(gt->all_nstates); idx++) {
        struct nstate *s = NULL;
        zarray_get(gt->all_nstates, idx, &s);

        printf("nstate %3d: type %s\n", idx, s->type);
        for (int eidx = 0; eidx < zarray_size(s->out); eidx++) {
            struct nedge *e = NULL;
            zarray_get(s->out, eidx, &e);

            printf("  [%3d - %3d ], epsilon=%d, ==> %d\n", e->c0, e->c1, e->epsilon, e->destination->id);
//            printf("  [%d '%c', %d '%c'], epsilon=%d, ==> %d\n", e->c0, e->c0, e->c1, e->c1, e->epsilon, e->destination->id);
        }
    }

    for (int idx = 0; idx < zarray_size(gt->dstates); idx++) {
        struct dstate *s = NULL;
        zarray_get(gt->dstates, idx, &s);

        printf("dstate %3d: type %s\n", idx, s->type);
        for (int eidx = 0; eidx < zarray_size(s->out); eidx++) {
            struct dedge *e = NULL;
            zarray_get(s->out, eidx, &e);

            printf("  [%3d - %3d], ==> %d\n", e->c0, e->c1, e->destination->id);
//            printf("  [%d '%c', %d '%c'], ==> %d\n", e->c0, e->c0, e->c1, e->c1, e->destination->id);
        }
    }

}

zarray_t *generic_tokenizer_tokenize_with_path(generic_tokenizer_t *gt, const char *path, const char *_s)
{
    if (!gt->compiled)
        compile(gt);

    char *s;
    if (1) {
        int len = strlen(_s);
        s = malloc(len+2);
        memcpy(s, _s, len);
        s[len] = EOF_CHAR;
        s[len+1] = 0;
    }

    zarray_t *tokens = zarray_create(sizeof(struct gt_token*));

    string_feeder_t *sf = string_feeder_create(s);

    struct dstate *state = gt->droot;
    string_buffer_t *sb = string_buffer_create();

    int c = string_feeder_next(sf);

    struct gt_token *tok = NULL;

    while (1) {
        int consumed = 0;

        if (tok == NULL)
            tok = calloc(1, sizeof(struct gt_token));

        if (string_buffer_size(sb) == 0) {
            tok->line = string_feeder_get_line(sf);
            tok->column = string_feeder_get_column(sf);
        }

        for (int eidx = 0; eidx < zarray_size(state->out); eidx++) {
            struct dedge *e = NULL;
            zarray_get(state->out, eidx, &e);

            if (c >= e->c0 && c <= e->c1) {
                consumed = 1;
                state = e->destination;
                break;
            }
        }

        if (consumed) {
            string_buffer_append(sb, c);

            if (!string_feeder_has_next(sf)) {
                if (state->type != NULL || state->ignore) {
                    if (!state->ignore) {
                        tok->path = strdup(path);
                        tok->type = strdup(state->type);
                        tok->token = string_buffer_to_string(sb);
                        zarray_add(tokens, &tok);
                        tok = NULL;
                    }
                } else {
                    // Arriving here means that we ran out of input
                    // but do not have a terminal token.
                    tok->path = strdup(path);
                    tok->type = strdup(gt->error_type);
                    tok->token = string_buffer_to_string(sb);
                    zarray_add(tokens, &tok);
                    tok = NULL;
//                    printf("generic_tokenizer: incomplete token. (%s)\n", string_buffer_to_string(sb));
                }

                goto done;
            }

            c = string_feeder_next(sf);
            continue;
        }

        // NOT CONSUMED. This forces a token production.

        // If the token is zero length, it's an error condition. Force
        // us to consume one character so we keep returning a
        // zero-length token.
        if (string_buffer_size(sb) == 0) {
            string_buffer_append(sb, c);
            tok->path = strdup(path);
            tok->type = strdup(gt->error_type);
            tok->token = string_buffer_to_string(sb);
            zarray_add(tokens, &tok);
            tok = NULL;

            string_buffer_reset(sb);
            state = gt->droot;

            // consume this character.
            c = string_feeder_next(sf);
            continue;
        }

        // Is this a terminal state? If so, produce a token.
        if (state->type != NULL && !state->ignore) {
            tok->path = strdup(path);
            tok->type = strdup(state->type);
            tok->token = string_buffer_to_string(sb);

            zarray_add(tokens, &tok);
            tok = NULL;
            string_buffer_reset(sb);
            state = gt->droot;

            // don't consume another character; restart with the
            // current character.
            continue;
        }

        if (state->ignore) {
            // don't consume another character.
            string_buffer_reset(sb);
            state = gt->droot;
            continue;
        }

        // The state we're in is NOT a terminal state. It's an error.
        // If we're not in the terminal state, just produce what we
        // have and start over.
        if (1) {
            tok->path = strdup(path);
            tok->type = strdup(gt->error_type);
            tok->token = string_buffer_to_string(sb);
            zarray_add(tokens, &tok);
            tok = NULL;

            string_buffer_reset(sb);
            state = gt->droot;

            if (!string_feeder_has_next(sf))
                goto done;

            // don't consume this one.
            continue;
        }

        printf("HOW HERE?\n");

        string_buffer_reset(sb);
    }

  done:
    if (tok != NULL)
        free(tok);

    free(s);
    string_feeder_destroy(sf);
    string_buffer_destroy(sb);
    return tokens;
}

zarray_t *generic_tokenizer_tokenize(generic_tokenizer_t *gt, const char *_s)
{
    return generic_tokenizer_tokenize_with_path(gt, "<no path>", _s);
}

zarray_t *generic_tokenizer_tokenize_path(generic_tokenizer_t *gt, const char *path)
{
    FILE *f = fopen(path, "rb");
    if (f == NULL)
        return NULL;

    fseek(f, 0, SEEK_END);
    long length = ftell(f);
    fseek(f, 0, SEEK_SET);

    char *s = malloc(length+1);
    size_t readlen = fread(s, 1, length, f);
    assert(readlen == length);
    s[length] = 0;

    zarray_t *tokens = generic_tokenizer_tokenize_with_path(gt, path, s);
    free(s);
    fclose(f);
    return tokens;
}

void generic_tokenizer_destroy(generic_tokenizer_t *gt)
{
    freepool_destroy(gt->fp);
    memset(gt, 0, sizeof(generic_tokenizer_t));
    free(gt);
}

void generic_tokenizer_tokens_destroy(zarray_t *tokens)
{
    for (int i = 0; i < zarray_size(tokens); i++) {
        struct gt_token *tok = NULL;
        zarray_get(tokens, i, &tok);
        free(tok->path);
        free(tok->token);
        free(tok->type);
        free(tok);
    }
    zarray_destroy(tokens);
}

generic_tokenizer_feeder_t *generic_tokenizer_feeder_create(zarray_t *tokens)
{
    generic_tokenizer_feeder_t *feeder = calloc(1, sizeof(generic_tokenizer_feeder_t));
    feeder->tokens = tokens;
    feeder->pos = 0;

    return feeder;
}

void generic_tokenizer_feeder_destroy(generic_tokenizer_feeder_t *feeder)
{
    memset(feeder, 0, sizeof(generic_tokenizer_feeder_t));
    free(feeder);
}

int generic_tokenizer_feeder_has_next(generic_tokenizer_feeder_t *feeder)
{
    return feeder->pos < zarray_size(feeder->tokens);
}

const char *generic_tokenizer_feeder_next(generic_tokenizer_feeder_t *feeder)
{
    if (!generic_tokenizer_feeder_has_next(feeder))
        return NULL;

    struct gt_token *tok = NULL;
    zarray_get(feeder->tokens, feeder->pos, &tok);
    feeder->pos++;
    return tok->token;
}

const struct gt_token *generic_tokenizer_feeder_next_token(generic_tokenizer_feeder_t *feeder)
{
    if (!generic_tokenizer_feeder_has_next(feeder))
        return NULL;

    struct gt_token *tok = NULL;
    zarray_get(feeder->tokens, feeder->pos, &tok);
    feeder->pos++;
    return tok;
}

const char *generic_tokenizer_feeder_last(generic_tokenizer_feeder_t *feeder)
{
    struct gt_token *tok = NULL;
    zarray_get(feeder->tokens, max(0, feeder->pos - 1), &tok);
    return tok->token;
}

const struct gt_token *generic_tokenizer_feeder_last_token(generic_tokenizer_feeder_t *feeder)
{
    struct gt_token *tok = NULL;
    zarray_get(feeder->tokens, max(0, feeder->pos - 1), &tok);
    return tok;
}

const char *generic_tokenizer_feeder_peek(generic_tokenizer_feeder_t *feeder)
{
    if (!generic_tokenizer_feeder_has_next(feeder))
        return NULL;

    struct gt_token *tok = NULL;
    zarray_get(feeder->tokens, feeder->pos, &tok);
    return tok->token;
}

const char *generic_tokenizer_feeder_peek_type(generic_tokenizer_feeder_t *feeder)
{
    if (!generic_tokenizer_feeder_has_next(feeder))
        return NULL;

    struct gt_token *tok = NULL;
    zarray_get(feeder->tokens, feeder->pos, &tok);
    return tok->type;
}

const struct gt_token *generic_tokenizer_feeder_peek_token(generic_tokenizer_feeder_t *feeder)
{
    if (!generic_tokenizer_feeder_has_next(feeder))
        return NULL;

    struct gt_token *tok = NULL;
    zarray_get(feeder->tokens, feeder->pos, &tok);
    return tok;
}

int generic_tokenizer_feeder_consume(generic_tokenizer_feeder_t *feeder, const char *s)
{
    if (!generic_tokenizer_feeder_has_next(feeder))
        return 0;

    if (!strcmp(generic_tokenizer_feeder_peek(feeder), s)) {
        generic_tokenizer_feeder_next(feeder);
        return 1;
    }

    return 0;
}

int generic_tokenizer_feeder_consume_type(generic_tokenizer_feeder_t *feeder, const char *s)
{
    if (!generic_tokenizer_feeder_has_next(feeder))
        return 0;

    if (!strcmp(generic_tokenizer_feeder_peek_type(feeder), s)) {
        generic_tokenizer_feeder_next(feeder);
        return 1;
    }

    return 0;
}

void generic_tokenizer_feeder_require(generic_tokenizer_feeder_t *feeder, const char *s)
{
    if (!generic_tokenizer_feeder_consume(feeder, s)) {
        const struct gt_token *tok = generic_tokenizer_feeder_peek_token(feeder);
        //fatal("generic_tokenizer_feeder_require(): At line %d, column %d, expected %s. Got %s instead\n", tok->line, tok->column, s, tok->token);
        fprintf(stderr, "generic_tokenizer_feeder_require(): At line %d, column %d, expected %s. Got %s instead\n", tok->line, tok->column, s, tok->token);
    }
}
