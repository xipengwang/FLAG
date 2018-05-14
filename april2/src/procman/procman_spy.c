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

#include <gtk/gtk.h>
#include <assert.h>
#include <inttypes.h>
#include "common/zarray.h"
#include "common/time_util.h"
#include "common/getopt.h"
#include "lcmtypes/procman_process_list_t.h"
#include "lcmtypes/procman_status_list_t.h"
#include "lcmtypes/procman_output_t.h"
#include "lcmtypes/procman_command_t.h"

#define MIXED "---"
#define NOENTER "1q2w3e4r"

//RGB
#define GRAY "#a0a0a0"

#define LI_GREEN "#a0FFa0"
#define LI_RED "#FFa0a0"
#define LI_YELLOW "#FFFFa0"

#define ALERT "#FF0000"

enum
{
    COL_GROUP = 0,
    COL_NAME,
    COL_CMD,
    COL_SUMMARY,
    COL_PROCID,
    COL_HOST,
    COL_AUTO,
    COL_AUTO_DELAY,
    COL_ENABLED,
    COL_RUNNING,
    COL_PID,
    COL_RESTARTS,
    COL_LAST_EXIT,
    RENDER_COLS,
    COL_OUTPUT,
    COL_UPDATE,
    NUM_COLS
} COL_TYPE;

typedef enum
{
    CMD_STOP = 0,
    CMD_START = 1,
    NUM_CMD
} CMD_TYPE;


static const char* col_names[RENDER_COLS] =
{
    "Group",
    "Name",
    "Command",
    "Summary",
    "ProcID",
    "Host",
    "AutoRestart",
    "Delay",
    "Enabled",
    "Running",
    "PID",
    "Restarts",
    "Last Exit",
};


static const int col_widths[RENDER_COLS] =
{
    150, //grp
    75,  //name
    300, //cmd
    85,  //summary
    65,  //procid
    65,  //host
    80,  //auto
    65,  //delay
    80,  //enable
    80,  //running
    50,  //pid
    50,  //restarts
    50,  //last exit
};

//State
typedef struct
{
    int running;
    lcm_t * lcm;
    GtkTreeStore    *model;
    GtkWidget       *tree_view;
    GtkWidget       *auto_btn;
    GtkWidget       *groups_btn;
    GtkWidget       *all_errs;
    GtkWidget       *select_output;
    uint64_t        init_utime;
    uint64_t        utime;
    GtkTextTagTable * tag_table; //For red coloring

}state_t;

typedef struct
{
    state_t * state;
    CMD_TYPE cmd;
}state_cmd_t;

static GtkTreeStore *
create_model (void)
{
    GtkTreeStore  *treestore;

    treestore = gtk_tree_store_new(NUM_COLS,
                                        G_TYPE_STRING, //grp
                                        G_TYPE_STRING, //name
                                        G_TYPE_STRING, //cmd
                                        G_TYPE_STRING, //summary
                                        G_TYPE_STRING, //procid
                                        G_TYPE_STRING, //host
                                        G_TYPE_STRING, //auto
                                        G_TYPE_STRING, //delay
                                        G_TYPE_STRING, //enable
                                        G_TYPE_STRING, //running
                                        G_TYPE_STRING, //pid
                                        G_TYPE_STRING, //restarts
                                        G_TYPE_STRING, //last exit
                                        G_TYPE_ERROR,  //render_col ender
                                        G_TYPE_POINTER, //Text buffers
                                        G_TYPE_BOOLEAN, //Update?
                                        -1);

    return treestore;
}

void
running_cell_renderer (GtkTreeViewColumn *col,
                        GtkCellRenderer   *renderer,
                        GtkTreeModel      *model,
                        GtkTreeIter       *iter,
                        gpointer           user_data)
{
    uintptr_t type = (uintptr_t) user_data;

    gchararray val;
    gtk_tree_model_get(model, iter, type, &val, -1);

    //assume normal render
    g_object_set(renderer, "background-set", FALSE, NULL);

    //only group is allowed to have null values
    if(!val && type != COL_GROUP)
        g_object_set(renderer, "background", "RED", "background-set", TRUE, NULL);
    else if(type == COL_RUNNING)
    {
        gchararray cmd;
        gtk_tree_model_get(model, iter, type, &cmd, -1);
        if(0 == strcmp(val, "Running"))
            g_object_set(renderer, "background", LI_GREEN, "background-set", TRUE, NULL);
        else if(0 == strcmp(val, "Stopped"))
            g_object_set(renderer, "background", LI_RED, "background-set", TRUE, NULL);
        else
            g_object_set(renderer, "background", LI_YELLOW, "background-set", TRUE, NULL);

        free(cmd);
    }
    else if(type == COL_ENABLED)
    {
        if(0 == strcmp(val, "Enabled"))
            g_object_set(renderer, "background", LI_GREEN, "background-set", TRUE, NULL);
        else if(0 == strcmp(val, "Disabled"))
            g_object_set(renderer, "background", LI_RED, "background-set", TRUE, NULL);
        else
            g_object_set(renderer, "background", LI_YELLOW, "background-set", TRUE, NULL);
    }
    else if(type == COL_RESTARTS || type == COL_LAST_EXIT)
    {
        if(0 != strcmp(val, "0"))
            g_object_set(renderer, "background", LI_RED, "background-set", TRUE, NULL);
    }
    else if(type == COL_SUMMARY)
    {
        if(0 == strcmp(val, "Running"))
            g_object_set(renderer, "background", LI_GREEN, "background-set", TRUE, NULL);
        else if(0 == strcmp(val, "Off"))
            g_object_set(renderer, "background", GRAY, "background-set", TRUE, NULL);
        else if(0 == strcmp(val, "DIED"))
            g_object_set(renderer, "background", LI_RED, "background-set", TRUE, NULL);
        else if(0 == strcmp(val, "Stopping"))
            g_object_set(renderer, "background", LI_YELLOW, "background-set", TRUE, NULL);
        else if(0 == strcmp(val, "Unknown"))
            g_object_set(renderer, "background", "Orange", "background-set", TRUE, NULL);
        else if(0 == strcmp(val, "Success"))
            g_object_set(renderer, "background", "white", "background-set", TRUE, NULL);
    }

    g_object_set(renderer, "text", val, NULL);
    free(val);
}

static GtkWidget*
create_tree_view (GtkTreeStore *model, int nosummary)
{
    GtkTreeViewColumn   *col;
    GtkCellRenderer     *renderer;
    GtkWidget           *view;

    view = gtk_tree_view_new();

    for(uintptr_t i = 0; i < RENDER_COLS; i++)
    {
        if(!nosummary && (i == COL_ENABLED || i == COL_RUNNING)) continue;
        if(nosummary && i == COL_SUMMARY) continue;

        col = gtk_tree_view_column_new();
        renderer = gtk_cell_renderer_text_new();
        gtk_tree_view_column_pack_start(col, renderer, FALSE);
        gtk_tree_view_column_add_attribute(col, renderer, "text", i);
        gtk_tree_view_column_set_cell_data_func(col, renderer, running_cell_renderer, (void *)  i, NULL);
        gtk_tree_view_column_set_title(col, col_names[i]);
        gtk_tree_view_column_set_sort_column_id(col, i);
        gtk_tree_view_column_set_resizable(col, TRUE);
        gtk_tree_view_column_set_sizing(col, GTK_TREE_VIEW_COLUMN_FIXED);
        gtk_tree_view_column_set_fixed_width(col, col_widths[i]);
        gtk_tree_view_column_queue_resize(col);

        gtk_tree_view_append_column(GTK_TREE_VIEW(view), col);

    }

    gtk_tree_view_set_model(GTK_TREE_VIEW(view), GTK_TREE_MODEL(model));

    gtk_tree_view_expand_all(GTK_TREE_VIEW(view));

    g_object_unref(model); /* destroy model automatically with view */

    return view;
}

//Checks if entire groups are running well
void
calc_running_success(GtkTreeStore * model)
{
    GtkTreeIter grp_iter;
    int valid = gtk_tree_model_get_iter_first(GTK_TREE_MODEL(model), &grp_iter);
    while(valid)
    {
        int died = 0;
        int stopping = 0;
        int unknown = 0;

        GtkTreeIter chl_iter;
        valid = gtk_tree_model_iter_children(GTK_TREE_MODEL(model), &chl_iter, &grp_iter);
        while(valid)
        {

            gchararray running, enabled;

            gtk_tree_model_get(GTK_TREE_MODEL(model), &chl_iter,
                                                    COL_RUNNING, &running,
                                                    COL_ENABLED, &enabled,
                                                            -1);
            if(running && enabled)
            {
                if(0 == strcmp(running, "Running") &&
                        (0 == strcmp(enabled, "Enabled")))
                    gtk_tree_store_set(model, &chl_iter, COL_SUMMARY,  "Running", -1);
                else if(0 == strcmp(running, "Running") &&
                        (0 == strcmp(enabled, "Disabled")))
                {
                    gtk_tree_store_set(model, &chl_iter, COL_SUMMARY,  "Stopping", -1);
                    stopping++;
                }
                else if(0 == strcmp(running, "Stopped") &&
                        (0 == strcmp(enabled, "Enabled")))
                {
                    gtk_tree_store_set(model, &chl_iter, COL_SUMMARY,  "DIED", -1);
                    died++;
                }
                else if(0 == strcmp(running, "Stopped") &&
                        (0 == strcmp(enabled, "Disabled")))
                    gtk_tree_store_set(model, &chl_iter, COL_SUMMARY,  "Off", -1);
                else
                {
                    gtk_tree_store_set(model, &chl_iter, COL_SUMMARY,  "Unknown", -1);
                    unknown++;
                }


            }
            else
            {
                gtk_tree_store_set(model, &chl_iter, COL_SUMMARY,  "Unknown", -1);
                unknown++;
            }


            if(running != NULL)
                free(running);
            if(enabled != NULL)
                free(enabled);

            valid = gtk_tree_model_iter_next(GTK_TREE_MODEL(model), &chl_iter);
        }
        if(died)
            gtk_tree_store_set(model, &grp_iter, COL_SUMMARY, "DIED" ,-1);
        else if(stopping)
            gtk_tree_store_set(model, &grp_iter, COL_SUMMARY, "Stopping" ,-1);
        else if(unknown)
            gtk_tree_store_set(model, &grp_iter, COL_SUMMARY, "Unknown" ,-1);
        else
            gtk_tree_store_set(model, &grp_iter, COL_SUMMARY, "Success" ,-1);

        valid = gtk_tree_model_iter_next(GTK_TREE_MODEL(model), &grp_iter);
    }
}

void
calc_proc_groups(GtkTreeStore * model)
{
    GtkTreeIter grp_iter;
    int valid = gtk_tree_model_get_iter_first(GTK_TREE_MODEL(model), &grp_iter);
    while(valid)
    {
        gtk_tree_store_set(model, &grp_iter, COL_UPDATE, TRUE, -1);
        valid = gtk_tree_model_iter_next(GTK_TREE_MODEL(model), &grp_iter);
    }
    int new;
    do
    {
        new = 0;
        int valid = gtk_tree_model_get_iter_first(GTK_TREE_MODEL(model), &grp_iter);
        while(valid)
        {
            gboolean update;
            gtk_tree_model_get(GTK_TREE_MODEL(model), &grp_iter, COL_UPDATE, &update, -1);
            if(update) break;
            valid = gtk_tree_model_iter_next(GTK_TREE_MODEL(model), &grp_iter);
        }

        if(valid)
        {
            new = 1;
            gchararray name_g = NOENTER,
                       cmd_g = NOENTER ,
                       procid_g = NOENTER,
                       host_g = NOENTER ,
                       enabled_g = NOENTER,
                       auto_g = NOENTER ,
                       delay_g = NOENTER;

            int init = 1;
            GtkTreeIter chl_iter;
            valid = gtk_tree_model_iter_children(GTK_TREE_MODEL(model), &chl_iter, &grp_iter);
            while(valid)
            {
                gchararray name_c, cmd_c, procid_c, host_c, enabled_c, auto_c, delay_c;

                gtk_tree_model_get(GTK_TREE_MODEL(model), &chl_iter,
                                                                COL_NAME,       &name_c,
                                                                COL_CMD,        &cmd_c,
                                                                COL_PROCID,     &procid_c,
                                                                COL_HOST,       &host_c,
                                                                COL_ENABLED,    &enabled_c,
                                                                COL_AUTO,       &auto_c,
                                                                COL_AUTO_DELAY, &delay_c,
                                                                -1);
                if(init)
                {
                    init = 0;
                    name_g      = strdup(name_c);
                    cmd_g       = strdup(cmd_c);
                    procid_g    = strdup(procid_c);
                    host_g      = strdup(host_c);
                    enabled_g   = strdup(enabled_c);
                    auto_g      = strdup(auto_c);
                    delay_g     = strdup(delay_c);
                }else{
                    if(strcmp(name_g, name_c) != 0)
                    {
                        if(strcmp(name_g, MIXED) != 0) free(name_g);
                        name_g = MIXED;
                    }
                    if(strcmp(cmd_g, cmd_c) != 0)
                    {
                        if(strcmp(cmd_g, MIXED) != 0) free(cmd_g);
                        cmd_g = MIXED;
                    }
                    if(strcmp(procid_g, procid_c) != 0)
                    {
                        if(strcmp(procid_g, MIXED) != 0) free(procid_g);
                        procid_g = MIXED;
                    }
                    if(strcmp(host_g, host_c) != 0)
                    {
                        if(strcmp(host_g, MIXED) != 0) free(host_g);
                        host_g = MIXED;
                    }
                    if(strcmp(enabled_g, enabled_c) != 0)
                    {
                        if(strcmp(enabled_g, MIXED) != 0) free(enabled_g);
                        enabled_g = MIXED;
                    }
                    if(strcmp(auto_g, auto_c) != 0)
                    {
                        if(strcmp(auto_g, MIXED) != 0) free(auto_g);
                        auto_g = MIXED;
                    }
                    if(strcmp(delay_g, delay_c) != 0)
                    {
                        if(strcmp(delay_g, MIXED) != 0) free(delay_g);
                        delay_g = MIXED;
                    }
                }
                free(name_c);
                free(cmd_c);
                free(procid_c);
                free(host_c);
                free(enabled_c);
                free(auto_c);
                free(delay_c);

                valid = gtk_tree_model_iter_next(GTK_TREE_MODEL(model), &chl_iter);
            }
            gtk_tree_store_set(model, &grp_iter,
                                            COL_NAME,   name_g,
                                            COL_CMD,    cmd_g,
                                            COL_PROCID, procid_g,
                                            COL_HOST,   host_g,
                                            COL_ENABLED,enabled_g,
                                            COL_AUTO,   auto_g,
                                            COL_AUTO_DELAY, delay_g,
                                            COL_UPDATE, FALSE,
                                            -1);
            if(strcmp(name_g, MIXED) != 0) free(name_g);
            if(strcmp(cmd_g, MIXED) != 0) free(cmd_g);
            if(strcmp(procid_g, MIXED) != 0) free(procid_g);
            if(strcmp(host_g, MIXED) != 0) free(host_g);
            if(strcmp(enabled_g, MIXED) != 0) free(enabled_g);
            if(strcmp(auto_g, MIXED) != 0) free(auto_g);
            if(strcmp(delay_g, MIXED) != 0) free(delay_g);
        }

    }
    while(new);
}

void
add_no_control(state_t * state)
{
    gdk_threads_enter();
    GtkTreeIter grp_iter;

    gtk_tree_store_clear(state->model);
    gtk_tree_store_insert(state->model, &grp_iter, NULL, -1);
    gtk_tree_store_set(state->model, &grp_iter,
                                                COL_GROUP,  "NO CONTROL",
                                                COL_NAME,   "NC",
                                                COL_CMD,    "NC",
                                                COL_PROCID, "NC",
                                                COL_HOST,   NOENTER,
                                                COL_ENABLED, NOENTER,
                                                COL_AUTO,   NOENTER,
                                                COL_AUTO_DELAY, NOENTER,
                                                -1);
    gdk_threads_leave();
}

void
on_proc_list(const lcm_recv_buf_t *rbuf, const char *channel,
                        const procman_process_list_t *msg, void *user)
{
    state_t * state = user;
    const procman_process_list_t * list = msg;

    bool new = false;

    //New init
    if(list->init_utime > state->init_utime)
    {
        new = true;
        printf("New controller ID current:%" PRIu64 " new: %" PRIu64 "\n", state->init_utime, list->init_utime);
        state->init_utime = list->init_utime;
        gdk_threads_enter();
        gtk_tree_store_clear(state->model);
        gdk_threads_leave();
    }

    //dated Init
    if(list->init_utime < state->init_utime)
    {
        printf("WARN Stale controller ID current:%" PRIu64 " stale:%" PRIu64 "\n", state->init_utime, list->init_utime);
        return;
    }

    if(list->utime > state->utime)
        state->utime = list->utime;

    if(msg->exit)
    {
        printf("procman controller going down...\n");
        add_no_control(state);
        return;
    }

    gdk_threads_enter();

    int groups = gtk_toggle_button_get_active(GTK_TOGGLE_BUTTON(state->groups_btn));

    //Add/refresh Proc data
    for(int i = 0; i < msg->nprocs; i++)
    {
        //Find Group
        GtkTreeIter grp_iter;
        procman_process_t * proc = &list->processes[i];
        int valid = gtk_tree_model_get_iter_first(GTK_TREE_MODEL(state->model), &grp_iter);
        while(valid && groups)
        {
            gchararray grp_name;
            gtk_tree_model_get(GTK_TREE_MODEL(state->model), &grp_iter, COL_GROUP, &grp_name, -1);
            if(strcmp(grp_name, proc->group) == 0)
            {
                free(grp_name);
                break;
            }

            valid = gtk_tree_model_iter_next(GTK_TREE_MODEL(state->model), &grp_iter);
            free(grp_name);
        }

        if(!valid)
        {
            gtk_tree_store_insert(state->model, &grp_iter, NULL, -1);
            gtk_tree_store_set(state->model, &grp_iter,
                                                        COL_NAME,   NOENTER,
                                                        COL_CMD,    NOENTER,
                                                        COL_PROCID, NOENTER,
                                                        COL_HOST,   NOENTER,
                                                        COL_ENABLED, NOENTER,
                                                        COL_AUTO,   NOENTER,
                                                        COL_AUTO_DELAY, NOENTER,
                                                        -1);
            if(groups)
                gtk_tree_store_set(state->model, &grp_iter,COL_GROUP, proc->group,-1);
            else
                gtk_tree_store_set(state->model, &grp_iter,COL_GROUP, "Overview",-1);

        }

        //Find Proc
        GtkTreeIter chl_iter;
        char procid[20];
        snprintf(procid, 20, "%d", proc->procid);
        valid = gtk_tree_model_iter_children(GTK_TREE_MODEL(state->model), &chl_iter, &grp_iter);
        while(valid)
        {
            gchararray compstr;
            gtk_tree_model_get(GTK_TREE_MODEL(state->model), &chl_iter, COL_PROCID, &compstr, -1);

            if(strcmp(compstr, procid) == 0)
            {
                free(compstr);
                break;
            }

            valid = gtk_tree_model_iter_next(GTK_TREE_MODEL(state->model), &chl_iter);
            free(compstr);
        }

        if(!valid)
        {
            gtk_tree_store_insert_after(state->model, &chl_iter, &grp_iter, NULL);
        }

        char delay[20];
        snprintf(delay, 20, "%d", proc->restart_delay_ms);
        gtk_tree_store_set(state->model, &chl_iter,
                                                    COL_GROUP,  proc->group,
                                                    COL_NAME,   proc->name,
                                                    COL_CMD,    proc->cmdline,
                                                    COL_PROCID, procid,
                                                    COL_HOST,   proc->host,
                                                    COL_ENABLED,   (proc->enabled ?  "Enabled" : "Disabled"),
                                                    COL_AUTO,   (proc->auto_restart ?  "Enabled" : "Disabled"),
                                                    COL_AUTO_DELAY, delay,
                                                    -1);


    }
    if(new && groups)
        gtk_tree_view_expand_all(GTK_TREE_VIEW(state->tree_view));

    //Computes group summary
    calc_proc_groups(state->model);
    calc_running_success(state->model);

    gdk_threads_leave();
}

//Calculates the status msg dependent part of group rows
void
calc_stat_groups(GtkTreeStore * model)
{
    GtkTreeIter grp_iter;
    int valid = gtk_tree_model_get_iter_first(GTK_TREE_MODEL(model), &grp_iter);
    while(valid)
    {
        gchararray running_g = MIXED,
                   pid_g    = MIXED,
                   last_exit_g = MIXED;


        int restarts = 0;
        int init = 1;
        GtkTreeIter chl_iter;
        valid = gtk_tree_model_iter_children(GTK_TREE_MODEL(model), &chl_iter, &grp_iter);
        while(valid)
        {

            gchararray running_c, pid_c, last_exit_c, restarts_c;

            gtk_tree_model_get(GTK_TREE_MODEL(model), &chl_iter,
                                                    COL_RUNNING, &running_c,
                                                    COL_PID, &pid_c,
                                                    COL_RESTARTS, &restarts_c,
                                                    COL_LAST_EXIT, &last_exit_c,
                                                            -1);
            if(running_c != NULL)
            {

                if(init)
                {
                    init = 0;
                    running_g   = strdup(running_c);
                    pid_g       = strdup(pid_c);
                    last_exit_g = strdup(last_exit_c);
                }else{
                    if(!running_c || strcmp(running_g, running_c) != 0)
                    {
                        if(strcmp(running_g, MIXED) != 0) free(running_g);
                        running_g = MIXED;
                    }
                    if(!pid_c || strcmp(pid_g, pid_c) != 0)
                    {
                        if(strcmp(pid_g, MIXED) != 0) free(pid_g);
                        pid_g = MIXED;
                    }
                    if(!last_exit_c || strcmp(last_exit_g, last_exit_c) != 0)
                    {
                        if(strcmp(last_exit_g, MIXED) != 0) free(last_exit_g);
                        last_exit_g = MIXED;
                    }
                }
                restarts += strtol(restarts_c, NULL, 10);

                free(running_c);
                free(pid_c);
                free(last_exit_c);
                free(restarts_c);
            }

            valid = gtk_tree_model_iter_next(GTK_TREE_MODEL(model), &chl_iter);
        }
        char restarts_g[20];
        snprintf(restarts_g, 20, "%d", restarts);
        gtk_tree_store_set(model, &grp_iter,
                                    COL_RUNNING,    running_g,
                                    COL_PID,        pid_g,
                                    COL_RESTARTS,   restarts_g,
                                    COL_LAST_EXIT,  last_exit_g,
                                        -1);

        if(strcmp(running_g, MIXED) != 0) free(running_g);
        if(strcmp(pid_g, MIXED) != 0) free(pid_g);
        if(strcmp(last_exit_g, MIXED) != 0) free(last_exit_g);

        valid = gtk_tree_model_iter_next(GTK_TREE_MODEL(model), &grp_iter);
    }
}

GtkTreeIter
find_proc_id(int procid, GtkTreeStore * model)
{
    GtkTreeIter fnd_chl_iter;
    //int found   = 0;
    //Find Group
    GtkTreeIter grp_iter;
    int valid = gtk_tree_model_get_iter_first(GTK_TREE_MODEL(model), &grp_iter);
    while(valid)
    {
        //Find Proc
        GtkTreeIter chl_iter;
        char procid_char[20];
        snprintf(procid_char, 20, "%d", procid);
        valid = gtk_tree_model_iter_children(GTK_TREE_MODEL(model), &chl_iter, &grp_iter);
        while(valid)
        {
            gchararray compstr;
            gtk_tree_model_get(GTK_TREE_MODEL(model), &chl_iter, COL_PROCID, &compstr, -1);

            if(strcmp(compstr, procid_char) == 0)
            {
                fnd_chl_iter = chl_iter;
            }

            valid = gtk_tree_model_iter_next(GTK_TREE_MODEL(model), &chl_iter);

            free(compstr);
        }

        valid = gtk_tree_model_iter_next(GTK_TREE_MODEL(model), &grp_iter);
    }

    return fnd_chl_iter;

}

void
on_stat_list(const lcm_recv_buf_t *rbuf, const char *channel,
                        const procman_status_list_t *msg, void *user)
{
    state_t * state = user;
    const procman_status_list_t * list = msg;

    //New init
    if(list->received_init_utime != state->init_utime)
    {
        printf("WARN status list from wrong controller %" PRId64 "\n", list->received_init_utime);
        return;
    }

    gdk_threads_enter();

    //Add/refresh Proc data
    for(int i = 0; i < msg->nprocs; i++)
    {
        procman_status_t * proc = &list->statuses[i];
        GtkTreeIter fnd_chl_iter = find_proc_id(proc->procid, state->model);

        if(!gtk_tree_store_iter_is_valid(state->model, &fnd_chl_iter))
            continue;

        gchararray hostname;
        gtk_tree_model_get(GTK_TREE_MODEL(state->model), &fnd_chl_iter, COL_HOST, &hostname, -1);
        if((strcmp(hostname, list->host) != 0) &&
                (strcmp(hostname, "localhost") != 0))
        {
            free(hostname);
            continue;
        }
        free(hostname);


        char pid[20];
        snprintf(pid, 20, "%d", proc->pid);

        char restarts[20];
        snprintf(restarts, 20, "%d", proc->restarts);

        char last_exit[20];
        snprintf(last_exit, 20, "%d", proc->last_exit_code);

        gtk_tree_store_set(state->model, &fnd_chl_iter,
                                                    COL_RUNNING,   (proc->running ?  "Running" : "Stopped"),
                                                    COL_PID,        pid,
                                                    COL_RESTARTS,   restarts,
                                                    COL_LAST_EXIT,  last_exit,
                                                    -1);


    }

    calc_stat_groups(state->model);
    calc_running_success(state->model);

    gdk_threads_leave();
}

void
scroll_to_end(GtkWidget* view)
{

    GtkTextBuffer * buf = gtk_text_view_get_buffer(GTK_TEXT_VIEW(view));
    GtkTextIter iter;
    gtk_text_buffer_get_end_iter(buf, &iter);
    gtk_text_view_scroll_to_iter(GTK_TEXT_VIEW(view),
                                &iter,
                                0,
                                TRUE,
                                1,
                                0);

}

void
on_output(const lcm_recv_buf_t *rbuf, const char *channel,
                        const procman_output_t *msg, void *user)
{
    state_t * state = user;

    if(msg->received_init_utime != state->init_utime)
        return;

    gdk_threads_enter();

    GtkTreeIter iter = find_proc_id(msg->procid, state->model);
    if(!gtk_tree_store_iter_is_valid(state->model, &iter))
    {
        gdk_threads_leave();
        return;
    }

    GtkTextBuffer * buf;
    gtk_tree_model_get(GTK_TREE_MODEL(state->model), &iter, COL_OUTPUT, &buf, -1);

    if(buf == NULL)
    {
        buf = gtk_text_buffer_new(state->tag_table);
        gtk_tree_store_set(state->model, &iter, COL_OUTPUT, buf, -1);
    }

    GtkTextIter end;
    gtk_text_buffer_get_end_iter(buf, &end);

    if(msg->stream == 1) //stderr
    {
        gtk_text_buffer_insert_with_tags_by_name(buf, &end, msg->data, strlen(msg->data), "error-color", NULL);

        GtkTextBuffer * allbuf = gtk_text_view_get_buffer(GTK_TEXT_VIEW(state->all_errs));
        gtk_text_buffer_get_end_iter(allbuf, &end);
        gtk_text_buffer_insert_with_tags_by_name(allbuf, &end, msg->data, strlen(msg->data), "error-color", NULL);
    }
    else
    {
        gtk_text_buffer_insert(buf, &end, msg->data, strlen(msg->data));
    }

    gdk_threads_leave();

}

void *
lcm_loop(void * imp)
{
    state_t * state = imp;
    while(state->running)
    {
        lcm_handle(state->lcm);
    }
    return NULL;
}

void *
autoscroll_loop(void * imp)
{
    state_t * state = imp;
    while(state->running)
    {
        gdk_threads_enter();
        if(gtk_toggle_button_get_active(GTK_TOGGLE_BUTTON(state->auto_btn)))
        {
            scroll_to_end(state->select_output);
            scroll_to_end(state->all_errs);
        }

        if(!gtk_toggle_button_get_active(GTK_TOGGLE_BUTTON(state->groups_btn)))
            gtk_tree_view_expand_all(GTK_TREE_VIEW(state->tree_view));

        gdk_threads_leave();
        usleep(1e5);
    }
    return NULL;
}

void
do_for_each (GtkTreeModel *model, GtkTreePath *path, GtkTreeIter *iter,
                                                                gpointer data)
{
    state_t * state = ((state_cmd_t *) data)->state;
    CMD_TYPE  cmd = ((state_cmd_t *) data)->cmd;
    //Look if group
    if(0 == gtk_tree_store_iter_depth(GTK_TREE_STORE(model), iter))
    {
        GtkTreeIter chl_iter;
        int valid = gtk_tree_model_iter_children(GTK_TREE_MODEL(model), &chl_iter, iter);
        while(valid)
        {
            do_for_each (model, path, &chl_iter, data);
            valid = gtk_tree_model_iter_next(GTK_TREE_MODEL(model), &chl_iter);
        }
    }
    else
    {
        gchararray procid_c;
        gtk_tree_model_get(GTK_TREE_MODEL(model), iter,
                                            COL_PROCID,     &procid_c,
                                            -1);

        procman_command_t  msg;
        msg.utime = utime_now(); // blacklist-ignore

        msg.received_utime = state->utime;
        msg.received_init_utime = state->init_utime;
        msg.procid = strtol(procid_c, NULL, 10);
        msg.enabled = cmd;
        procman_command_t_publish(state->lcm, "PROCMAN_COMMAND", &msg);
        free(procid_c);
    }

}

void
start_selected(GtkButton *button, gpointer data)
{
    state_t * state = data;
    state_cmd_t sc;
    sc.state = state;
    sc.cmd = CMD_START;
    GtkTreeSelection * select = gtk_tree_view_get_selection(GTK_TREE_VIEW(state->tree_view));
    gtk_tree_selection_selected_foreach(select, do_for_each, &sc);
}

void
stop_selected(GtkButton *button, gpointer data)
{
    state_t * state = data;
    state_cmd_t sc;
    sc.state = state;
    sc.cmd = CMD_STOP;
    GtkTreeSelection * select = gtk_tree_view_get_selection(GTK_TREE_VIEW(state->tree_view));
    gtk_tree_selection_selected_foreach(select, do_for_each, &sc);
}

void
on_close (GtkWidget *widget, GdkEvent *event, void *user)
{
    state_t * state = user;
    state->running = 0;
}

void
double_clicked (GtkTreeView       *tree_view,
                GtkTreePath       *path,
                GtkTreeViewColumn *column,
                gpointer           user_data)
{
    state_t * state = user_data;

    if(gtk_tree_path_get_depth(path) <= 1)
    {
        if(gtk_tree_view_row_expanded(GTK_TREE_VIEW(state->tree_view), path))
            gtk_tree_view_collapse_row(GTK_TREE_VIEW(state->tree_view), path);
        else
            gtk_tree_view_expand_row(GTK_TREE_VIEW(state->tree_view), path, TRUE);
    }

}

void
selected (GtkTreeSelection *treeselection,
                       gpointer user_data)
{
    state_t * state = user_data;
    GtkTreeModel * model = GTK_TREE_MODEL(state->model);
    GList * list = gtk_tree_selection_get_selected_rows(treeselection,
                                                        &model);

    if(list == NULL) return;
    GtkTreePath *  path = list->data;
    if(path == NULL) return;

    GtkTreeIter iter;
    gtk_tree_model_get_iter(model, &iter, path);

    GtkTextBuffer * buf;
    gtk_tree_model_get(GTK_TREE_MODEL(state->model), &iter, COL_OUTPUT, &buf, -1);
    if(buf == NULL)
    {
        buf = gtk_text_buffer_new(state->tag_table);
        gtk_tree_store_set(state->model, &iter, COL_OUTPUT, buf, -1);
    }
    gtk_text_view_set_buffer(GTK_TEXT_VIEW(state->select_output), buf);

    if(gtk_toggle_button_get_active(GTK_TOGGLE_BUTTON(state->auto_btn)))
        scroll_to_end(state->select_output);

    g_list_free_full (list, (GDestroyNotify) gtk_tree_path_free);
}


void
groups_toggled (GtkToggleButton *togglebutton,
                       gpointer         user_data)
{
    state_t * state = user_data;

    gtk_tree_store_clear(state->model);

}

int
main (int argc, char **argv)
{
    getopt_t * gopt = getopt_create();
    getopt_add_bool(gopt, 'h', "help", 0, "Show Help");
    getopt_add_bool(gopt, 's', "no-summary", 0, "don't summarize running/enabled into one");
    if(!getopt_parse(gopt, argc, argv, 1) ||
            getopt_get_bool(gopt, "help"))
    {
        getopt_do_usage(gopt);
        exit(EXIT_SUCCESS);
    }

    GtkWidget *window;

    gtk_init(&argc, &argv);
    gdk_threads_enter();

    gdk_threads_init ();

    window = gtk_window_new(GTK_WINDOW_TOPLEVEL);
    gtk_window_set_title(GTK_WINDOW(window), "ProcMan Spy");
    g_signal_connect(window, "delete_event", gtk_main_quit, NULL); /* dirty */

    state_t state;
    state.running   = 1;
    state.lcm       = lcm_create(NULL);
    state.model     = create_model();
    state.tree_view = create_tree_view(state.model, getopt_get_bool(gopt, "no-summary"));
    state.init_utime= 0;

    gtk_window_resize(GTK_WINDOW(window), 1000, 600);

    GtkWidget *main_vbox = gtk_vbox_new(FALSE, 0);
    g_signal_connect_swapped(G_OBJECT(window), "destroy", G_CALLBACK(gtk_main_quit), NULL);

    //Scrolling status section
    GtkWidget *scrolled_window = gtk_scrolled_window_new(NULL, NULL);
    gtk_scrolled_window_set_policy(GTK_SCROLLED_WINDOW(scrolled_window),
                                   GTK_POLICY_AUTOMATIC, GTK_POLICY_AUTOMATIC);
    gtk_container_add(GTK_CONTAINER(scrolled_window), state.tree_view);
    g_object_set(state.tree_view,    "rubber-banding", TRUE, NULL);
    gtk_tree_selection_set_mode(gtk_tree_view_get_selection(GTK_TREE_VIEW(state.tree_view)), GTK_SELECTION_MULTIPLE);
    g_signal_connect(state.tree_view, "row-activated", G_CALLBACK(double_clicked), &state);
    g_signal_connect(gtk_tree_view_get_selection(GTK_TREE_VIEW(state.tree_view)), "changed", G_CALLBACK(selected), &state);

    // Add some buttons to the middle area
    GtkWidget *btn_box      = gtk_hbox_new(FALSE, 0);

    GtkWidget *start_btn    = gtk_button_new_with_label("Start Selected");
    g_signal_connect(start_btn, "clicked", G_CALLBACK(start_selected), &state);
    gtk_box_pack_start (GTK_BOX (btn_box), start_btn, TRUE, TRUE, 5);

    GtkWidget *stop_btn     = gtk_button_new_with_label("Stop Selected");
    g_signal_connect(stop_btn, "clicked", G_CALLBACK(stop_selected), &state);
    gtk_box_pack_start (GTK_BOX (btn_box), stop_btn,  TRUE, TRUE, 5);

    state.auto_btn          = gtk_check_button_new_with_label("AutoScroll");
    gtk_box_pack_start (GTK_BOX (btn_box), state.auto_btn, TRUE, TRUE, 5);
    gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(state.auto_btn), TRUE);

    state.groups_btn          = gtk_check_button_new_with_label("Groups");
    gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(state.groups_btn), TRUE);
    g_signal_connect(state.groups_btn, "toggled", G_CALLBACK(groups_toggled), &state);
    gtk_box_pack_start (GTK_BOX (btn_box), state.groups_btn, TRUE, TRUE, 5);

    gtk_box_pack_start (GTK_BOX (main_vbox), btn_box, FALSE, FALSE, 5);


    //left text pane
    GtkWidget *all_out         = gtk_text_view_new();
    state.all_errs = all_out;
    gtk_text_view_set_editable(GTK_TEXT_VIEW(all_out), FALSE);

    GdkColor        red;
    gdk_color_parse(ALERT, &red);
    GtkTextBuffer * buf = gtk_text_view_get_buffer(GTK_TEXT_VIEW(all_out));
    gtk_text_buffer_create_tag( buf,
            "error-color",
            "foreground-gdk", &red, "foreground-set",  TRUE, NULL);
    state.tag_table = gtk_text_buffer_get_tag_table(buf);

    GtkWidget *all_window = gtk_scrolled_window_new(NULL, NULL);
    gtk_scrolled_window_set_policy(GTK_SCROLLED_WINDOW(all_window),
                                   GTK_POLICY_AUTOMATIC, GTK_POLICY_AUTOMATIC);
    gtk_container_add(GTK_CONTAINER(all_window), all_out);


    //right text pane
    GtkWidget *select_out         = gtk_text_view_new();
    state.select_output = select_out;
    gtk_text_view_set_editable(GTK_TEXT_VIEW(select_out), FALSE);
    GtkWidget *select_window = gtk_scrolled_window_new(NULL, NULL);
    gtk_scrolled_window_set_policy(GTK_SCROLLED_WINDOW(select_window),
                                   GTK_POLICY_AUTOMATIC, GTK_POLICY_AUTOMATIC);
    gtk_container_add(GTK_CONTAINER(select_window), select_out);

    //Text combo paned
    GtkWidget *bottom_box      = gtk_hpaned_new();
    gtk_paned_pack1 (GTK_PANED (bottom_box), all_window, TRUE, TRUE);
    gtk_paned_pack2 (GTK_PANED (bottom_box), select_window, TRUE, TRUE);

    gtk_box_pack_start (GTK_BOX (main_vbox), bottom_box, TRUE, TRUE, 0);

    //Full paned
    GtkWidget *full_display      = gtk_vpaned_new();
    gtk_paned_pack1(GTK_PANED (full_display), scrolled_window, TRUE, TRUE);
    gtk_paned_pack2(GTK_PANED (full_display), main_vbox, FALSE, TRUE);
    gtk_paned_set_position(GTK_PANED (full_display), 400);
    gtk_container_add(GTK_CONTAINER(window), full_display);

    gtk_widget_show_all(window);

    procman_process_list_t_subscribe (state.lcm, "PROCMAN_PROCESS_LIST", on_proc_list, &state);
    procman_status_list_t_subscribe (state.lcm, "PROCMAN_STATUS_LIST", on_stat_list, &state);
    procman_output_t_subscribe (state.lcm, "PROCMAN_OUTPUT", on_output, &state);
    pthread_t lcm_pthread;
    pthread_create(&lcm_pthread, NULL, lcm_loop, &state);

    pthread_t scroll_pthread;
    pthread_create(&scroll_pthread, NULL, autoscroll_loop, &state);
    gdk_threads_leave();

    add_no_control(&state);

    gtk_main();

    state.running = 0;

    return 0;
}
