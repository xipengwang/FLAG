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
#include <unistd.h>
#include <stdlib.h>
#include <pthread.h>
#include <lcm/lcm.h>

#include "common/getopt.h"
#include "common/config.h"
#include "common/zhash.h"
#include "common/zset.h"
#include "common/time_util.h"
#include "common/string_util.h"

#include "lcmtypes/alarm_t.h"
#include "lcmtypes/led_message_t.h"
#include "lcmtypes/speak_t.h"

struct channel_info
{
  char *channel;
  int  rxcount;
  double min, max;
};

struct topic_info
{
  char *topic;
  int nalarms;
  double credit;
  int64_t last_credit_utime;
  int64_t last_speak_utime;
};

typedef struct state state_t;
struct state {
  getopt_t *gopt;
  config_t *config;
  lcm_t *lcm;

  zarray_t *channel_infos; // struct channel_info*
  zhash_t *topic_infos;

  int64_t uninhibit_utime;
  char * speak_args;
  double repeat_inhibit_time;
};

static void rate_handler(const lcm_recv_buf_t *rbuf,
			 const char *channel,
			 void *user)
{
  struct channel_info *cinfo = user;
  cinfo->rxcount++;
}

// rate limiting is organized by topic.
void do_alarm(state_t *state, const char *topic, const char *msg)
{
  struct topic_info *topic_info;
  if (!zhash_get(state->topic_infos, &topic, &topic_info)) {
    topic_info = calloc(1, sizeof(struct topic_info));
    char *t = strdup(topic);
    zhash_put(state->topic_infos, &t, &topic_info, NULL, NULL);
    topic_info->credit = config_get_double(state->config,
                                           "alarm.maxAlarms",
                                           5);
  }

  int64_t now = utime_now();

  if (now < state->uninhibit_utime) {
    printf("%15.3f muting due to startup: %s:%s\n", utime_now() / 1.0E6, topic, msg);
    return;
  }

  if ((now - topic_info->last_speak_utime) / 1.0E6 < 1.0 / config_get_double(state->config, "alarm.topic_max_rate", 0.1)) {
    printf("%15.3f muting topic due to rate: %s:%s\n", utime_now() / 1.0E6, topic, msg);
    return;
  }

  if (topic_info->credit < 1) {
    if (topic_info->last_credit_utime != 0) {
      double dt = (now - topic_info->last_credit_utime) / 1.0E6;
      topic_info->credit += dt / state->repeat_inhibit_time;
    }
    topic_info->last_credit_utime = now;
  }

  if (topic_info->credit < 1) {
    printf("%15.3f muting topic due to insufficient credit: %s:%s\n", utime_now() / 1.0E6, topic, msg);
    return;
  }

  topic_info->credit--;

  speak_t speak;
  memset(&speak, 0, sizeof(speak));
  speak.utime = utime_now();
  speak.args = state->speak_args;
  speak.priority = 2;
  speak.message = (char*) msg;

  printf("%15.3f speak: %s\n", utime_now() / 1.0E6, speak.message);

  speak_t_publish(state->lcm, "SPEAK", &speak);

  led_message_t led;
  led.utime = speak.utime;
  led.msg = speak.message;
  led.red = led.green = led.blue = 225;
  led.rotate_keepalive = 1;

  led_message_t_publish(state->lcm, "LED_MESSAGE", &led);

  topic_info->last_speak_utime = now;

}

void do_alarmf(state_t *state, const char *topic, const char *fmt, int rate)
{

  char *buf = NULL;
  buf = sprintf_alloc(fmt, topic, rate);

  do_alarm(state, topic, buf);

  free(buf);
}

void on_alarm(const lcm_recv_buf_t *rbuf,
              const char *channel, const alarm_t *msg, void *userdata)
{
  state_t *state = userdata;

  if (strlen(msg->message) > 0)
    do_alarm(state, msg->topic, msg->message);
}

int main(int argc, char **argv)
{
  setlinebuf(stdout);
  setlinebuf(stderr);

  state_t *state = calloc(1, sizeof(state_t));
  state->gopt = getopt_create();
  state->channel_infos = zarray_create(sizeof(struct channel_info*));
  state->topic_infos = zhash_create(sizeof(char*), sizeof(struct topic_info*),
				    zhash_str_hash, zhash_str_equals);

  getopt_add_bool(state->gopt, 'h', "help", 0, "Show usage");
  getopt_add_string(state->gopt, 'c', "config", "/path/to/robot.config", "config file");

  if (!getopt_parse(state->gopt, argc, argv, 1) || getopt_get_bool(state->gopt, "help")) {
    printf("Usage: %s [options]\n", argv[0]);
    getopt_do_usage(state->gopt);
    return 1;
  }

  state->config = config_create_path(getopt_get_string(state->gopt, "config"));
  if (!state->config) {
    printf("unable to load config file: %s\n", getopt_get_string(state->gopt, "config"));
    return 1;
  }

  state->lcm = lcm_create(NULL);

  const zarray_t *keys = config_get_keys(state->config);
  const char *match_str = "alarm.lcm_rates.";

  for (int i = 0; i < zarray_size(keys); i++) {
    char *key = NULL;
    zarray_get(keys, i, &key);
    if (!strncmp(match_str, key, strlen(match_str))) {

      char *channel = strdup(key+strlen(match_str));

      double minmax[2];
      config_require_doubles_len(state->config, key, minmax, 2);
      printf("%15.3f monitoring channel %-20s %6.1f <= rate <= %6.1f\n",
	     utime_now() / 1.0E6, channel, minmax[0], minmax[1]);

      struct channel_info *cinfo = calloc(1, sizeof(struct channel_info));
      cinfo->channel = channel;
      cinfo->min = minmax[0];
      cinfo->max = minmax[1];

      zarray_add(state->channel_infos, &cinfo);

      lcm_subscribe(state->lcm, channel, rate_handler, cinfo);
    }
  }

  double delay = config_get_double(state->config, "alarm.startup_delay", 0);
  state->uninhibit_utime = utime_now() + delay*1.0E6;
  state->repeat_inhibit_time = config_get_double(state->config,
                                                 "alarm.repeatInhibitTime",
                                                 8);
  state->speak_args = strdup(config_get_string(state->config, "alarm.espeak_args", ""));


  uint64_t last_utime = utime_now();

  alarm_t_subscribe(state->lcm, "ALARM.*", on_alarm, state);

  while (1) {
    lcm_handle_timeout(state->lcm, 100);

    uint64_t now = utime_now();
    double dt = (now - last_utime) / 1.0E6;
    if (dt > config_get_double(state->config, "alarm.lcm_rates_period", 2.0)) {
      for (int i = 0; i < zarray_size(state->channel_infos); i++) {
	struct channel_info *cinfo;
	zarray_get(state->channel_infos, i, &cinfo);

	double rate = cinfo->rxcount / dt;
	if (rate < cinfo->min) {
	  do_alarmf(state, cinfo->channel, "low rate on %s: %d", rate);
	}

	if (rate > cinfo->max) {
	  do_alarmf(state, cinfo->channel, "high rate on %s: %d", rate);
	}

	cinfo->rxcount = 0;
      }
      last_utime = now;
    }
  }

  return 0;
}
