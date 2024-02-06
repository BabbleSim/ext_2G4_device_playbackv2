/*
 * Copyright 2018 Oticon A/S
 * Copyright 2024 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef BS_PLAYBACKV2_ARGS_H
#define BS_PLAYBACKV2_ARGS_H

#include "bs_types.h"
#include "bs_cmd_line.h"
#include "bs_cmd_line_typical.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct{
  BS_BASIC_DEVICE_OPTIONS_FIELDS
  bool txoff;
  bool rxoff;
  bool rssioff;
  bool ccaoff;
  char* inputf;
} playbackv2_args_t;

void bs_playbackv2_argsparse(int argc, char *argv[], playbackv2_args_t *args);

#ifdef __cplusplus
}
#endif

#endif
