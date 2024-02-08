/*
 * Copyright 2018 Oticon A/S
 * Copyright 2024 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include "bs_tracing.h"
#include "bs_oswrap.h"
#include "bs_pc_2G4.h"
#include "bs_pc_2G4_types.h"
#include "bs_pc_2G4_utils.h"
#include "bs_playbackv2_args.h"


/**
 * This devices plays back the activity of another device as it was recorded
 * by the phy in a previous simulation
 * (Note that powers and antenna gains will have minor rounding errors)
 */

static FILE* tx_f= NULL, *rx_f= NULL, *RSSI_f = NULL, *CCA_f = NULL;

static p2G4_txv2_t tx_s;
uint8_t *tx_packet = NULL;
static p2G4_rssi_t RSSI_s;
static p2G4_rxv2_t rx_s;
static p2G4_address_t rx_phy_addr[16];
static p2G4_cca_t cca_s;

static void open_one_input_file(const char* inputf, const char *type,
                                char *filename, FILE **file){
  sprintf(filename,"%s.%s.csv",inputf, type);
  *file = bs_fopen(filename, "r");
  bs_skipline(*file); //skip heading
  if ( feof(*file) ){
    bs_trace_raw(3,"%s file %s is empty => will not use it\n",type, filename);
    fclose(*file);
    *file = NULL;
  }
}

static void open_input_files(const char* inputf, playbackv2_args_t* args){
  char *filename;
  filename = bs_calloc(strlen(inputf) + 10,sizeof(char));

  if ( args->rxoff == false ) {
    open_one_input_file(inputf, "Rxv2", filename, &rx_f);
  }
  if ( args->txoff == false ) {
    open_one_input_file(inputf, "Txv2", filename, &tx_f);
  }
  if ( args->rssioff == false ) {
    open_one_input_file(inputf, "RSSI", filename, &RSSI_f);
  }
  if ( args->ccaoff == false ) {
    open_one_input_file(inputf, "CCA", filename, &CCA_f);
  }


  free(filename);

  if ((RSSI_f == NULL) && (tx_f == NULL) && (rx_f == NULL) && (CCA_f == NULL)) {
    bs_trace_warning_line("No input in any of the files??\n");
  }
}

void close_input_files(){
  if (tx_f !=NULL) {
    fclose(tx_f);
  }
  if (rx_f !=NULL) {
    fclose(rx_f);
  }
  if (RSSI_f !=NULL) {
    fclose(RSSI_f);
  }
  if (CCA_f !=NULL) {
    fclose(CCA_f);
  }
}

void read_next_tx(){

  if (!tx_f) {
    ;
  } else if (feof(tx_f)) {
    fclose(tx_f);
    tx_f = NULL;
  } else {
    int read;
    double center_freq = 0;
    double power = 0;
    p2G4_freq_t freq;
    p2G4_txv2_t *txs = &tx_s;
    read = fscanf(tx_f,
        "%"SCNtime",%"SCNtime",\
        %"SCNtime",%"SCNtime",\
        %lf,\
        0x%"SCNx64",%"SCNu16",\
        %"SCNu16",\
        %lf,\
        %"SCNtime",%"SCNtime",\
        %"SCNu16",",

        &txs->start_tx_time, &txs->end_tx_time,
        &txs->start_packet_time, &txs->end_packet_time,

        &center_freq,

        &txs->phy_address,
        &txs->radio_params.modulation,
        &txs->coding_rate,

        &power,

        &txs->abort.abort_time,
        &txs->abort.recheck_time,

        &txs->packet_size);

    p2G4_freq_from_d(center_freq, 0, &freq);

    txs->radio_params.center_freq = freq;
    txs->power_level = p2G4_power_from_d(power);

    if (read < 12) {
      if ((read > 0) || !feof(tx_f)) { //otherwise it was probably an empty trailing line
        bs_trace_warning_line("Corrupted input Tx file disabling it (%i)\n", read);
      } else {
        bs_trace_raw(3,"Reached end of Tx file\n");
      }
      fclose(tx_f);
      tx_f = NULL;
    } else {
      if (txs->packet_size > 0) {
        char buffer[txs->packet_size*3 + 1];
        if (tx_packet != NULL) { free(tx_packet); }
        tx_packet = bs_calloc(txs->packet_size, sizeof(char));
        bs_readline(buffer, txs->packet_size*3 + 1, tx_f);
        bs_read_hex_dump(buffer, tx_packet, txs->packet_size);
      } else {
        bs_skipline(tx_f); //skip \n
      }
    }
  }

  if (tx_f == NULL) {
    tx_s.start_tx_time = TIME_NEVER;
  }
}

static bool rx_check_read(int count, int expected) {
  if ( count < expected ){
    if ( ( count > 0 ) || !feof(rx_f) ) { //otherwise it was probably an empty trailing line
      bs_trace_warning_line("Corrupted input Rx file disabling it (%i %i)\n",count, feof(rx_f));
    } else {
      bs_trace_raw(3,"Reached end of Rx file\n");
    }
    fclose(rx_f);
    rx_f = NULL;
    rx_s.start_time = TIME_NEVER;
    return true;
  }
  return false;
}

void read_next_rx(){

  if (!rx_f) {
    ;
  } else if (feof(rx_f)) {
    fclose(rx_f);
    rx_f = NULL;
  } else {
    int read;
    double center_freq = 0;
    double ant_gain = 0;
    p2G4_freq_t freq;
    p2G4_rxv2_t *req = &rx_s;
    read = fscanf(rx_f,
        "%"SCNtime",%"SCNu32",\
        %"SCNu8",\"[",
        &req->start_time,
        &req->scan_duration,
        &req->n_addr);

    if (rx_check_read(read, 3)) return;

    for (int i = 0 ; i < req->n_addr ; i++) {
      read +=fscanf(rx_f,
                    "0x%08"PRIx64,
                    (uint64_t*)&rx_phy_addr[i]);
      if (i < (int)req->n_addr - 1) {
        read +=fscanf(rx_f,",");
      }
    }
    if (rx_check_read(read, 3 + req->n_addr)) return;

    read = fscanf(rx_f,
                  "]\", %"SCNu16","
                  "%lf,%lf,"

                  "%"SCNu16","
                  "%"SCNu16",%"SCNu16","
                  "%"SCNu16","
                  "%"SCNu16",%"SCNu32","

                  "%"SCNu32","
                  "%"SCNu16","
                  "%"SCNu8","

                  "%"SCNu8","
                  "%"SCNtime",%"SCNtime",",
                  &req->radio_params.modulation,
                  &center_freq,
                  &ant_gain,

                  &req->acceptable_pre_truncation,
                  &req->sync_threshold,
                  &req->header_threshold,
                  &req->pream_and_addr_duration,
                  &req->header_duration,
                  &req->error_calc_rate,

                  &req->forced_packet_duration,
                  &req->coding_rate,
                  &req->prelocked_tx,

                  &req->resp_type,
                  &req->abort.abort_time,
                  &req->abort.recheck_time
                  );

    if (rx_check_read(read, 15)) return;

    p2G4_freq_from_d(center_freq, 0, &freq);

    req->radio_params.center_freq = freq;
    req->antenna_gain = p2G4_power_from_d(ant_gain);

    bs_skipline(rx_f); //skip remainder of the line
  }

  if (rx_f == NULL) {
    rx_s.start_time = TIME_NEVER;
  }
}

void read_next_RSSI() {

  if (!RSSI_f){
    ;
  } else if (feof(RSSI_f)) {
    fclose(RSSI_f);
    RSSI_f = NULL;
  } else {
    int read;
    double center_freq = 0;
    double ant_gain = 0;
    p2G4_rssi_t *Req = &RSSI_s;
    p2G4_freq_t freq;
    read = fscanf(RSSI_f,
        "%"SCNtime","
        "%"SCNu16","
        "%lf,"
        "%lf",
        &Req->meas_time,
        &Req->radio_params.modulation,
        &center_freq,
        &ant_gain
        );

    p2G4_freq_from_d(center_freq, 0, &freq);

    Req->radio_params.center_freq = freq;
    Req->antenna_gain = p2G4_power_from_d(ant_gain);

    if ( read < 3 ){
      if ( ( read > 0 ) || !feof(RSSI_f) ){ //otherwise it was probably an empty trailing line
        bs_trace_warning_line("Corrupted input RSSI file disabling it\n");
      } else {
        bs_trace_raw(3,"Reached end of RSSI file\n");
      }
      fclose(RSSI_f);
      RSSI_f = NULL;
    } else {
      bs_skipline(RSSI_f); //skip remainder of the line
    }
  }

  if (RSSI_f == NULL){
    RSSI_s.meas_time = TIME_NEVER;
  }
}

void read_next_CCA() {

  if (!CCA_f){
    ;
  } else if (feof(CCA_f)) {
    fclose(CCA_f);
    CCA_f = NULL;
  } else {
    int read;
    double center_freq = 0;
    double ant_gain = 0;
    double mod_thresh = 0;
    double rssi_thresh = 0;

    p2G4_cca_t *req = &cca_s;
    p2G4_freq_t freq;
    read = fscanf(CCA_f,
        "%"SCNtime","
        "%"SCNu32","
        "%"SCNu32","

        "%"SCNu16","
        "%lf,"
        "%lf,"

        "%lf,"
        "%lf,"
        "%"SCNu8","

        "%"SCNtime",%"SCNtime,

        &req->start_time,
        &req->scan_duration,
        &req->scan_period,

        &req->radio_params.modulation,
        &center_freq,
        &ant_gain,

        &mod_thresh,
        &rssi_thresh,
        &req->stop_when_found,

        &req->abort.abort_time,
        &req->abort.recheck_time
        );

    p2G4_freq_from_d(center_freq, 0, &freq);

    req->radio_params.center_freq = freq;
    req->antenna_gain = p2G4_power_from_d(ant_gain);
    req->mod_threshold = p2G4_RSSI_value_from_dBm(mod_thresh);
    req->rssi_threshold = p2G4_RSSI_value_from_dBm(rssi_thresh);

    if (read < 11) {
      if ((read > 0) || !feof(CCA_f)) { //otherwise it was probably an empty trailing line
        bs_trace_warning_line("Corrupted input CCA file disabling it\n");
      } else {
        bs_trace_raw(3,"Reached end of CCA file\n");
      }
      fclose(CCA_f);
      CCA_f = NULL;
    } else {
      bs_skipline(CCA_f); //skip remainder of the line
    }
  }

  if (CCA_f == NULL) {
    cca_s.start_time = TIME_NEVER;
  }
}

int main(int argc, char *argv[]) {
  playbackv2_args_t args;
  uint8_t *packet_ptr;

  bs_playbackv2_argsparse(argc, argv, &args);

  open_input_files(args.inputf, &args);

  p2G4_dev_initcom_c(args.device_nbr, args.s_id, args.p_id, NULL);

  read_next_tx();
  read_next_rx();
  read_next_RSSI();
  read_next_CCA();

  while ((tx_f !=  NULL) || (rx_f!= NULL) || (RSSI_f != NULL) || (CCA_f != NULL)) {
    int result = -1;
    if ((tx_f !=  NULL) &&
        (tx_s.start_tx_time < rx_s.start_time) &&
        (tx_s.start_tx_time < RSSI_s.meas_time) &&
        (tx_s.start_tx_time < cca_s.start_time)) {
      p2G4_tx_done_t tx_done_s;
      result =  p2G4_dev_req_txv2_c_b(&tx_s, tx_packet, &tx_done_s);
      read_next_tx();
    } else if ((rx_f !=  NULL) &&
              (rx_s.start_time < tx_s.start_tx_time) &&
              (rx_s.start_time < RSSI_s.meas_time) &&
              (rx_s.start_time < cca_s.start_time)) {
      p2G4_rxv2_done_t rx_done_s;
      packet_ptr = NULL;
      result = p2G4_dev_req_rxv2_c_b(&rx_s, rx_phy_addr, &rx_done_s, &packet_ptr, 0, NULL);
      free(packet_ptr);
      read_next_rx();
    } else if ((RSSI_f !=  NULL) &&
              (RSSI_s.meas_time < rx_s.start_time) &&
              (RSSI_s.meas_time < tx_s.start_tx_time) &&
              (RSSI_s.meas_time < cca_s.start_time)) {
      p2G4_rssi_done_t RSSI_done_s;
      result = p2G4_dev_req_RSSI_c_b(&RSSI_s, &RSSI_done_s);
      read_next_RSSI();
    } else if ((CCA_f !=  NULL) &&
        (cca_s.start_time < rx_s.start_time) &&
        (cca_s.start_time < tx_s.start_tx_time) &&
        (cca_s.start_time < RSSI_s.meas_time)) {
      p2G4_cca_done_t cca_done_s;
      result = p2G4_dev_req_cca_c_b(&cca_s, &cca_done_s);
      read_next_CCA();
    }
    if (result == -1) {
      bs_trace_raw(3,"We have been disconnected\n");
      break;
    }
  }

  close_input_files();
  if (tx_packet != NULL) free(tx_packet);

  p2G4_dev_disconnect_c();
  return 0;
}
