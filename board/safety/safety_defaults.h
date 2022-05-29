int default_rx_hook(CAN_FIFOMailBox_TypeDef *to_push) {
  UNUSED(to_push);
  return true;
}

int block = 0;
// Custom ID with mix of radar tracks and AEB/FCW signals
void send_id(uint8_t fca_cmd_act, uint8_t aeb_cmd_act, uint8_t cf_vsm_warn_fca11, uint8_t cf_vsm_warn_scc12,
             uint8_t obj_valid, uint8_t acc_obj_lat_pos_1, uint8_t acc_obj_lat_pos_2, uint8_t acc_obj_dist_1,
             uint8_t acc_obj_dist_2, uint8_t acc_obj_rel_spd_1, uint8_t acc_obj_rel_spd_2);
// Send SCC11 to bus1
void escc_scc11(uint32_t scc11_first_4_bytes, uint32_t scc11_second_4_bytes);
// Send SCC12 to bus1
void escc_scc12(uint32_t scc12_first_4_bytes, uint32_t scc12_second_4_bytes);
//void escc_scc13(uint32_t scc13_first_4_bytes, uint32_t scc13_second_4_bytes);
//void escc_scc14(uint32_t scc14_first_4_bytes, uint32_t scc14_second_4_bytes);
// Send FCA11 to bus1
void escc_fca11(uint32_t fca11_first_4_bytes, uint32_t fca11_second_4_bytes);
//void escc_fca12(uint32_t fca12_first_4_bytes, uint32_t fca12_second_4_bytes);

// *** no output safety mode ***

static void nooutput_init(int16_t param) {
  UNUSED(param);
  controls_allowed = false;
  relay_malfunction_reset();
}

static int nooutput_tx_hook(CAN_FIFOMailBox_TypeDef *to_send) {
  UNUSED(to_send);
  return false;
}

static int nooutput_tx_lin_hook(int lin_num, uint8_t *data, int len) {
  UNUSED(lin_num);
  UNUSED(data);
  UNUSED(len);
  return false;
}

uint8_t fca_cmd_act = 0;
uint8_t aeb_cmd_act = 0;
uint8_t cf_vsm_warn_fca11 = 0;
uint8_t cf_vsm_warn_scc12 = 0;
// Initialize variables to store radar points bytes to send to 2AA
uint8_t obj_valid = 0;
uint8_t acc_obj_lat_pos_1 = 0;
uint8_t acc_obj_lat_pos_2 = 0;
uint8_t acc_obj_dist_1 = 0;
uint8_t acc_obj_dist_2 = 0;
uint8_t acc_obj_rel_spd_1 = 0;
uint8_t acc_obj_rel_spd_2 = 0;
uint32_t scc11_first_4_bytes = 0;
uint32_t scc11_second_4_bytes = 0;
uint32_t scc12_first_4_bytes = 0;
uint32_t scc12_second_4_bytes = 0;
/**uint32_t scc13_first_4_bytes = 0;
uint32_t scc13_second_4_bytes = 0;
uint32_t scc14_first_4_bytes = 0;
uint32_t scc14_second_4_bytes = 0;**/
uint32_t fca11_first_4_bytes = 0;
uint32_t fca11_second_4_bytes = 0;
/**uint32_t fca12_first_4_bytes = 0;
uint32_t fca12_second_4_bytes = 0;**/

static int default_fwd_hook(int bus_num, CAN_FIFOMailBox_TypeDef *to_fwd) {
  int bus_fwd = -1;
  int addr = GET_ADDR(to_fwd);

  int is_scc_msg = ((addr == 1056) || (addr == 1057) || (addr == 1290) || (addr == 905));  // SCC11 || SCC12 || SCC13 || SCC14
  int is_fca_msg = ((addr == 909) || (addr == 1155));  // FCA11 || FCA12

  if (bus_num == 0) {
    // ESCC is receiving messages from sunnypilot or openpilot
    if (is_scc_msg || is_fca_msg) {
      block = 1;
    }
    bus_fwd = 2;
  }
  if (bus_num == 2) {
    // TODO: Do we still need to send a custom messages/ID?
    // SCC11: Forward radar points to 0x2AA
    if (addr == 1056) {
      obj_valid = (GET_BYTE(to_fwd, 2) & 0x1);
      acc_obj_lat_pos_1 = GET_BYTE(to_fwd, 3);
      acc_obj_lat_pos_2 = (GET_BYTE(to_fwd, 4) & 0x1);
      acc_obj_dist_1 = (GET_BYTE(to_fwd, 4) & 0xFE);
      acc_obj_dist_2 = (GET_BYTE(to_fwd, 5) & 0xF);
      acc_obj_rel_spd_1 = (GET_BYTE(to_fwd, 5) & 0xF0);
      acc_obj_rel_spd_2 = GET_BYTE(to_fwd, 6);

      scc11_first_4_bytes = (GET_BYTE(to_fwd, 0) | GET_BYTE(to_fwd, 1) | GET_BYTE(to_fwd, 2) | GET_BYTE(to_fwd, 3));
      scc11_second_4_bytes = (GET_BYTE(to_fwd, 4) | GET_BYTE(to_fwd, 5) | GET_BYTE(to_fwd, 6) | GET_BYTE(to_fwd, 7));
      escc_scc11(scc11_first_4_bytes, scc11_second_4_bytes);
    }
    // SCC12: Detect AEB, override and forward is_scc_msg && is_fca_msg
    if (addr == 1057) {
      int aeb_decel_cmd = GET_BYTE(to_fwd, 2);
      int aeb_req = (GET_BYTE(to_fwd, 6) >> 6) & 1U;
      aeb_cmd_act = (GET_BYTE(to_fwd, 6) >> 6) & 1U;
      cf_vsm_warn_scc12 = ((GET_BYTE(to_fwd, 0) >> 4) & 0x2);

      scc12_first_4_bytes = (GET_BYTE(to_fwd, 0) | GET_BYTE(to_fwd, 1) | GET_BYTE(to_fwd, 2) | GET_BYTE(to_fwd, 3));
      scc12_second_4_bytes = (GET_BYTE(to_fwd, 4) | GET_BYTE(to_fwd, 5) | GET_BYTE(to_fwd, 6) | GET_BYTE(to_fwd, 7));
      escc_scc12(scc12_first_4_bytes, scc12_second_4_bytes);

      if ((aeb_decel_cmd != 0) || (aeb_req != 0)) {
        block = 0;
      }
    }
    /**if (addr == 1290) {
      scc13_first_4_bytes = (GET_BYTE(to_fwd, 0) | GET_BYTE(to_fwd, 1) | GET_BYTE(to_fwd, 2) | GET_BYTE(to_fwd, 3));
      scc13_second_4_bytes = (GET_BYTE(to_fwd, 4) | GET_BYTE(to_fwd, 5) | GET_BYTE(to_fwd, 6) | GET_BYTE(to_fwd, 7));
      escc_scc13(scc13_first_4_bytes, scc13_second_4_bytes);
    }
    if (addr == 905) {
      scc14_first_4_bytes = (GET_BYTE(to_fwd, 0) | GET_BYTE(to_fwd, 1) | GET_BYTE(to_fwd, 2) | GET_BYTE(to_fwd, 3));
      scc14_second_4_bytes = (GET_BYTE(to_fwd, 4) | GET_BYTE(to_fwd, 5) | GET_BYTE(to_fwd, 6) | GET_BYTE(to_fwd, 7));
      escc_scc14(scc14_first_4_bytes, scc14_second_4_bytes);
    }**/
    // FCA11: Detect FCW, override and forward is_scc_msg && is_fca_msg
    if (addr == 909) {
      int CR_VSM_DecCmd = GET_BYTE(to_fwd, 1);
      int FCA_CmdAct = (GET_BYTE(to_fwd, 2) >> 4) & 1U;
      int CF_VSM_DecCmdAct = (GET_BYTE(to_fwd, 3) >> 7) & 1U;
      fca_cmd_act = (GET_BYTE(to_fwd, 2) >> 4) & 1U;
      cf_vsm_warn_fca11 = ((GET_BYTE(to_fwd, 0) >> 2) & 0x2);

      fca11_first_4_bytes = (GET_BYTE(to_fwd, 0) | GET_BYTE(to_fwd, 1) | GET_BYTE(to_fwd, 2) | GET_BYTE(to_fwd, 3));
      fca11_second_4_bytes = (GET_BYTE(to_fwd, 4) | GET_BYTE(to_fwd, 5) | GET_BYTE(to_fwd, 6) | GET_BYTE(to_fwd, 7));
      escc_fca11(fca11_first_4_bytes, fca11_second_4_bytes);

      if ((CR_VSM_DecCmd != 0) || (FCA_CmdAct != 0) || (CF_VSM_DecCmdAct != 0)) {
        block = 0;
      }
    }
    /**if (addr == 1155) {
      fca12_first_4_bytes = (GET_BYTE(to_fwd, 0) | GET_BYTE(to_fwd, 1) | GET_BYTE(to_fwd, 2) | GET_BYTE(to_fwd, 3));
      fca12_second_4_bytes = (GET_BYTE(to_fwd, 4) | GET_BYTE(to_fwd, 5) | GET_BYTE(to_fwd, 6) | GET_BYTE(to_fwd, 7));
      escc_fca12(fca12_first_4_bytes, fca12_second_4_bytes);
    }**/
    send_id(fca_cmd_act, aeb_cmd_act, cf_vsm_warn_fca11, cf_vsm_warn_scc12 , obj_valid, acc_obj_lat_pos_1, acc_obj_lat_pos_2, acc_obj_dist_1, acc_obj_dist_2, acc_obj_rel_spd_1, acc_obj_rel_spd_2);
    int block_msg = (block && (is_scc_msg || is_fca_msg));
    if (!block_msg) {
      bus_fwd = 0;
    }
  }
  return bus_fwd;
}

const safety_hooks nooutput_hooks = {
  .init = nooutput_init,
  .rx = default_rx_hook,
  .tx = nooutput_tx_hook,
  .tx_lin = nooutput_tx_lin_hook,
  .fwd = default_fwd_hook,
};

// *** all output safety mode ***

static void alloutput_init(int16_t param) {
  UNUSED(param);
  controls_allowed = true;
  relay_malfunction_reset();
}

static int alloutput_tx_hook(CAN_FIFOMailBox_TypeDef *to_send) {
  UNUSED(to_send);
  return true;
}

static int alloutput_tx_lin_hook(int lin_num, uint8_t *data, int len) {
  UNUSED(lin_num);
  UNUSED(data);
  UNUSED(len);
  return true;
}

const safety_hooks alloutput_hooks = {
  .init = alloutput_init,
  .rx = default_rx_hook,
  .tx = alloutput_tx_hook,
  .tx_lin = alloutput_tx_lin_hook,
  .fwd = default_fwd_hook,
};
