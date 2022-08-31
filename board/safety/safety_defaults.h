int default_rx_hook(CAN_FIFOMailBox_TypeDef *to_push) {
  UNUSED(to_push);
  return true;
}

int block = 0;
// Custom ID for ESCC fingerprinting, lead car info (not radar tracks), AEB/FCW signals
void escc_id(uint8_t fca_cmd_act, uint8_t aeb_cmd_act, uint8_t cf_vsm_warn_fca11, uint8_t cf_vsm_warn_scc12, uint8_t cf_vsm_deccmdact_scc12, uint8_t cf_vsm_deccmdact_fca11, uint8_t cr_vsm_deccmd_scc12, uint8_t cr_vsm_deccmd_fca11,
             uint8_t obj_valid, uint8_t acc_objstatus, uint8_t acc_obj_lat_pos_1, uint8_t acc_obj_lat_pos_2, uint8_t acc_obj_dist_1,
             uint8_t acc_obj_dist_2, uint8_t acc_obj_rel_spd_1, uint8_t acc_obj_rel_spd_2);

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

// Initialize bytes to send to 2AB
uint8_t fca_cmd_act = 0;
uint8_t aeb_cmd_act = 0;
uint8_t cf_vsm_warn_fca11 = 0;
uint8_t cf_vsm_warn_scc12 = 0;
uint8_t cf_vsm_deccmdact_scc12 = 0;
uint8_t cf_vsm_deccmdact_fca11 = 0;
uint8_t cr_vsm_deccmd_scc12 = 0;
uint8_t cr_vsm_deccmd_fca11 = 0;
uint8_t obj_valid = 0;
uint8_t acc_objstatus = 0;
uint8_t acc_obj_lat_pos_1 = 0;
uint8_t acc_obj_lat_pos_2 = 0;
uint8_t acc_obj_dist_1 = 0;
uint8_t acc_obj_dist_2 = 0;
uint8_t acc_obj_rel_spd_1 = 0;
uint8_t acc_obj_rel_spd_2 = 0;

static int default_fwd_hook(int bus_num, CAN_FIFOMailBox_TypeDef *to_fwd) {
  int bus_fwd = -1;
  int addr = GET_ADDR(to_fwd);

  int is_scc_msg = ((addr == 1056) || (addr == 1057) || (addr == 1290) || (addr == 905));  // SCC11 || SCC12 || SCC13 || SCC14
  int is_fca_msg = ((addr == 909) || (addr == 1155));  // FCA11 || FCA12

  if (bus_num == 0) {
    // ESCC is receiving messages from sunnypilot/openpilot
    if (is_scc_msg || is_fca_msg) {
      block = 1;
    }
    bus_fwd = 2;
  }
  if (bus_num == 2) {
    // SCC11: Forward radar points to sunnypilot/openpilot
    if (addr == 1056) {
      obj_valid = (GET_BYTE(to_fwd, 2) & 0x1U);
      acc_objstatus = ((GET_BYTE(to_fwd, 2) >> 6) & 0x3U);
      acc_obj_lat_pos_1 = GET_BYTE(to_fwd, 3);
      acc_obj_lat_pos_2 = (GET_BYTE(to_fwd, 4) & 0x1U);
      acc_obj_dist_1 = ((GET_BYTE(to_fwd, 4) >> 1) & 0x7FU);
      acc_obj_dist_2 = (GET_BYTE(to_fwd, 5) & 0xFU);
      acc_obj_rel_spd_1 = ((GET_BYTE(to_fwd, 5) >> 4) & 0xFU);
      acc_obj_rel_spd_2 = GET_BYTE(to_fwd, 6);
    }
    // SCC12: Detect AEB, override and forward is_scc_msg && is_fca_msg
    if (addr == 1057) {
      aeb_cmd_act = (GET_BYTE(to_fwd, 6) >> 6) & 1U;
      cf_vsm_warn_scc12 = ((GET_BYTE(to_fwd, 0) >> 4) & 0x3U);
      cf_vsm_deccmdact_scc12 = (GET_BYTE(to_fwd, 0) >> 1) & 1U;
      cr_vsm_deccmd_scc12 = GET_BYTE(to_fwd, 2);
    }
    // FCA11: Detect AEB, override and forward is_scc_msg && is_fca_msg
    if (addr == 909) {
      fca_cmd_act = (GET_BYTE(to_fwd, 2) >> 4) & 1U;
      cf_vsm_warn_fca11 = ((GET_BYTE(to_fwd, 0) >> 3) & 0x3U);
      cf_vsm_deccmdact_fca11 = ((GET_BYTE(to_fwd, 3) >> 7) & 1U);
      cr_vsm_deccmd_fca11 = GET_BYTE(to_fwd, 1);
    }
    escc_id(fca_cmd_act, aeb_cmd_act, cf_vsm_warn_fca11, cf_vsm_warn_scc12, cf_vsm_deccmdact_scc12, cf_vsm_deccmdact_fca11, cr_vsm_deccmd_scc12, cr_vsm_deccmd_fca11, obj_valid, acc_objstatus, acc_obj_lat_pos_1, acc_obj_lat_pos_2, acc_obj_dist_1, acc_obj_dist_2, acc_obj_rel_spd_1, acc_obj_rel_spd_2);
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
