int default_rx_hook(CAN_FIFOMailBox_TypeDef *to_push) {
  UNUSED(to_push);
  return true;
}

int block = 0;
// Custom ID for SMDPS fingerprinting
void smdps_id(void);
// Custom ID for ESCC fingerprinting
void escc_id(void);
// Send SCC11
void escc_scc11(uint32_t scc11_first_4_bytes, uint32_t scc11_second_4_bytes);
// Send SCC12
void escc_scc12(uint32_t scc12_first_4_bytes, uint32_t scc12_second_4_bytes);
// Send FCA11
void escc_fca11(uint32_t fca11_first_4_bytes, uint32_t fca11_second_4_bytes);

static void send_mdps_enable_speed(CAN_FIFOMailBox_TypeDef *to_fwd){
  bool is_speed_unit_mph = GET_BYTE(to_fwd, 2) & 0x2;

  int mdps_cutoff_speed = is_speed_unit_mph ? 76 : 120;  // factor of 2 from dbc

  int veh_clu_speed = GET_BYTE(to_fwd, 1) | (GET_BYTE(to_fwd, 2) & 0x1) << 8;

  if (veh_clu_speed < mdps_cutoff_speed) {
    to_fwd->RDLR &= 0xFFFE00FF;
    to_fwd->RDLR |= mdps_cutoff_speed << 8;
  }
};

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

uint32_t scc11_first_4_bytes = 0;
uint32_t scc11_second_4_bytes = 0;
uint32_t scc12_first_4_bytes = 0;
uint32_t scc12_second_4_bytes = 0;
uint32_t fca11_first_4_bytes = 0;
uint32_t fca11_second_4_bytes = 0;

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
    if (addr == 1265) {
      send_mdps_enable_speed(to_fwd);
    }
    smdps_id();
  }
  if (bus_num == 2) {
    // SCC11: Forward radar points to sunnypilot/openpilot
    if (addr == 1056) {
      scc11_first_4_bytes = (GET_BYTE(to_fwd, 0) | GET_BYTE(to_fwd, 1) | GET_BYTE(to_fwd, 2) | GET_BYTE(to_fwd, 3));
      scc11_second_4_bytes = (GET_BYTE(to_fwd, 4) | GET_BYTE(to_fwd, 5) | GET_BYTE(to_fwd, 6) | GET_BYTE(to_fwd, 7));
      escc_scc11(scc11_first_4_bytes, scc11_second_4_bytes);
    }
    // SCC12: Detect AEB, override and forward is_scc_msg && is_fca_msg
    if (addr == 1057) {
      int aeb_decel_cmd = GET_BYTE(to_fwd, 2);
      int aeb_req = (GET_BYTE(to_fwd, 6) >> 6) & 1U;

      scc12_first_4_bytes = (GET_BYTE(to_fwd, 0) | GET_BYTE(to_fwd, 1) | GET_BYTE(to_fwd, 2) | GET_BYTE(to_fwd, 3));
      scc12_second_4_bytes = (GET_BYTE(to_fwd, 4) | GET_BYTE(to_fwd, 5) | GET_BYTE(to_fwd, 6) | GET_BYTE(to_fwd, 7));
      escc_scc12(scc12_first_4_bytes, scc12_second_4_bytes);

      if ((aeb_decel_cmd != 0) || (aeb_req != 0)) {
        block = 0;
      }
    }
    // FCA11: Detect AEB, override and forward is_scc_msg && is_fca_msg
    if (addr == 909) {
      int CR_VSM_DecCmd = GET_BYTE(to_fwd, 1);
      int FCA_CmdAct = (GET_BYTE(to_fwd, 2) >> 4) & 1U;
      int CF_VSM_DecCmdAct = (GET_BYTE(to_fwd, 3) >> 7) & 1U;

      fca11_first_4_bytes = (GET_BYTE(to_fwd, 0) | GET_BYTE(to_fwd, 1) | GET_BYTE(to_fwd, 2) | GET_BYTE(to_fwd, 3));
      fca11_second_4_bytes = (GET_BYTE(to_fwd, 4) | GET_BYTE(to_fwd, 5) | GET_BYTE(to_fwd, 6) | GET_BYTE(to_fwd, 7));
      escc_fca11(fca11_first_4_bytes, fca11_second_4_bytes);

      if ((CR_VSM_DecCmd != 0) || (FCA_CmdAct != 0) || (CF_VSM_DecCmdAct != 0)) {
        block = 0;
      }
    }
    escc_id();
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
