int default_rx_hook(CAN_FIFOMailBox_TypeDef *to_push) {
  UNUSED(to_push);
  return true;
}

int block = 0;
void send_id(uint8_t fca_cmd_act, uint8_t aeb_cmd_act, uint8_t cf_vsm_warn_fca11, uint8_t cf_vsm_warn_scc12,
             uint8_t obj_valid, uint8_t acc_obj_lat_pos_1, uint8_t acc_obj_lat_pos_2, uint8_t acc_obj_dist_1,
             uint8_t acc_obj_dist_2, uint8_t acc_obj_rel_spd_1, uint8_t acc_obj_rel_spd_2);
void escc_scc11(uint8_t mainmode_acc, uint8_t sccinfodisplay, uint8_t alivecounteracc, uint8_t vsetdis, uint8_t objvalid,
                uint8_t driveralertdisplay, uint8_t taugapset, uint8_t navi_scc_curve_status, uint8_t navi_scc_curve_act,
                uint8_t navi_scc_camera_act, uint8_t navi_scc_camera_status, uint8_t acc_objstatus,
                uint8_t acc_objlatpos_1, uint8_t acc_objlatpos_2, uint8_t acc_objrelspd_1, uint8_t acc_objrelspd_2,
                uint8_t acc_objdist_1, uint8_t acc_objdist_2);
void escc_scc12(uint8_t cf_vsm_prefill, uint8_t cf_vsm_deccmdact, uint8_t cf_vsm_hbacmd, uint8_t cf_vsm_warn,
                uint8_t cf_vsm_stat, uint8_t cf_vsm_beltcmd, uint8_t accfailinfo, uint8_t accmode, uint8_t stopreq,
                uint8_t cr_vsm_deccmd, uint8_t takeoverreq, uint8_t prefill, uint8_t cf_vsm_confmode, uint8_t aeb_failinfo,
                uint8_t aeb_status, uint8_t aeb_cmdact, uint8_t aeb_stopreq, uint8_t cr_vsm_alive, uint8_t cr_vsm_chksum,
                uint8_t areqvalue_1, uint8_t areqvalue_2, uint8_t areqraw_1, uint8_t areqraw_2);
void escc_scc13(uint8_t sccdrvmodervalue, uint8_t scc_equip, uint8_t aebdrvsetstatus, uint8_t lead_veh_dep_alert_usm);
void escc_scc14(uint8_t comfortbandupper, uint8_t comfortbandlower_1, uint8_t comfortbandlower_2,
                uint8_t jerkupperlimit_1, uint8_t jerkupperlimit_2, uint8_t jerklowerlimit_1, uint8_t jerklowerlimit_2,
                uint8_t accmode, uint8_t objgap);
void escc_fca11(uint8_t cf_vsm_prefill, uint8_t cf_vsm_hbacmd, uint8_t cf_vsm_warn, uint8_t cf_vsm_beltcmd,
                uint8_t cr_vsm_deccmd, uint8_t fca_status, uint8_t fca_cmdact, uint8_t fca_stopreq,
                uint8_t fca_drvsetstatus_1, uint8_t fca_drvsetstatus_2, uint8_t cf_vsm_deccmdact, uint8_t fca_failinfo,
                uint8_t fca_relativevelocity_1, uint8_t fca_relativevelocity_2, uint8_t fca_timetocollision,
                uint8_t cr_fca_alive, uint8_t cr_fca_chksum, uint8_t supplemental_counter, uint8_t paint1_status)
void escc_fca12(uint8_t fca_usm, uint8_t fca_drvsetstate);
void escc_frt_radar11(uint8_t cf_fca_equip_front_radar);

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
static int default_fwd_hook(int bus_num, CAN_FIFOMailBox_TypeDef *to_fwd) {
  int bus_fwd = -1;
  int addr = GET_ADDR(to_fwd);

  int is_scc_msg = ((addr == 1056) || (addr == 1057) || (addr == 1290) || (addr == 905));  // SCC11 || SCC12 || SCC13 || SCC14
  int is_fca_msg = ((addr == 909) || (addr == 1155));  // FCA11 || FCA12
  int is_frt_radar_msg = (addr == 1186);  // FRT_RADAR11

  if (bus_num == 0) {
    // ESCC is receiving messages from sunnypilot or openpilot
    if (is_scc_msg || is_fca_msg || is_frt_radar_msg) {
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

      uint8_t mainmode_acc = (GET_BYTE(to_fwd, 0) & 0x1);
      uint8_t sccinfodisplay = (GET_BYTE(to_fwd, 0) & 0xE);
      uint8_t alivecounteracc = (GET_BYTE(to_fwd, 0) & 0xF0);
      uint8_t vsetdis = GET_BYTE(to_fwd, 1);
      uint8_t objvalid = (GET_BYTE(to_fwd, 2) & 0x1);
      uint8_t driveralertdisplay = (GET_BYTE(to_fwd, 2) & 0x6);
      uint8_t taugapset = (GET_BYTE(to_fwd, 2) & 0x38);
      uint8_t navi_scc_curve_status = (GET_BYTE(to_fwd, 7) & 0x3);
      uint8_t navi_scc_curve_act = (GET_BYTE(to_fwd, 7) & 0xC);
      uint8_t navi_scc_camera_act = (GET_BYTE(to_fwd, 7) & 0x30);
      uint8_t navi_scc_camera_status = (GET_BYTE(to_fwd, 7) & 0xC0);
      uint8_t acc_objstatus = (GET_BYTE(to_fwd, 2) & 0xC0);
      uint8_t acc_objlatpos_1 = GET_BYTE(to_fwd, 3);
      uint8_t acc_objlatpos_2 = (GET_BYTE(to_fwd, 4) & 0x1);
      uint8_t acc_objrelspd_1 = (GET_BYTE(to_fwd, 5) & 0xF0);
      uint8_t acc_objrelspd_2 = GET_BYTE(to_fwd, 6);
      uint8_t acc_objdist_1 = (GET_BYTE(to_fwd, 4) & 0xFE);
      uint8_t acc_objdist_2 = (GET_BYTE(to_fwd, 5) & 0xF);
      escc_scc11(mainmode_acc, sccinfodisplay, alivecounteracc, vsetdis,
                 objvalid, driveralertdisplay, taugapset, navi_scc_curve_status,
                 navi_scc_curve_act, navi_scc_camera_act, navi_scc_camera_status,
                 acc_objstatus, acc_objlatpos_1, acc_objlatpos_2, acc_objrelspd_1,
                 acc_objrelspd_2, acc_objdist_1, acc_objdist_2);
    }
    // SCC12: Detect AEB, override and forward is_scc_msg && is_frt_radar_msg && is_fca_msg
    if (addr == 1057) {
      int aeb_decel_cmd = GET_BYTE(to_fwd, 2);
      int aeb_req = (GET_BYTE(to_fwd, 6) >> 6) & 1U;
      aeb_cmd_act = (GET_BYTE(to_fwd, 6) >> 6) & 1U;
      cf_vsm_warn_scc12 = ((GET_BYTE(to_fwd, 0) >> 4) & 0x2);

      uint8_t cf_vsm_prefill = (GET_BYTE(to_fwd, 0) & 0x1);
      uint8_t cf_vsm_deccmdact = (GET_BYTE(to_fwd, 0) & 0x2);
      uint8_t cf_vsm_hbacmd = (GET_BYTE(to_fwd, 0) & 0xC);
      uint8_t cf_vsm_warn = (GET_BYTE(to_fwd, 0) & 0x30);
      uint8_t cf_vsm_stat = (GET_BYTE(to_fwd, 0) & 0xC0);
      uint8_t cf_vsm_beltcmd = (GET_BYTE(to_fwd, 1) & 0x7);
      uint8_t accfailinfo = (GET_BYTE(to_fwd, 1) & 0x18);
      uint8_t accmode = (GET_BYTE(to_fwd, 1) & 0x60);
      uint8_t stopreq = (GET_BYTE(to_fwd, 1) & 0x80);
      uint8_t cr_vsm_deccmd = GET_BYTE(to_fwd, 2);
      uint8_t takeoverreq = (GET_BYTE(to_fwd, 4) & 0x8);
      uint8_t prefill = (GET_BYTE(to_fwd, 4) & 0x10);
      uint8_t cf_vsm_confmode = (GET_BYTE(to_fwd, 6) & 0x3);
      uint8_t aeb_failinfo = (GET_BYTE(to_fwd, 6) & 0xC);
      uint8_t aeb_status = (GET_BYTE(to_fwd, 6) & 0x30);
      uint8_t aeb_cmdact = (GET_BYTE(to_fwd, 6) & 0x60);
      uint8_t aeb_stopreq = (GET_BYTE(to_fwd, 6) & 0x80);
      uint8_t cr_vsm_alive = (GET_BYTE(to_fwd, 7) & 0xF);
      uint8_t cr_vsm_chksum = (GET_BYTE(to_fwd, 7) & 0xF0);
      uint8_t areqvalue_1 = (GET_BYTE(to_fwd, 4) & 0xE0);
      uint8_t areqvalue_2 = GET_BYTE(to_fwd, 5);
      uint8_t areqraw_1 = GET_BYTE(to_fwd, 3);
      uint8_t areqraw_2 = (GET_BYTE(to_fwd, 4) & 0x7);
      escc_scc12(cf_vsm_prefill, cf_vsm_deccmdact, cf_vsm_hbacmd, cf_vsm_warn,
                 cf_vsm_stat, cf_vsm_beltcmd, accfailinfo, accmode, stopreq,
                 cr_vsm_deccmd, takeoverreq, prefill, cf_vsm_confmode, aeb_failinfo,
                 aeb_status, aeb_cmdact, aeb_stopreq, cr_vsm_alive, cr_vsm_chksum,
                 areqvalue_1, areqvalue_2, areqraw_1, areqraw_2);

      if ((aeb_decel_cmd != 0) || (aeb_req != 0)) {
        block = 0;
      }
    }
    if (addr == 1290) {
      uint8_t sccdrvmodervalue, uint8_t scc_equip, uint8_t aebdrvsetstatus, uint8_t lead_veh_dep_alert_usm
    }
    if (addr == 905) {
      uint8_t comfortbandupper, uint8_t comfortbandlower_1, uint8_t comfortbandlower_2,
      uint8_t jerkupperlimit_1, uint8_t jerkupperlimit_2, uint8_t jerklowerlimit_1, uint8_t jerklowerlimit_2,
      uint8_t accmode, uint8_t objgap
    }
    // FCA11: Detect FCW, override and forward is_scc_msg && is_frt_radar_msg && is_fca_msg
    if (addr == 909) {
      int CR_VSM_DecCmd = GET_BYTE(to_fwd, 1);
      int FCA_CmdAct = (GET_BYTE(to_fwd, 2) >> 4) & 1U;
      int CF_VSM_DecCmdAct = (GET_BYTE(to_fwd, 3) >> 7) & 1U;
      fca_cmd_act = (GET_BYTE(to_fwd, 2) >> 4) & 1U;
      cf_vsm_warn_fca11 = ((GET_BYTE(to_fwd, 0) >> 2) & 0x2);
      if ((CR_VSM_DecCmd != 0) || (FCA_CmdAct != 0) || (CF_VSM_DecCmdAct != 0)) {
        block = 0;
      }
    }
    if (addr == 1155) {
      uint8_t fca_usm, uint8_t fca_drvsetstate
    }
    if (addr == 1186) {
      uint8_t cf_fca_equip_front_radar
    }
    send_id(fca_cmd_act, aeb_cmd_act, cf_vsm_warn_fca11, cf_vsm_warn_scc12 , obj_valid, acc_obj_lat_pos_1, acc_obj_lat_pos_2, acc_obj_dist_1, acc_obj_dist_2, acc_obj_rel_spd_1, acc_obj_rel_spd_2);
    int block_msg = (block && (is_scc_msg || is_fca_msg || is_frt_radar_msg));
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
