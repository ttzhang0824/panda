#pragma once

void set_mads_state(const bool state) {
  controls_allowed_lat = state;
  disengaged_from_brakes = state;
}

void mads_acc_main_check(void) {
  if (enable_mads) {
    controls_allowed_lat = acc_main_on;
  }

  if (!acc_main_on) {
    controls_allowed = false;
    set_mads_state(false);
  }
}

void alt_button_check(void) {
  if (alt_button_pressed && !alt_button_pressed_prev) {
    controls_allowed_lat = true;
  }
  alt_button_pressed_prev = alt_button_pressed;
}

void mads_exit_controls(const bool should_disengage) {
  if (should_disengage) {
    if (controls_allowed_lat) {
      disengaged_from_brakes = true;
    }
    controls_allowed_lat = false;
  } else {
    disengaged_from_brakes = true;
  }

  controls_allowed = false;
}

void mads_resume_controls(const bool disengaged) {
  disengaged_from_brakes = false;
  if (disengaged) {
    controls_allowed_lat = true;
  }
}

void check_braking_condition(bool state, bool state_prev) {
  bool dlob = (alternative_experience & ALT_EXP_ENABLE_MADS);

  if (enable_mads) {
    if (state && (!state_prev || vehicle_moving)) {
      mads_exit_controls(dlob);
    } else if (!state && disengaged_from_brakes) {
      mads_resume_controls(dlob);
    } else {
    }
  } else {
    if (state && (!state_prev || vehicle_moving)) {
      controls_allowed = false;
      set_mads_state(false);
    }
  }
}

void mads_disengage_lateral_on_brake(void) {
  if (controls_allowed_lat) {
    disengaged_from_brakes = true;
  }
  controls_allowed_lat = false;
}
