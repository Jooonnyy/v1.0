#include "board_lift.h"
#include "board.h"

static int32_t position_pulses;
static int32_t target_pulses;
static uint8_t pending;
static uint8_t busy;
static volatile uint8_t stop_request;
static uint32_t last_requested_pulses;
static uint32_t last_issued_pulses;

static int32_t div_round(int64_t numerator, int32_t denominator)
{
  int64_t half = (int64_t)denominator / 2;
  if (numerator >= 0) return (int32_t)((numerator + half) / denominator);
  return (int32_t)(-((-numerator + half) / denominator));
}

static uint32_t magnitude_pulses(int32_t value_pulses)
{
  int64_t value_wide_pulses = value_pulses;
  return (uint32_t)(value_wide_pulses < 0 ? -value_wide_pulses : value_wide_pulses);
}

int32_t Lift_PulsesFromHeightUm(int32_t height_um)
{
  return div_round((int64_t)height_um * (int64_t)LIFT_MICROSTEPS_PER_REV,
                   LIFT_SCREW_LEAD_UM);
}

int32_t Lift_HeightUmFromPulses(int32_t pulses)
{
  return div_round((int64_t)pulses * (int64_t)LIFT_SCREW_LEAD_UM,
                   (int32_t)LIFT_MICROSTEPS_PER_REV);
}

int32_t Lift_AngleMdegFromHeightUm(int32_t height_um)
{
  return div_round((int64_t)height_um * LIFT_FULL_TURN_MDEG,
                   LIFT_SCREW_LEAD_UM);
}

int32_t Lift_HeightUmFromAngleMdeg(int32_t angle_mdeg)
{
  return div_round((int64_t)angle_mdeg * (int64_t)LIFT_SCREW_LEAD_UM,
                   (int32_t)LIFT_FULL_TURN_MDEG);
}

static int32_t soft_min_pulses(void)
{
  return Lift_PulsesFromHeightUm(LIFT_MIN_UM);
}

static int32_t soft_max_pulses(void)
{
  return Lift_PulsesFromHeightUm(LIFT_MAX_UM);
}

static uint8_t limit_blocks(uint8_t going_up)
{
#if LIFT_USE_HW_LIMITS
  return going_up ? Input_Active(INPUT_LIMIT_TOP)
                  : Input_Active(INPUT_LIMIT_BOTTOM);
#else
  (void)going_up;
  return 0U;
#endif
}

static uint32_t period_us_for(uint32_t index_pulses, uint32_t total_pulses)
{
  const uint32_t cruise_period_us = (uint32_t)LIFT_STEP_PERIOD_US;
  const uint32_t start_period_us = (uint32_t)LIFT_START_PERIOD_US;
  uint32_t ramp_pulses = (uint32_t)LIFT_RAMP_PULSES;
  uint32_t edge_pulses;
  uint32_t span_us;

  if (ramp_pulses == 0U || start_period_us <= cruise_period_us)
    return cruise_period_us;
  if (ramp_pulses > total_pulses / 2U) ramp_pulses = total_pulses / 2U;
  if (ramp_pulses == 0U) return cruise_period_us;

  edge_pulses = index_pulses;
  if (total_pulses - 1U - index_pulses < edge_pulses)
    edge_pulses = total_pulses - 1U - index_pulses;
  if (edge_pulses >= ramp_pulses) return cruise_period_us;

  span_us = start_period_us - cruise_period_us;
  return start_period_us -
         (uint32_t)(((uint64_t)span_us * (edge_pulses + 1U)) / ramp_pulses);
}

static LiftRunResult run_move(int32_t delta_pulses)
{
  uint32_t total_pulses = magnitude_pulses(delta_pulses);
  uint32_t issued_pulses = 0U;
  uint32_t index_pulses;
  uint8_t going_up = (delta_pulses > 0) ? 1U : 0U;
  uint8_t direction_high;
  LiftRunResult result = LIFT_RUN_COMPLETE;

  if (total_pulses == 0U) return LIFT_RUN_COMPLETE;

  direction_high = going_up ? (uint8_t)(LIFT_DIR_HIGH_IS_UP != 0)
                            : (uint8_t)(LIFT_DIR_HIGH_IS_UP == 0);
  Stepper_Direction(STEPPER_1, direction_high);
  Board_DelayUs(10U);
  Stepper_Enable(STEPPER_1, 1U);
  HAL_Delay(2U);

  for (index_pulses = 0U; index_pulses < total_pulses; ++index_pulses) {
    if (stop_request) { result = LIFT_RUN_STOPPED; break; }
    if (limit_blocks(going_up)) { result = LIFT_RUN_LIMIT; break; }
    if (Stepper_Fault(STEPPER_1)) { result = LIFT_RUN_FAULT; break; }
    Stepper_Pulse(STEPPER_1);
    ++issued_pulses;
    Board_DelayUs(period_us_for(index_pulses, total_pulses) - 6U);
  }

  position_pulses += going_up ? (int32_t)issued_pulses : -(int32_t)issued_pulses;
  last_requested_pulses = total_pulses;
  last_issued_pulses = issued_pulses;

#if LIFT_HOLD_AFTER_MOVE
  if (result != LIFT_RUN_COMPLETE) Stepper_Enable(STEPPER_1, 0U);
#else
  Stepper_Enable(STEPPER_1, 0U);
#endif
  return result;
}

static LiftStatus queue_pulses(int32_t wanted_pulses)
{
  if (pending || busy) return LIFT_BUSY;
  if (wanted_pulses < soft_min_pulses() || wanted_pulses > soft_max_pulses())
    return LIFT_ERR_RANGE;
  if (Stepper_Fault(STEPPER_1)) return LIFT_ERR_FAULT;
  target_pulses = wanted_pulses;
  pending = 1U;
  stop_request = 0U;
  return LIFT_OK;
}

void Lift_Init(void)
{
  Stepper_Enable(STEPPER_1, 0U);
  Stepper_Enable(STEPPER_2, 0U);
  position_pulses = 0;
  target_pulses = 0;
  pending = 0U;
  busy = 0U;
  stop_request = 0U;
  last_requested_pulses = 0U;
  last_issued_pulses = 0U;
}

int32_t Lift_PositionUm(void)
{
  return Lift_HeightUmFromPulses(position_pulses);
}

int32_t Lift_PositionPulses(void)
{
  return position_pulses;
}

void Lift_SetZero(void)
{
  position_pulses = 0;
  target_pulses = 0;
  pending = 0U;
}

uint8_t Lift_Busy(void)
{
  return (uint8_t)((pending || busy) ? 1U : 0U);
}

LiftStatus Lift_MoveToUm(int32_t height_um)
{
  return queue_pulses(Lift_PulsesFromHeightUm(height_um));
}

LiftStatus Lift_JogUm(int32_t delta_um)
{
  int32_t delta_pulses = Lift_PulsesFromHeightUm(delta_um);
  int64_t wanted_pulses;

  if (delta_pulses == 0 && delta_um != 0)
    delta_pulses = (delta_um > 0) ? 1 : -1;
  wanted_pulses = (int64_t)position_pulses + (int64_t)delta_pulses;
  if (wanted_pulses > 2147483647LL || wanted_pulses < -2147483648LL)
    return LIFT_ERR_RANGE;
  return queue_pulses((int32_t)wanted_pulses);
}

void Lift_RequestStop(void)
{
  stop_request = 1U;
  pending = 0U;
}

void Lift_Cancel(void)
{
  pending = 0U;
}

LiftRunResult Lift_RunPending(void)
{
  int32_t delta_pulses;
  LiftRunResult result;

  if (!pending) return LIFT_RUN_IDLE;
  pending = 0U;
  busy = 1U;
  delta_pulses = target_pulses - position_pulses;
  result = run_move(delta_pulses);
  busy = 0U;
  return result;
}

uint32_t Lift_LastRequestedPulses(void)
{
  return last_requested_pulses;
}

uint32_t Lift_LastIssuedPulses(void)
{
  return last_issued_pulses;
}

const char *Lift_StatusText(LiftStatus status)
{
  switch (status) {
    case LIFT_OK: return "accepted";
    case LIFT_BUSY: return "busy";
    case LIFT_ERR_RANGE: return "outside soft travel window";
    case LIFT_ERR_FAULT: return "DRV8825 nFAULT low";
    default: return "unknown status";
  }
}

const char *Lift_RunResultText(LiftRunResult result)
{
  switch (result) {
    case LIFT_RUN_IDLE: return "idle";
    case LIFT_RUN_COMPLETE: return "complete";
    case LIFT_RUN_STOPPED: return "stopped by request";
    case LIFT_RUN_LIMIT: return "hardware limit";
    case LIFT_RUN_FAULT: return "DRV8825 fault";
    default: return "unknown result";
  }
}
