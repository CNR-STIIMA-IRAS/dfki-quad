#pragma once

#include "common/constants.hpp"

static constexpr int FEET_POSITION_SEQUENCE_SIZE = int((MPC_DT / MPC_CONTROL_DT) * GAIT_SEQUENCE_SIZE);

// message publishers
static const bool PUBLISH_SWING_LEG_TRAJECTORIES = false;
static const bool PUBLISH_GAIT_STATE = true;
static const bool PUBLISH_OPEN_LOOP_TRAJECTORY = true;
static const bool PUBLISH_SOLVE_TIME = true;
static const bool PUBLISH_WBC_SOLVE_TIME = true;
static const bool PUBLISH_WBC_TARGET = true;
static const bool PUBLISH_GAIT_SEQUENCE = true;
static const bool PUBLISH_HEARTBEAT = true;

#ifdef ROBOT_MODEL
#if ROBOT_MODEL == GO2
static constexpr bool USE_WBC = true;
#elif ROBOT_MODEL == ULAB
static constexpr bool USE_WBC = false;
#endif
#endif

// #define DEBUG_PRINTS
