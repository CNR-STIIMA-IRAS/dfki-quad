#ifndef COMMON__COMMON_CONSTANTS_HPP_
#define COMMON__COMMON_CONSTANTS_HPP_

static constexpr int N_LEGS = 4;
static constexpr int N_JOINTS_PER_LEG = 3;
static constexpr int GAIT_SEQUENCE_SIZE = 100;
static constexpr int MPC_PREDICTION_HORIZON = 10;
static constexpr int STATE_SIZE = 13;
static constexpr double MPC_DT = 0.05;
static constexpr double MPC_CONTROL_DT = 0.01;
static constexpr double SWING_LEG_DT = 0.002;
static constexpr double CONTROL_DT = 0.002;
static constexpr double WBC_CYCLE_DT = CONTROL_DT;
static constexpr double MODEL_ADAPTATION_DT = 0.01;
static constexpr double MODEL_ADAPTATION_BATCH_SIZE = 100;

#endif // COMMON__COMMON_CONSTANTS_HPP_