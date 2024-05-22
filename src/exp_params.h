#ifndef EXP_PARAMS_H_
#define EXP_PARAMS_H_

// EXPERIMENT PARAMETERS

/** Control frequency (Hz).
 */
#define FREQUENCY (800.0)

/** Number of samples to collect.
 */
#define N_SAMPLES (12000)

// #define PREDEF_SEED_M 4

#define INPUT_CMD_OMEGA_AMP_M 0

// #define INPUT_VOLT_OFFSET_M 0.1

#define INPUT_CMD_OMEGA_MIN_PERIOD_M 10

// #define PREDEF_SEED_B 5

#define INPUT_VOLT_AMP_B 0.18

// #define INPUT_VOLT_OFFSET_B 0.07

// #define INPUT_VOLT_MIN_PERIOD_B 0.05


/**
 * MOTOR Vel CONTROLLER
 */

/** Motor vel controller proportionnal gain
 */

#define K_M_P 0.4

/** Motor vel controller integral gain
 */

#define K_M_I 0.03

#endif  // EXP_PARAMS_H_