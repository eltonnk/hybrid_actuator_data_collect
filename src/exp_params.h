#ifndef EXP_PARAMS_H_
#define EXP_PARAMS_H_

// EXPERIMENT PARAMETERS

/** Control frequency (Hz).
 */
#define FREQUENCY (800.0)

/** Number of samples to collect.
 */
#define N_SAMPLES (500000)

// #define PREDEF_SEED_M 4

#define INPUT_CMD_OMEGA_AMP_M_START 10

#define INPUT_CMD_OMEGA_AMP_M_POST_START 1.0

#define INPUT_CMD_OMEGA_AMP_M_STEP 0.1

// #define INPUT_VOLT_OFFSET_M 0.1

#define INPUT_CMD_THETA_MIN_PERIOD_M 15

// #define PREDEF_SEED_B 5

#define INPUT_VOLT_AMP_B 0.00

// #define INPUT_VOLT_OFFSET_B 0.07

// #define INPUT_VOLT_MIN_PERIOD_B 0.05


/**
 * MOTOR Vel CONTROLLER
 */

/** Motor vel controller proportionnal gain
 */

#define K_M_P 0.95

/** Motor vel controller integral gain
 */

#define K_M_I 0.15


/** Derivative time constant for motor angle.
 */
#define TAU_THETA (50.0)


/** Motor vel controller deriv gain
 */

#define K_M_D 0.9

#endif  // EXP_PARAMS_H_


/**
 * CURRENT SENSE NOISE FILTER
 * 6th order, low pass, bessel filter  and cutoff frequency at 100 Hz
 */

/** Current sense noise filter numerator coefficients
 */
#define TORQ_NOI_FIL_NUM {0.0008791 , 0.00527463, 0.01318656, 0.01758209, 0.01318656,0.00527463, 0.0008791 }

/** Current sense noise filter denominator coefficients
 */
#define TORQ_NOI_FIL_DEN {-2.76883538,  3.4972914 , -2.51747179,  1.0744572 ,-0.255424  ,  0.02624525}