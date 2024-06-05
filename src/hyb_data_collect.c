/**
 * @file hyb_data_collect.c
 * @author Antoine Henri, Steven Dahdah[]
 * @brief Virtual wall haptic demo using the Quanser Haptics 2 DOF Pantograph.
 * @version 1.0
 * @date 2024-02-27
 * 
 * Quanser's HIL SDK is used to command Quanser's QPID and Quanser's MAPAQ L2,
 * which in turn drive the Quanser Haptics 2 DOF Pantograph.
 * 
 * This application simulates a virtual wall using force feedback on the user's
 * hand that holds the pantograph's end effector. All signals that can be
 * by the QPID (currents, velocities, positions, etc) are coolected and saved
 * in a csv file found in the `data/` folder.
 * 
 * Kinematic equations and pantograph dimensions were derived from the following
 * sources
 * 
 */

#include <stdio.h>
#include <stdbool.h>

#define _USE_MATH_DEFINES
#include <math.h>
#include <time.h>
#include <Windows.h>
#include <direct.h>

#include "exp_params.h"
#include "csv.h"
#include "lti_sys.h"
#include "hil.h"
#include "prbs.h"
#include "quanser_signal.h"
#include "quanser_messages.h"
#include "quanser_thread.h"

/** If defined, will used predefined seeds for the input PRBS signals so that
 * the test is reproducible. If not, will generate random seeds based on time of day.
 */
#define PREDEFINED_SEED


/** Number of samples to collect to compute current sense offsets
 */
#define N_SAMPLES_CURRENT_OFFSET (2000)

/** Number of samples to zero out in inputs.
 */
#define N_PAD (0)

/** Number of turns user must turn the pulley anti-clockwise to start the
 * control loop.
 */
#define NBR_IDLE_TURNS (2)

/** Angle (in rad) the pulley must be turned to to start the
 * control loop.
 */
#define ANGLE_IDLE (NBR_IDLE_TURNS* 2.0 * M_PI)


/** Encoder ratio for all encoders, before capstans (rad/count).
 */
#define K_ENC (-2.0 * M_PI / (4.0*2048))


/** Amplifier gain (A/V).
 */
#define G_A (0.5)

/** Current sense ratio (A/V)
 */
#define G_B (2.0)

/** Torque sensor ratio (Nm/V)
 * See datasheet for the 01324-310 torque sensor by Sensor Developments Inc.
 * and datasheet for the SCM5B38-05 Strain gage Input Module by Dataforth
 */

#define G_T (3.53/5.0)

/** Maximum current for motor (A) (determined by lin amp max current).
 */
#define I_M_MAX (2.4)

/** Maximum current for brake (A).
 */
#define I_B_MAX (0.19)

/** Maximum analog output voltage (V).
 */
#define ANALOG_MAX_V (5)

/** Maximum length of a Quanser message.
 */
#define MESSAGE_LEN (512)

typedef enum {
  HYBRID_OPEN_BOARD,
  HYBRID_HOME,
  HYBRID_CREATE_TASK,
  HYBRID_START_TASK,
  HYBRID_SAVE_CURRENT_OFFSET,
  HYBRID_COMPUTE_CURRENT_OFFSET,
  HYBRID_IDLE,
  HYBRID_RUN,
  HYBRID_STOP_TASK,
  HYBRID_DELETE_TASK,
  HYBRID_CLOSE_BOARD,
  HYBRID_SAVE,
  HYBRID_STOP,
} HybridState;


/** Becomes true if SIGINT is captured.
 */
static bool sigint = false;

/** Signal handler for SIGINT.
 */
void signal_handler(int signal) { sigint = true; }

/** Print error message.
 *
 * @param[in] error Quanser error code.
 */
void print_error(t_error result);

/** Convert encoder counts to radians and apply offset to get phi_1 and 
 * phi_3 angles at axis zero.
 *
 * @param[in]  count Array of two encoder counts.
 * @param[in] offset Array of two encoder offsets (initial positions).
 * @param[out]   rad Array of two encoder position in radians.
 */
void count_to_rad(t_int32 count[2], t_int32 offset[2], t_double rad[2]);

/** Convert encoder count velocities to actuator angular velocities
 *
 * @param[in]  count_vel Array of two encoder counts velocities (count/sec)
 * @param[out]   rad_vel Array of two actuator angular velocities (rad/sec)
 */
void count_to_rad_vel(t_double count_vel[2], t_double rad_vel[2]);


/** Convert linear amplifier current sense voltage to amplifier output current.
 * and convert torque sensor voltage signal ouput to instantaneous torque value
 *
 * @param[in]  volt Array of two current sense voltages, and one torque sensor voltage
 * @param[in] volt_offset Array of three voltage offsets (initial current sense bias).
 * @param[out]   amp Array of two amplifier output currents.
 * @param[out]   torque Array of one torque value.
 */
void volt_sg_to_measurement(t_double volt[3], t_double volt_offset[3], t_double amp[2], t_double torque[1]);


/** Clamp voltage within a range.
 *
 * @param[in]   current Array containing current to clamp.
 * @param[in]   limit Positive saturation limit for voltage.
 * @param[out]  saturated_current Clamped current.
 * @param[out]  saturation_status Indicates if and how current was clamped:
 * 0.0 -> Not clamping voltage;
 * 1.0 -> Clamping voltage to maximum value;
 * -1.0 -> Clamping voltage to minimum value;
 * NAN -> Limit is negative, and unusable, limit value must be changed to positive value.
 */
void clamp(t_double current[2], t_double limit, t_double saturated_current[2], 
           t_double saturation_status[2]);

/** Generate a smoothed pseudorandom binary sequence with zero-padding.
 *
 * `n_pad` can lead to `min_period` being violated once at the beginning of the
 * sequence.
 *
 * @param[in]     cutoff Cutoff frequency (Hz).
 * @param[in]      n_pad Number of timesteps to replace with zeros.
 * @param[in]  min_value Minimum waveform value.
 * @param[in]  max_value Maximum waveform value.
 * @param[in] min_period Minimum high or low period (s)
 * @param[in]     f_samp Sampling frequency (Hz).
 * @param[in]       seed Random seed.
 * @param[in]        len Length of sequence to generate.
 * @param[out]       out Array to store sequence.
 *
 * @retval        PRBS_SUCCESS Successfully generated sequence.
 * @retval PRBS_WARNING_REPEAT Successfully generated sequence, but it repeats.
 */
PrbsStatus smooth_prbs(double cutoff, size_t n_pad, double min_value,
                       double max_value, double min_period, double f_samp,
                       uint16_t seed, size_t len, double *out);

/** Generate a chirp signal.
 *
 * @param[in]     n_pad Number of timesteps of zeros to prepend.
 * @param[in] amplitude Chirp amplitude.
 * @param[in]  min_freq Starting chirp frequency (Hz).
 * @param[in]  max_freq Ending chirp frequency (Hz).
 * @param[in]    f_samp Sampling frequency (Hz).
 * @param[in]       len Length of sequence to generate.
 * @param[out]      out Array to store sequence.
 */
void chirp(size_t n_pad, double amplitude, double min_freq, double max_freq,
           double f_samp, size_t len, double *out);

/** Generate a square signal.
 *
 * @param[in]     n_pad Number of timesteps of zeros to prepend.
 * @param[in] amplitude Square signal amplitude.
 * @param[in]  freq signal frequency (Hz).
 * @param[in]  duty_cycle Portion of square signal period spent at positive 
 *                        amplitude. Between 0 and 1.
 * @param[in]    timestep_samp Control loop timestep (sec)
 * @param[in]       len Length of sequence to generate.
 * @param[out]      out Array to store sequence.
 */
void square(size_t n_pad, double amplitude, double freq, double duty_cycle,
           double timestep_samp, size_t len, double *out);

void sine(size_t n_pad, double amplitude, double freq, 
           double f_samp, size_t len, double *out);


int main(int argc, char *argv[])
{
  // Experiment Parameters
  bool error_occured = false;

  // set highest possible priority for a soft real-time thread
  qsched_param_t scheduling_parameters;
  scheduling_parameters.sched_priority =
      qsched_get_priority_max(QSCHED_FIFO);
  qthread_setschedparam(qthread_self(), QSCHED_FIFO,
                        &scheduling_parameters);

  // Catch Ctrl+C so application may be shut down cleanly
  qsigaction_t action;
  action.sa_handler = signal_handler;
  action.sa_flags = 0;
  qsigemptyset(&action.sa_mask);
  qsigaction(SIGINT, &action, NULL);

  // Board parameters
  static const char board_type[] = "qpid_e";
  static const char board_identifier[] = "0";

  // Board handle
  t_card board;

  // Task handle
  t_task task;

  // Current sample for current sense offset
  t_int32 k_cur_off = 0;

  // Index for state machien state where we wait for brake current steady state
  t_int32 k_pad = 0;

  // Current sample
  t_int32 k = 0;

  // Timestep
  t_double timestep = 1.0 / FREQUENCY;

  // Integral filter for angle controller
  LtiSys integ_theta;
  t_double num_integ_theta[] = {timestep, 0};
  t_double den_integ_theta[] = {-1.0};
  t_double num_buff_integ_theta[2] = {0};
  t_double den_buff_integ_theta[1] = {0};
  ltisys_init(&integ_theta, LTI_SYS_NO_OUTPUT_SAT, num_integ_theta,
              den_integ_theta, num_buff_integ_theta, den_buff_integ_theta, 2,
              1);

  // Derivative filter for angle controller
  LtiSys deriv_theta;
  t_double num_deriv_theta[] = {TAU_THETA / (1 + TAU_THETA / FREQUENCY),
                                -TAU_THETA / (1 + TAU_THETA / FREQUENCY)};
  t_double den_deriv_theta[] = {-1.0 / (1 + TAU_THETA / FREQUENCY)};
  t_double num_buff_deriv_theta[2] = {0};
  t_double den_buff_deriv_theta[1] = {0};
  ltisys_init(&deriv_theta, LTI_SYS_NO_OUTPUT_SAT, num_deriv_theta,
              den_deriv_theta, num_buff_deriv_theta, den_buff_deriv_theta, 2,
              1);

  // Current sense noise filters
  LtiSys noise_filter_0;
  t_double num_noise_filter_0[] = TORQ_NOI_FIL_NUM;
  t_double den_noise_filter_0[] = TORQ_NOI_FIL_DEN;
#define NOISE_FILTER_SIZE_NUM_COEFF ARRAY_LENGTH(num_noise_filter_0)
#define NOISE_FILTER_SIZE_DEN_COEFF ARRAY_LENGTH(den_noise_filter_0)
  t_double num_buff_noise_filter_0[NOISE_FILTER_SIZE_NUM_COEFF] = {0};
  t_double den_buff_noise_filter_0[NOISE_FILTER_SIZE_DEN_COEFF] = {0};
  ltisys_init(&noise_filter_0, LTI_SYS_NO_OUTPUT_SAT, num_noise_filter_0,
              den_noise_filter_0, num_buff_noise_filter_0, den_buff_noise_filter_0, 
              NOISE_FILTER_SIZE_NUM_COEFF, NOISE_FILTER_SIZE_DEN_COEFF);

  // Analog output channels in use
  const t_uint32 analog_out_channels[] = {0, 1};
  // Analog input channels in use
  const t_uint32 analog_in_channels[] = {0, 1, 2};
  // Encoder channels in use
  const t_uint32 encoder_channels[] = {0};
  // Encoder velocity channels in use
  const t_uint32 encoder_vel_channels[] = {14000};


#define NUM_ANALOG_OUT_CHANNELS ARRAY_LENGTH(analog_out_channels)
#define NUM_ANALOG_IN_CHANNELS ARRAY_LENGTH(analog_in_channels)
#define NUM_ENCODER_CHANNELS ARRAY_LENGTH(encoder_channels)

  // Desired amplifier output current signal

//   t_int16 seed_m;
//   t_int16 seed_b;

// #ifdef PREDEFINED_SEED
//   seed_m = PREDEF_SEED_M;
//   seed_b = PREDEF_SEED_B;
// #else
//   const t_int16 lower = 1;
//   const t_int16 upper = 999;
//   srand(time(NULL));
//   seed_m = lower + rand() % (upper - lower + 1);
//   seed_b = lower + rand() % (upper - lower + 1);
// #endif


  t_double target_signal[NUM_ANALOG_OUT_CHANNELS][N_SAMPLES] = {0.0};
  t_double *target_signal_omega = (t_double*)calloc(N_SAMPLES, sizeof(t_double));
  // square(N_PAD, 0.6, 0.1, 0.5, timestep, N_SAMPLES - N_PAD, target_signal[0]);
  // square(N_PAD, -0.05, 0.1, 0.5, timestep, N_SAMPLES - N_PAD, target_signal[1]);

  // smooth_prbs(0.0, N_PAD, INPUT_VOLT_OFFSET_M-INPUT_VOLT_AMP_M, INPUT_VOLT_OFFSET_M + INPUT_VOLT_AMP_M, INPUT_VOLT_MIN_PERIOD_M, FREQUENCY, seed_m, 
  //             N_SAMPLES - N_PAD, target_signal[0]);
  // smooth_prbs(0.0, N_PAD, INPUT_VOLT_OFFSET_B-INPUT_VOLT_AMP_B, INPUT_VOLT_OFFSET_B + INPUT_VOLT_AMP_B, INPUT_VOLT_MIN_PERIOD_B, FREQUENCY, seed_b, 
  //             N_SAMPLES - N_PAD, target_signal[1]);

  // sine(N_PAD, INPUT_CMD_OMEGA_AMP_M, 1.0/INPUT_CMD_THETA_MIN_PERIOD_M, FREQUENCY, N_SAMPLES - N_PAD, target_signal[0]);

  t_double delta_increment = (INPUT_CMD_OMEGA_AMP_M_START - INPUT_CMD_OMEGA_AMP_M_END) / ((t_double)INPUT_CMD_OMEGA_NBR_PERIODS);
  
  int period_len = (N_SAMPLES - N_PAD) / INPUT_CMD_OMEGA_NBR_PERIODS;
  int j = 0;
  for (int i = 1; i <  N_SAMPLES - N_PAD; i++) {
    j = i / period_len;
    j = j > 0 ? j-1 : 0;
    target_signal[0][i] = target_signal[0][i-1] + (INPUT_CMD_OMEGA_AMP_M_START - delta_increment*j)*timestep;
    target_signal_omega[i] = (INPUT_CMD_OMEGA_AMP_M_START - delta_increment*j);
  }

  for (int i = 0; i <  N_SAMPLES - N_PAD; i++) {
    target_signal[1][i] = INPUT_VOLT_AMP_B;
  }


  // accumulated current offsets (A)
  t_double analog_input_offsets[N_SAMPLES][NUM_ANALOG_IN_CHANNELS] = {0};

  // Analog values to output (V)
  t_double analog_outputs[NUM_ANALOG_OUT_CHANNELS]= {0.0};
  // Analog values from linear amplifier current sense (V) and torque sensor
  t_double analog_inputs[NUM_ANALOG_IN_CHANNELS]= {0.0};
  // Encoder counts
  t_int32 encoder_counts[NUM_ENCODER_CHANNELS]= {0};
  // Encoder count velocity 
  t_double encoder_count_vel[NUM_ENCODER_CHANNELS]= {0};

  // Linear amplifier cmd signal (A)
  t_double sat_analog_outputs[NUM_ANALOG_OUT_CHANNELS]= {0.0};
  // Indicates wheter or not a current command was saturated
  t_double sat_status[NUM_ANALOG_OUT_CHANNELS]= {0.0};
  // Linear amplifier current output (A)
  t_double amplifier_output_current[2]= {0.0};
  // Torque sensor reading (N)
  t_double torque[1]= {0.0};
  // Analog values from linear amplifier current sense used to compute current sense offset(V)
  t_double analog_offset[NUM_ANALOG_IN_CHANNELS]= {0.0};
  // Encoder offsets (counts)
  t_int32 encoder_offsets[NUM_ENCODER_CHANNELS] = {0};
  // Encoder counts converted to linkage angle at axis zero in radians (with offset)
  t_double axis_0_radians[NUM_ENCODER_CHANNELS] = {0.0};
  // Angualr velocities for linkages at axis zero in radians/sec
  t_double axis_0_ang_vel[NUM_ENCODER_CHANNELS] = {0.0};

  // Low-pass filtered torque sensor output (Nm)
  t_double filt_torque= 0.0;

  t_double motor_amplifier_input_command = 0.0;
  t_double theta_error =0.0;
  t_double theta_error_integ= 0.0;
  t_double theta_error_deriv= 0.0;

  // Set up CSV
  CsvData csv_data;
  csv_data.header =
      "t, " 
      "torque, theta, omega, theta_cmd, omega_cmd";
  csv_data.n_col = 6;
  csv_data.n_row = N_SAMPLES;
  CsvStatus csv_status  = csv_init(&csv_data);

  if (csv_status != CSV_SUCCESS){
    printf("Failed initialize csv data buffer. \n");
    csv_print_error(csv_status);
    error_occured = true;
  }

  // Current Qube FSM state
  HybridState state = HYBRID_OPEN_BOARD;

  while (true)
  {
    switch (state) {
      case HYBRID_OPEN_BOARD: {
        // Open board
        t_error result = hil_open(board_type, board_identifier, &board);
        // Check result
        if (result == 0) {
          printf("Press CTRL-C to stop program.\n");
          state = HYBRID_HOME;
        } else {
          print_error(result);
          printf("Error raised when opening connection to board: \n");
          state = HYBRID_CLOSE_BOARD;
          error_occured = true;
        }
        break;
      }
      case HYBRID_HOME: {
        // Read encoder to get offsets
        t_error result =
            hil_read_encoder(board, encoder_channels,
                            NUM_ENCODER_CHANNELS, encoder_offsets);
    
        // Check result
        if (result == 0) {
          printf("Homing succesful.\n"
          "Please wind the pulley anti-clockwise %u full turns to start "
          "data collection.\n", NBR_IDLE_TURNS);
          state = HYBRID_CREATE_TASK;
        } else {
          printf("Error raised when homing: \n");
          print_error(result);
          state = HYBRID_CLOSE_BOARD;
          error_occured = true;
        }
        break;
      }
      case HYBRID_CREATE_TASK: {


        t_error result = hil_task_create_reader_writer(
          board, (t_uint32)FREQUENCY, 
          analog_in_channels, NUM_ANALOG_IN_CHANNELS,
          encoder_channels, NUM_ENCODER_CHANNELS, 
          NULL, 0,
          encoder_vel_channels, NUM_ENCODER_CHANNELS, 
          analog_out_channels, NUM_ANALOG_OUT_CHANNELS,
          NULL, 0,
          NULL, 0,
          NULL, 0,
          &task
        );

       
        // Check result
        if (result == 0) {
          state = HYBRID_START_TASK;
        } else {
          printf("Error raised when creating task: \n");
          print_error(result);
          state = HYBRID_DELETE_TASK;
          error_occured = true;
        }
        break;
      }

      case HYBRID_START_TASK: {
        // Start encoder reader task
        t_error result = hil_task_start(task, HARDWARE_CLOCK_0, FREQUENCY, -1);
        // Check result
        if (result == 0) {
          state = HYBRID_SAVE_CURRENT_OFFSET;
        } else {
          printf("Error raised when starting task: \n");
          print_error(result);
          state = HYBRID_STOP_TASK;
          error_occured = true;
        }
        break;
      }
      case HYBRID_SAVE_CURRENT_OFFSET:
      {
        // Check for interrrupt
        if (sigint) {
          state = HYBRID_STOP_TASK;
          break;
        }
        
        // Read DAC inputs

        t_error result = hil_task_read_write(
          task, 1, 
          analog_offset, encoder_counts, 
          NULL, encoder_count_vel,
          sat_analog_outputs, NULL, 
          NULL, NULL
        );

        analog_input_offsets[k_cur_off][0] = analog_offset[0];
        analog_input_offsets[k_cur_off][1] = analog_offset[1];
        analog_input_offsets[k_cur_off][2] = analog_offset[2];

        if (result < 0) {
          printf("Error raised when computing current offset: \n");
          print_error(result);
          state = HYBRID_STOP_TASK;
          error_occured = true;
          break;
        }
        
        k_cur_off++;
        if (k_cur_off >= N_SAMPLES_CURRENT_OFFSET){
          state = HYBRID_COMPUTE_CURRENT_OFFSET;
          break;
        }
        break;
      }
      case HYBRID_COMPUTE_CURRENT_OFFSET:
      {
        // Check for interrrupt
        if (sigint) {
          state = HYBRID_STOP_TASK;
          break;
        }

        for (t_uint32 i= 0; i < N_SAMPLES_CURRENT_OFFSET; i++){
          analog_offset[0] += analog_input_offsets[i][0];
          analog_offset[1] += analog_input_offsets[i][1];
          analog_offset[2] += analog_input_offsets[i][2];
        }

        analog_offset[0] = analog_offset[0] / N_SAMPLES_CURRENT_OFFSET;
        analog_offset[1] = analog_offset[1] / N_SAMPLES_CURRENT_OFFSET;
        analog_offset[2] = analog_offset[2] / N_SAMPLES_CURRENT_OFFSET;
        state = HYBRID_IDLE;
        
        break;
      }
      case HYBRID_IDLE: {
        // Check for interrrupt
        if (sigint) {
          state = HYBRID_STOP_TASK;
          break;
        }
        
        // TODO : uncomment section below once the sensors are tested

        analog_outputs[0] = 0.0;
        analog_outputs[1] = INPUT_VOLT_AMP_B;

        // Saturate current command
        clamp(analog_outputs, ANALOG_MAX_V, sat_analog_outputs, sat_status);


        t_error result = hil_task_read_write(
          task, 1, 
          analog_inputs, encoder_counts, 
          NULL, encoder_count_vel,
          sat_analog_outputs, NULL, 
          NULL, NULL
        );

        if (result < 0) {
          printf("Error raised when read/writing to DAC: \n");
          print_error(result);
          state = HYBRID_STOP_TASK;
          error_occured = true;
          break;
        }


      // fail safe if the current command saturation check doesn't stop overcurrent
      // to be extra safe this would be before the current filtering but it would stop
      // simulations because of current spikes which were really just noise
        if (
          amplifier_output_current[0] >  I_M_MAX || 
          amplifier_output_current[0] <  -I_M_MAX ||
          amplifier_output_current[1] >  I_B_MAX || 
          amplifier_output_current[1] <  -I_B_MAX
        ){
          printf("Current going over max allowed current, stopping control loop for safety. \n");
          state = HYBRID_STOP_TASK;
          error_occured = true;
        }

      
        k_pad++;
        if (k_pad >= N_PAD){
          state = HYBRID_RUN;
          break;
        }
        

        break;
      }


    
      case HYBRID_RUN: {
        // Check for interrrupt
        if (sigint) {
          state = HYBRID_STOP_TASK;
          break;
        }
        
        // TODO : uncomment section below once the sensors are tested

        theta_error =  target_signal[0][k] - axis_0_radians[0];
        theta_error_integ =  ltisys_output(&integ_theta, theta_error);
        theta_error_deriv = ltisys_output(&deriv_theta, theta_error);
        motor_amplifier_input_command = K_M_P * theta_error + K_M_I * theta_error_integ + K_M_D * theta_error_deriv;

        analog_outputs[0] = motor_amplifier_input_command;
        analog_outputs[1] = target_signal[1][k];

        

        // Saturate current command
        clamp(analog_outputs, ANALOG_MAX_V, sat_analog_outputs, sat_status);


        t_error result = hil_task_read_write(
          task, 1, 
          analog_inputs, encoder_counts, 
          NULL, encoder_count_vel,
          sat_analog_outputs, NULL, 
          NULL, NULL
        );

        if (result < 0) {
          printf("Error raised when read/writing to DAC: \n");
          print_error(result);
          state = HYBRID_STOP_TASK;
          error_occured = true;
          break;
        }

        // Convert DAC inputs to usable values
        count_to_rad(encoder_counts, encoder_offsets, axis_0_radians);
        count_to_rad_vel(encoder_count_vel, axis_0_ang_vel);
        volt_sg_to_measurement(analog_inputs, analog_offset, amplifier_output_current, torque);

        filt_torque = ltisys_output(&noise_filter_0, torque[0]);
        
        // Filter DAC inputs


      // fail safe if the current command saturation check doesn't stop overcurrent
      // to be extra safe this would be before the current filtering but it would stop
      // simulations because of current spikes which were really just noise
        if (
          amplifier_output_current[0] >  I_M_MAX || 
          amplifier_output_current[0] <  -I_M_MAX ||
          amplifier_output_current[1] >  I_B_MAX || 
          amplifier_output_current[1] <  -I_B_MAX
        ){
          printf("Current going over max allowed current, stopping control loop for safety. \n");
          state = HYBRID_STOP_TASK;
          error_occured = true;
        }

       

        
        

        // Print radian value to terminal after sending command
        // for (t_uint8 channel = 0; channel < NUM_ENCODER_CHANNELS; channel++)
        // {
        //   printf("E%d: %7.4f  ", encoder_channels[channel], axis_0_radians[channel]);
        // }

        // for (t_uint8 channel = 0; channel < NUM_ANALOG_IN_CHANNELS; channel++)
        // {
        //   printf("E%d: %7.4f  ", analog_in_channels[channel], amplifier_output_current[channel]);
        // }
        // printf("\n");

        

        CsvStatus csv_status = csv_set((double)(timestep * k), k, 0, &csv_data);
        // csv_status |= csv_set((double)sat_analog_outputs[0], k, 1, &csv_data);
        // csv_status |= csv_set((double)sat_analog_outputs[1], k, 2, &csv_data);
        // csv_status |= csv_set((double)amplifier_output_current[0], k, 3, &csv_data);
        // csv_status |= csv_set((double)amplifier_output_current[1], k, 4, &csv_data);
        csv_status |= csv_set((double)torque[0], k, 1, &csv_data);
        csv_status |= csv_set((double)axis_0_radians[0], k, 2, &csv_data);
        csv_status |= csv_set((double)axis_0_ang_vel[0], k, 3, &csv_data);
        csv_status |= csv_set((double)target_signal[0][k], k, 4, &csv_data);
        csv_status |= csv_set((double)target_signal_omega[k], k, 5, &csv_data);
        // csv_status |= csv_set((double)filt_torque, k, 9, &csv_data);
        // HACK: Using bitewise or here so that any of the above return values
        // which is not zero will set the status as non-zero, even if the 
        // next return values are zero
        if (csv_status != CSV_SUCCESS){
          printf("Failed to set csv data to buffer. \n");
          csv_print_error(csv_status);
          error_occured = true;
        }

        k++;
        if (k >= N_SAMPLES){
          state = HYBRID_STOP_TASK;
          break;
        }
        

        break;
      }

      case HYBRID_STOP_TASK: {

        

        // Stop the task
        t_error result = hil_task_stop(task);
        // Check it it was successful
        if (result != 0) {
          printf("Error raised when stopping task: \n");
          print_error(result);
          error_occured = true;
        }

        // Write zero to analog outputs
        for (t_uint8 i = 0; i < NUM_ANALOG_OUT_CHANNELS; i++){
          analog_outputs[i] = 0.0;
        }
        result =
            hil_write_analog(board, analog_out_channels,
                             NUM_ANALOG_OUT_CHANNELS, analog_outputs);

        // Check it it was successful
        if (result != 0) {
          printf("Error raised when stopping motor and brake: \n");
          print_error(result);
          error_occured = true;
        }

        state = HYBRID_DELETE_TASK;
        break;
      }
      case HYBRID_DELETE_TASK: {

        

        // Delete the task
        t_error result = hil_task_delete(task);
        // Check it it was successful
        if (result != 0) {
          printf("Error raised when deleting task: \n");
          print_error(result);
          error_occured = true;
        }
        state = HYBRID_CLOSE_BOARD;
        break;
      }
      case HYBRID_CLOSE_BOARD: {
        


        // Close the board
        t_error result = hil_close(board);
        // Check it it was successful
        if (result != 0) {
          printf("Error raised when closing board: \n");
          print_error(result);
          error_occured = true;
        }
        state = HYBRID_STOP;
        break;
      }
      
      case HYBRID_STOP: {
       

        // Make sure all boards are closed
        t_error result = hil_close_all();
        // Check it it was successful
        if (result != 0) {
          printf("Error raised when closing all HIL ressources: \n");
          print_error(result);
          error_occured = true;
        }
        state = HYBRID_SAVE;
        
      }

      case HYBRID_SAVE: {
        
        
        // Get timestamp
        time_t timer;
        char filename_buffer[MESSAGE_LEN];

        struct tm *tm_info;
        timer = time(NULL);
        tm_info = localtime(&timer);
        _mkdir("..\\..\\..\\data");
        strftime(filename_buffer, sizeof(filename_buffer), "..\\..\\..\\data\\hybrid_%Y%m%dT%H%M%S.csv",
                 tm_info);


        printf("Saving control loop data at %s .\n", filename_buffer);
        CsvData * ptr_csv_data = &csv_data;
        CsvStatus csv_status = csv_save(filename_buffer, ptr_csv_data);
        if (csv_status != CSV_SUCCESS){
          printf("Failed to save csv data. \n");
          csv_print_error(csv_status);
          error_occured = true;
        }
        csv_free(&csv_data);
        if (csv_status != CSV_SUCCESS){
          printf("Failed to free csv data. \n");
          csv_print_error(csv_status);
          error_occured = true;
        }

        free(target_signal_omega);

        if (error_occured){
          Sleep(10000);
        }

        printf("Succesfully saved control loop data.\n");
        
        return 0;
      }
    }
  }
}

void print_error(t_error result) {
  char message[MESSAGE_LEN];
  msg_get_error_message(NULL, result, message, ARRAY_LENGTH(message));
  printf("ERROR %d: %s\n", -result, message);
}

void count_to_rad(t_int32 count[2], t_int32 offset[2], t_double rad[2]) {
    rad[0] = (count[0] - offset[0]) * K_ENC;
}

void count_to_rad_vel(t_double count_vel[2], t_double rad_vel[2]) {
    rad_vel[0] = count_vel[0] * K_ENC;
}

void volt_sg_to_measurement(t_double volt[3], t_double volt_offset[3], t_double amp[2], t_double torque[1]) {
    amp[0] = (volt[0] - volt_offset[0]) * G_B;
    amp[1] = (volt[1] - volt_offset[1]) * G_B;
    torque[0] = (volt[2] - volt_offset[2]) * G_T;
}

void clamp(t_double current[2], t_double limit, t_double saturated_current[2], t_double saturation_status[2]) {
  // Make sure limit is positive
  if (limit < 0) {
    for(int i = 0; i < 2; i++){
      saturated_current[i] = 0.0;
      saturation_status[i] = NAN;
    }
  }
  // Saturate
  for(int i = 0; i < 2; i++){
    saturated_current[i] = 0.0;
    saturation_status[i] = 0.0;
    if (current[i] > limit) {
      saturated_current[i] = limit;
      saturation_status[i] = 1.0;
    } else if (current[i] < -limit) {
      saturated_current[i] = -limit;
      saturation_status[i] = -1.0;
    } else {
      saturated_current[i] = current[i];
      saturation_status[i] = 0.0;
    }
  }
}

PrbsStatus smooth_prbs(double cutoff, size_t n_pad, double min_value,
                       double max_value, double min_period, double f_samp,
                       uint16_t seed, size_t len, double *out) {
  // Generate PRBS
  t_double *raw_prbs = (t_double *)calloc(len, sizeof(t_double));
  PrbsStatus status =
      prbs(min_value, max_value, min_period, f_samp, seed, len, raw_prbs);
  for (int i = 0; i < n_pad; i++) {
    raw_prbs[i] = 0.0;
  }
  if (cutoff <= 0.0) {
    // Copy
    for (int i = 0; i < len; i++) {
      out[i] = raw_prbs[i];
    }
  } else {
    // Create filter
    LtiSys filt;
    t_double num_filt[] = {(cutoff / f_samp) / (1.0 + cutoff / f_samp), 0};
    t_double den_filt[] = {-1.0 / (1.0 + cutoff / f_samp)};
    t_double num_buff_filt[2] = {0};
    t_double den_buff_filt[1] = {0};
    ltisys_init(&filt, LTI_SYS_NO_OUTPUT_SAT, num_filt, den_filt, num_buff_filt,
                den_buff_filt, 2, 1);
    // Run filter
    for (int i = 0; i < len; i++) {
      out[i] = ltisys_output(&filt, raw_prbs[i]);
    }
  }
  // Clean up
  free(raw_prbs);
  return status;
}

void chirp(size_t n_pad, double amplitude, double min_freq, double max_freq,
           double f_samp, size_t len, double *out) {
  // Chirp rate
  double c = (max_freq - min_freq) / ((len - n_pad) / f_samp);
  // Fill in output array
  for (int i = 0; i < len; i++) {
    if (i < n_pad) {
      out[i] = 0.0;
    } else {
      double phi = 2.0 * M_PI * c * ((i - n_pad) / f_samp) + min_freq;
      out[i] = amplitude * sin(phi * ((i - n_pad) / f_samp));
    }
  }
}

void square(size_t n_pad, double amplitude, double freq, double duty_cycle,
           double timestep_samp, size_t len, double *out){

  double period = 1.0/freq;
  double up_period = duty_cycle * period;

  int period_cycles =(int) (period / timestep_samp);
  int up_period_cycles =(int) (up_period / timestep_samp);
  
  int i_on = 0;

  for (int i = 0; i < len; i++) {
    if (i < n_pad) {
      out[i] = 0.0;
    } else {
      i_on = i - n_pad;
      i_on = i_on % period_cycles;
      out[i] = (i_on <= up_period_cycles) ? amplitude : -1.0 * amplitude;
      
    }
  }
}

void sine(size_t n_pad, double amplitude, double freq, 
           double f_samp, size_t len, double *out) {
  // Chirp rate
  double c = 2.0 * M_PI* freq / f_samp;
  // Fill in output array
  for (int i = 0; i < len; i++) {
    if (i < n_pad) {
      out[i] = 0.0;
    } else {
      
      out[i] = amplitude * sin(c* (i - n_pad));
    }
  }
}


