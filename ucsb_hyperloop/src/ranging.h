#ifndef RANGING_H_
#define RANGING_H_

#include "board.h"

// New Short-Ranging Code
// Distances of down-ward facing short ranging sensors from the pod's center.
#define d_F		125.0	// cm
#define d_R		24.0
#define d_B		125.0
#define d_L		24.0

// Distances of I-beam facing short ranging sensors from the pod's center.
#define I_BEAM_RANGNG_FRONT	104.0		// TODO
#define I_BEAM_RANGNG_BACK	96.0		// TODO

#define PI_CONSTANT		3.14159262

float pitch_i;
float roll_i;
float z_ci;

float yaw_i;
float y_ci;

// Old Short-Ranging Code (DELETE LATER!)
#define SHORT_FRONT_DIST		8.5f
#define SHORT_BACK_DIST			8.5f
#define SHORT_FRONT_HEIGHT		5.4f
#define SHORT_BACK_HEIGHT		5.4f
#define SHORT_FRONT_LEFT_DIST	5.1f
#define SHORT_FRONT_RIGHT_DIST	5.1f
#define SHORT_BACK_LEFT_DIST	5.1f
#define SHORT_BACK_RIGHT_DIST	5.1f
#define SHORT_RIGHT_DIST_AVG	((SHORT_BACK_RIGHT_DIST*SHORT_BACK_DIST + SHORT_FRONT_RIGHT_DIST*SHORT_FRONT_DIST))/(SHORT_BACK_DIST + SHORT_FRONT_DIST)
//#define SHORT_RIGHT_AVG_INV		(1.0 / SHORT_RIGHT_DIST_AVG)
//#define SHORT_FRONT_DIST_INV	(1.0 / SHORT_FRONT_DIST)
//#define SHORT_BACK_DIST_INV		(1.0 / SHORT_BACK_DIST)
//#define SHORT_AXIS_SUM_INV		(1.0 / (SHORT_FRONT_DIST + SHORT_BACK_DIST))

#define LONG_FRONT_DIST			12.45
#define LONG_BACK_DIST			12.45
#define LONG_FRONT_LEFT_DIST	4.3
#define LONG_FRONT_RIGHT_DIST	4.3
#define LONG_BACK_LEFT_DIST		4.3
#define LONG_BACK_RIGHT_DIST	4.3
#define LONG_FRONT_DIST_INV		(1.0 / LONG_FRONT_DIST)
#define LONG_BACK_DIST_INV		(1.0 / LONG_BACK_DIST)
#define LONG_AXIS_SUM_INV		(1.0 / (LONG_FRONT_DIST + LONG_BACK_DIST))

#define _LPC_ADC_ID 		LPC_ADC
#define _LPC_ADC_IRQ 		ADC_IRQn
#define LONG_FRONT_INITIAL	45.0	// cm
#define LONG_BACK_INITIAL	45.0	// cm
#define SHORT_FRONT_INITIAL	2.5		// cm
#define SHORT_BACK_INITIAL	2.5		// cm
#define ALPHA 				0
#define BETA				(1 - ALPHA)

float short_front_right_pyth;
float short_back_right_pyth;
float short_right_pyth_inv;
float short_right_avg_inv;
float short_front_dist_inv;
float short_back_dist_inv;
float short_axis_sum_inv;

volatile uint8_t Burst_Mode_Flag, Interrupt_Continue_Flag;
volatile uint8_t channelTC, dmaChannelNum;
volatile uint8_t ADC_Interrupt_Done_Flag;
float ShortRangingMovingAverage[4];
float LongRangingMovingAverage[4];
uint32_t DMAbuffer;
uint16_t ShortRangingDataRaw[4];
uint16_t LongRangingDataRaw[4];
ADC_CLOCK_SETUP_T ADCSetup;

typedef struct{

  float frontLeft;
  float frontRight;
  float backLeft;
  float backRight;

} rangingData;

typedef struct{

	float y;
	float z;
	float roll;
	float pitch;
	float yaw;

} positionAttitudeData;

static const float arcSinLUT[] =
{	-90.00, -82.82, -79.84, -77.55, -75.61, -73.90, -72.35, -70.93, -69.60, -68.34,
	-67.16, -66.03, -64.94, -63.90, -62.90, -61.93, -60.99, -60.07, -59.18, -58.32,
	-57.47, -56.65, -55.84, -55.05, -54.27, -53.51, -52.76, -52.02, -51.30, -50.58,
	-49.88, -49.19, -48.51, -47.83, -47.17, -46.51, -45.86, -45.22, -44.58, -43.96,
	-43.34, -42.72, -42.11, -41.51, -40.91, -40.32, -39.73, -39.15, -38.57, -38.00,
	-37.43, -36.87, -36.31, -35.75, -35.20, -34.65, -34.11, -33.57, -33.03, -32.50,
	-31.97, -31.44, -30.91, -30.39, -29.87, -29.35, -28.84, -28.33, -27.82, -27.31,
	-26.81, -26.30, -25.80, -25.31, -24.81, -24.32, -23.82, -23.33, -22.84, -22.36,
	-21.87, -21.39, -20.91, -20.43, -19.95, -19.47, -19.00, -18.52, -18.05, -17.58,
	-17.10, -16.64, -16.17, -15.70, -15.23, -14.77, -14.30, -13.84, -13.38, -12.92,
	-12.46, -12.00, -11.54, -11.08, -10.62, -10.16, -9.71, -9.25, -8.80, -8.34,
	-7.89, -7.44, -6.98, -6.53, -6.08, -5.63, -5.17, -4.72, -4.27, -3.82,
	-3.37, -2.92, -2.47, -2.02, -1.57, -1.12, -0.67, 0.00, 0.00, 0.67,
	1.12, 1.57, 2.02, 2.47, 2.92, 3.37, 3.82, 4.27, 4.72, 5.17,
	5.63, 6.08, 6.53, 6.98, 7.44, 7.89, 8.34, 8.80, 9.25, 9.71,
	10.16, 10.62, 11.08, 11.54, 12.00, 12.46, 12.92, 13.38, 13.84, 14.30,
	14.77, 15.23, 15.70, 16.17, 16.64, 17.10, 17.58, 18.05, 18.52, 19.00,
	19.47, 19.95, 20.43, 20.91, 21.39, 21.87, 22.36, 22.84, 23.33, 23.82,
	24.32, 24.81, 25.31, 25.80, 26.30, 26.81, 27.31, 27.82, 28.33, 28.84,
	29.35, 29.87, 30.39, 30.91, 31.44, 31.97, 32.50, 33.03, 33.57, 34.11,
	34.65, 35.20, 35.75, 36.31, 36.87, 37.43, 38.00, 38.57, 39.15, 39.73,
	40.32, 40.91, 41.51, 42.11, 42.72, 43.34, 43.96, 44.58, 45.22, 45.86,
	46.51, 47.17, 47.83, 48.51, 49.19, 49.88, 50.58, 51.30, 52.02, 52.76,
	53.51, 54.27, 55.05, 55.84, 56.65, 57.47, 58.32, 59.18, 60.07, 60.99,
	61.93, 62.90, 63.90, 64.94, 66.03, 67.16, 68.34, 69.60, 70.93, 72.35,
	73.90, 75.61, 77.55, 79.84, 82.82, 90.00 };

/* Starts at 0.34V, goes to 2.43V, increments in intervals of 0.01V */
/*static const float shortRangingDistanceLUT[] =
{    17.664, 17.139, 16.641, 16.169, 15.719, 15.292, 14.885, 14.499, 14.130, 13.780,
     13.445, 13.126, 12.822, 12.532, 12.254, 11.989, 11.735, 11.492, 11.260, 11.037,
     10.823, 10.618, 10.421, 10.232, 10.050, 9.876, 9.707, 9.545, 9.389, 9.238,
     9.092, 8.951, 8.815, 8.684, 8.557, 8.433, 8.314, 8.198, 8.085, 7.976,
     7.869, 7.766, 7.665, 7.567, 7.472, 7.379, 7.288, 7.200, 7.113, 7.029,
     6.946, 6.865, 6.786, 6.709, 6.633, 6.559, 6.486, 6.415, 6.344, 6.276,
     6.208, 6.142, 6.076, 6.012, 5.949, 5.887, 5.826, 5.766, 5.707, 5.648,
     5.591, 5.534, 5.478, 5.423, 5.369, 5.316, 5.263, 5.211, 5.159, 5.109,
     5.058, 5.009, 4.960, 4.912, 4.864, 4.817, 4.770, 4.724, 4.679, 4.634,
     4.589, 4.546, 4.502, 4.459, 4.417, 4.375, 4.333, 4.292, 4.251, 4.211,
     4.171, 4.131, 4.092, 4.054, 4.016, 3.978, 3.940, 3.903, 3.867, 3.830,
     3.794, 3.759, 3.723, 3.689, 3.654, 3.620, 3.586, 3.552, 3.519, 3.486,
     3.454, 3.421, 3.389, 3.358, 3.326, 3.295, 3.265, 3.234, 3.204, 3.174,
     3.145, 3.115, 3.086, 3.058, 3.029, 3.001, 2.973, 2.945, 2.918, 2.891,
     2.864, 2.837, 2.811, 2.785, 2.759, 2.733, 2.708, 2.683, 2.658, 2.633,
     2.609, 2.584, 2.560, 2.537, 2.513, 2.490, 2.467, 2.444, 2.421, 2.398,
     2.376, 2.354, 2.332, 2.311, 2.289, 2.268, 2.247, 2.226, 2.205, 2.185,
     2.165, 2.144, 2.125, 2.105, 2.085, 2.066, 2.047, 2.028, 2.009, 1.990,
     1.972, 1.954, 1.935, 1.917, 1.900, 1.882, 1.865, 1.847, 1.830, 1.813,
     1.796, 1.780, 1.763, 1.747, 1.731, 1.715, 1.699, 1.683, 1.667, 1.652,
     1.636, 1.621, 1.606, 1.591, 1.577, 1.562, 1.547, 1.533, 1.519, 1.505 };*/

//Previous LUT ended at 2.43V and had .01V increments
/* Starts at 0.34V, goes to 2.44V, increments in intervals of 0.05V for SR0 */
static const float shortRangingDistanceLUT0[] =
{	28.0, 24.0, 19.9, 16.7, 14.4, 13.6, 13.0, 11.9, 10.8, 9.6, 8.8, 8.0, 7.6,
	6.8, 6.4, 5.9, 5.5, 5.1, 4.9, 4.75, 4.6, 4.4, 4.1, 3.9, 3.6, 3.4, 3.2, 3.1,
	3.0, 2.8, 2.6, 2.5, 2.35, 2.3, 2.2, 2.1, 2.05, 2.0, 1.95, 1.9, 1.8, 1.65, 1.5 };

/* Starts at 0.34V, goes to 2.44V, increments in intervals of 0.05V for SR1 */
static const float shortRangingDistanceLUT1[] =
{	40.0, 40.0, 40.0, 40.0, 40.0, 13.3, 12.8, 10.4, 10.0, 8.95, 8.4, 7.8, 6.9, 6.5,
	5.85, 5.5, 5.3, 4.85, 4.5, 4.4, 4.15, 3.9, 3.75, 3.5, 3.4, 3.3, 3.25, 2.95, 2.6,
	2.55, 2.5, 2.35, 2.15, 2.1, 2.05, 2.0, 1.95, 1.9, 1.85, 1.8, 1.7, 1.65, 1.55 };

/* Starts at 0.34V, goes to 2.44V, increments in intervals of 0.05V for SR2 */
static const float shortRangingDistanceLUT2[] =
{	28.0, 25.6, 19.5, 16.6, 14.2, 13.5, 13.0, 11.8, 9.9, 9.2, 8.5, 7.8, 7.2, 6.4,
	6.2, 5.8, 5.5, 5.2, 4.9, 4.6, 4.3, 4.1, 4.0, 3.7, 3.55, 3.3, 3.1, 2.9, 2.75,
	2.65, 2.55, 2.5, 2.45, 2.35, 2.3, 2.2, 2.1, 2.05, 1.95, 1.9, 1.8, 1.7, 1.6 };

/* Starts at 0.34V, goes to 2.44V, increments in intervals of 0.05V for SR3 */
static const float shortRangingDistanceLUT3[] =
{	24.0, 22.3, 18.5, 14.6, 13.8, 13.4, 12.0, 10.9, 9.8, 8.9, 7.9, 7.0, 6.7, 6.1,
	5.8, 5.6, 5.4, 5.1, 4.4, 4.0, 3.8, 3.65, 3.4, 3.3, 3.2, 3.0, 2.95, 2.9, 2.6,
	2.5, 2.4, 2.35, 2.3, 2.25, 2.2, 2.1, 2.05, 2.0, 1.9, 1.85, 1.8, 1.775, 1.75 };

/* Starts at 0.49V, goes to 2.73V, increments in intervals of 0.01V. */
static const float longRangingDistanceLUT[] =
{    136.944, 134.068, 131.290, 128.606, 126.013, 123.507, 121.085, 118.743, 116.479, 114.289,
     112.171, 110.121, 108.138, 106.218, 104.359, 102.559, 100.815, 99.126, 97.489, 95.903,
     94.364, 92.872, 91.425, 90.021, 88.659, 87.336, 86.051, 84.804, 83.592, 82.414,
     81.269, 80.156, 79.073, 78.020, 76.995, 75.997, 75.026, 74.080, 73.158, 72.260,
     71.384, 70.530, 69.697, 68.884, 68.091, 67.317, 66.561, 65.822, 65.101, 64.395,
     63.706, 63.031, 62.371, 61.726, 61.094, 60.475, 59.869, 59.275, 58.694, 58.123,
     57.564, 57.016, 56.478, 55.950, 55.432, 54.923, 54.423, 53.932, 53.450, 52.976,
     52.510, 52.051, 51.601, 51.157, 50.721, 50.291, 49.868, 49.452, 49.042, 48.638,
     48.239, 47.847, 47.460, 47.079, 46.703, 46.331, 45.965, 45.604, 45.248, 44.896,
     44.548, 44.205, 43.866, 43.532, 43.201, 42.874, 42.552, 42.232, 41.917, 41.605,
     41.297, 40.992, 40.690, 40.391, 40.096, 39.804, 39.515, 39.229, 38.946, 38.665,
     38.388, 38.113, 37.841, 37.571, 37.304, 37.040, 36.778, 36.519, 36.262, 36.007,
     35.754, 35.504, 35.256, 35.011, 34.767, 34.526, 34.286, 34.049, 33.814, 33.581,
     33.349, 33.120, 32.892, 32.667, 32.443, 32.221, 32.001, 31.782, 31.566, 31.351,
     31.137, 30.926, 30.716, 30.507, 30.301, 30.096, 29.892, 29.690, 29.490, 29.291,
     29.093, 28.897, 28.703, 28.510, 28.318, 28.128, 27.939, 27.751, 27.565, 27.381,
     27.197, 27.015, 26.834, 26.655, 26.477, 26.300, 26.124, 25.950, 25.777, 25.605,
     25.434, 25.264, 25.096, 24.929, 24.763, 24.598, 24.435, 24.272, 24.111, 23.951,
     23.791, 23.633, 23.476, 23.320, 23.166, 23.012, 22.859, 22.708, 22.557, 22.407,
     22.259, 22.111, 21.965, 21.819, 21.675, 21.531, 21.389, 21.247, 21.106, 20.967,
     20.828, 20.690, 20.553, 20.417, 20.282, 20.148, 20.015, 19.882, 19.751, 19.620,
     19.491, 19.362, 19.234, 19.107, 18.981, 18.855, 18.731, 18.607, 18.484, 18.362,
     18.241, 18.120, 18.000, 17.882, 17.764 };


float arcsin(float x);
positionAttitudeData computePositionAttitudeRanging();
rangingData getLongDistance();
void convertVoltage(uint16_t data, uint8_t sensor);
rangingData getShortDistance();
void ADC_IRQHandler();
void rangingSensorsInit();
void rangingSensorsCalibrate();
void convertVoltageShort(uint8_t sensor);
void convertVoltageLong(uint8_t sensor);
void Ranging_Int_Measure();
void initADCChannel(uint8_t channel, uint8_t port, uint8_t pin, uint8_t func, float init_val);

#endif
