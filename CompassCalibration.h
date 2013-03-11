#ifndef __COMPASS_CALIBRATION_H__
#define __COMPASS_CALIBRATION_H__

/* Compass Calibration Transform Structure
 * The calibration progress should be applied
 * outside the algorithm.
 *
 * Ex:
 * x = (RawX - off_x) * w11
 * y = (RawY - off_y) * w22
 * z = (RawZ - off_z) * w33
 */

typedef struct {
    /* hard iron offsets */
    double off_x;
    double off_y;
    double off_z;

    /* soft iron matrix members */
    double w11;
    double w22;
    double w33;

    /* geomagnetic strength */
    double bfield;
} CompassCalData;

/* CompassCal_init
 * Initialize calibration algorithm. Must be called at once.
 *
 * If compass has been calibrated before, old calibration data
 * could be passed into algorithm.
 */
void CompassCal_init(int caled, const CompassCalData &data);

/* CompassCal_collectData
 * collect compass data to calibrate
 *
 * The input rawdata(rawMagX, rawMagY, rawMaxZ)
 * must be past in unit of micro-Tesla
 * CurrentTimMSec stands for current time in ms.
 *
 * Return 0 if raw data is rejected,
 * return 1 if the data is accepted.
 */
int CompassCal_collectData(float rawMagX, float rawMagY, float rawMagZ, long currentTimeMSec);

/* CompassCal_readyCheck
 * Check if enough raw data has been collected
 * to generate a calibration result.
 *
 * Return 1 if enough raw data has been
 * otherwise return 0.
 */
int CompassCal_readyCheck();

/* CompassCal_computeCal
 * Compute transform structure and store
 * the result in data.
 *
 * This function should only be called
 * after calling CompassCal_readyCheck and
 * the return value is 1.
 */
void CompassCal_computeCal(CompassCalData* data);

#endif /*__COMPASS_CALIBRATION_H__*/
