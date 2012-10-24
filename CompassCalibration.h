#ifndef __COMPASS_CALIBRATION_H__
#define __COMPASS_CALIBRATION_H__

/* Compass Calibration Transform Structure
 * The calibration progress should be applied
 * outside the algorithm.
 *
 * Ex:
 * X = (RawX - (minx + maxx) / 2) * matrix[0][0]
 * Y = (RawY - (miny + maxy) / 2) * matrix[1][1]
 * Z = (RawZ - (minz + maxz) / 2) * matrix[2][2]
 */
typedef struct {
    /* Magnetic Soft Iron matrix */
    float matrix[3][3];

    /* Magnetic Hard Iron positions */
    float minx;
    float maxx;
    float miny;
    float maxy;
    float minz;
    float maxz;
} CompassCalData;

/* CompassCal_init
 * Must be called every time stating calibration
 * algorithm.
 *
 * If calied = 1, compass calibration algorithm
 * will be initialized with stored hard/soft
 * iron data, and do some futher calibration. In
 * this condition CompassCal_readyCheck will always
 * return true.
 * If calied = 0, algorithm will start new calibration
 * progress and CompassCal_readyCheck won't return ture
 * if not enough samples are collected.
 *
 * If calied = 1, data points to transform structure,
 * else points to NULL.
 */
void CompassCal_init(int calied, CompassCalData* data);

/* CompassCal_collectData
 * Called to pass un-calibrated data from sensor to
 * calibration algorithm.
 *
 * The input rawdata(rawMagX, rawMagY, rawMaxZ)
 * must be past in unit of micro-Tesla
 * CurrentTimMSec stands for current time in ms.
 *
 * Return 0 if raw data is rejected from calibration
 * and return 1 if the data is accepted.
 */
int CompassCal_collectData(float rawMagX, float rawMagY, float rawMagZ, long currentTimeMSec);

/* CompassCal_readyCheck
 * Check if enough raw data has been collected
 * to generate a calibration resualt.
 *
 * Return 1 if enough raw data has been
 * otherwise return 0.
 * This function will always return 1
 * with algorithm is init by calied = 1.
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
