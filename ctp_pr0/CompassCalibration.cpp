#include <cutils/log.h>
#include <math.h>

#include "CompassCalibration.h"

/* Earth magnetic parameters */
#define MAG_MAX 80.0f /* 80 mt*/
#define MAG_MIN 25.0f /* 25 mt*/
#define MAG_MAX_INC 30.0f /* 30 mt */

/* Geometry parameters */
#define LINE_SEGS 8

/* Used to collect enough data from sensor */
static int line_segs[3][LINE_SEGS];
static int fit_line;

/* Used to reject sudden jump data */
static struct prev_point {
    float p[3];
    long time;
} prev_p;

static int calibrated;

static CompassCalData calData;

static inline float cal_datax(int rawMagX)
{
    return (rawMagX - (calData.minx + calData.maxx) / 2) * calData.matrix[0][0];
}

static inline float cal_datay(int rawMagY)
{
    return (rawMagY - (calData.miny + calData.maxy) / 2) * calData.matrix[1][1];
}

static inline float cal_dataz(int rawMagZ)
{
    return (rawMagZ - (calData.minz + calData.maxz) / 2) * calData.matrix[2][2];
}

/* Filter abnoral data from calibration */
static int abnormal_data(float rawMagX, float rawMagY, float rawMagZ, long currentTimeMSec)
{
    int fast_change = 0;
    /* ignore fast change data */
    if (prev_p.time != 0 && currentTimeMSec - prev_p.time <= 20) {
        if (fabs(rawMagX - prev_p.p[0]) > MAG_MAX_INC ||
            fabs(rawMagY - prev_p.p[1]) > MAG_MAX_INC ||
            fabs(rawMagZ - prev_p.p[2]) > MAG_MAX_INC)
            fast_change = 1;
    }
    prev_p.time = currentTimeMSec;
    prev_p.p[0] = rawMagX;
    prev_p.p[1] = rawMagY;
    prev_p.p[2] = rawMagZ;
    if (fast_change == 1)
        return 1;

    /* Any mag field larger than MAG_MAX is an abnormal data
     * With one exception, if not calibrated yet, accept big data.
     * Because without hard iron calibration the normal data
     * also can be big.
     */
    if (calibrated == 0)
        return 0;
    if (pow(cal_datax(rawMagX), 2) +
        pow(cal_datay(rawMagY), 2) +
        pow(cal_dataz(rawMagZ), 2) >= pow(MAG_MAX, 2))
        return 1;
    return 0;
}

/* Find enough data to fit x, y and z axes
 * return 1 for fit, 0 for not
 */
static int fit_line_segs(float x, float y, float z)
{
    /* one point only fit one seg on one axe, this
     * make more data has to be collected to finish
     * calibration, thus obtain better calibration
     * result.
     */
    if (LINE_SEGS < 2)
        return 0; /* avoid div0 exception */
    float off = x - calData.minx;
    float per = off / (calData.maxx - calData.minx);
    int seg = (int) (per / (1.0f / LINE_SEGS));
    /* val is between min and max */
    if (per != 1 && per != 0) {
        if (line_segs[0][seg] == 0) {
            line_segs[0][seg] = 1;
            LOGD("CompassCalibration:fit_line,x[%d]=[%f,%f,%f]", seg, x, y, z);
            return 1;
        }
    }

    off = y - calData.miny;
    per = off / (calData.maxy - calData.miny);
    seg = (int) (per / (1.0f / LINE_SEGS));
    /* val is between min and max */
    if (per != 1 && per != 0) {
        if (line_segs[1][seg] == 0) {
            line_segs[1][seg] = 1;
            LOGD("CompassCalibration:fit_line,y[%d]=[%f,%f,%f]", seg, x, y, z);
            return 1;
        }
    }

    off = z - calData.minz;
    per = off / (calData.maxz - calData.minz);
    seg = (int) (per / (1.0f / LINE_SEGS));
    /* val is between min and max */
    if (per != 1 && per != 0) {
        if (line_segs[2][seg] == 0) {
            line_segs[2][seg] = 1;
            LOGD("CompassCalibration:fit_line,z[%d]=[%f,%f,%f]", seg, x, y, z);
            return 1;
        }
    }
    return 0;
}

/* reset calibration algorithm */
static void reset()
{
    calData.minx = calData.miny = calData.minz = 0;
    calData.maxx = calData.maxy = calData.maxz = 0;
    for (int i = 0; i < 3; i++)
        for (int k = 0; k < 3; k++)
            calData.matrix[i][k] = 0;
    calibrated = 0;
    prev_p.time = 0;
    prev_p.p[0] = 0;
    prev_p.p[1] = 0;
    prev_p.p[2] = 0;
    fit_line = 0;
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < LINE_SEGS; j ++)
            line_segs[i][j] = 0;
}

void CompassCal_init(int calied, CompassCalData* data)
{
    reset();
    if (calied == 0)
        return;
    calData.minx = data->minx;
    calData.maxx = data->maxx;
    calData.miny = data->miny;
    calData.maxy = data->maxy;
    calData.minz = data->minz;
    calData.maxz = data->maxz;

    calData.matrix[0][0] = data->matrix[0][0];
    calData.matrix[1][1] = data->matrix[1][1];
    calData.matrix[2][2] = data->matrix[2][2];
    calibrated = 1;
}

/* return 0 reject value, return 1 accept value. */
int CompassCal_collectData(float rawMagX, float rawMagY, float rawMagZ, long currentTimeMSec)
{
    if (abnormal_data(rawMagX, rawMagY, rawMagZ, currentTimeMSec))
        return 0;

    if (calData.maxx == 0 && calData.minx == 0)
        calData.maxx = calData.minx = rawMagX;
    else {
        if (rawMagX > calData.maxx)
            calData.maxx = rawMagX;
        if (rawMagX < calData.minx)
            calData.minx = rawMagX;
    }

    if (calData.maxy == 0 && calData.miny == 0)
        calData.maxy = calData.miny = rawMagY;
    else {
        if (rawMagY > calData.maxy)
            calData.maxy = rawMagY;
        if (rawMagY < calData.miny)
            calData.miny = rawMagY;
    }

    if (calData.maxz == 0 && calData.minz == 0)
        calData.maxz = calData.minz = rawMagZ;
    else {
        if (rawMagZ > calData.maxz)
            calData.maxz = rawMagZ;
        if (rawMagZ < calData.minz)
            calData.minz = rawMagZ;
    }

    if (fit_line)
        fit_line_segs(rawMagX, rawMagY, rawMagZ);
    return 1;
}

/* check if calibration complete */
int CompassCal_readyCheck()
{
    if (calibrated == 1)
        return calibrated;

    /* condition1: reach MAG_MIN at each axe */
    if (calData.maxx - calData.minx < 2 * MAG_MIN ||
        calData.maxy - calData.miny < 2 * MAG_MIN ||
        calData.maxz - calData.minz < 2 * MAG_MIN)
        return 0;

    /* condition 2: reach every line seg of every axe */
    fit_line = 1;
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < LINE_SEGS; j++)
            if (line_segs[i][j] == 0)
                return 0;
    }

    calibrated = 1;
    return calibrated;
}

void CompassCal_computeCal(CompassCalData* data)
{
    /* Compute soft/hard iron, compute offset
     * and scale only, no rotate affect involved.
     */
    float max_length = calData.maxx - calData.minx;
    if (calData.maxy - calData.miny > max_length)
        max_length = calData.maxy - calData.miny;
    if (calData.maxz - calData.minz > max_length)
        max_length = calData.maxz - calData.minz;
    calData.matrix[0][0] = max_length / (calData.maxx - calData.minx);
    calData.matrix[1][1] = max_length / (calData.maxy - calData.miny);
    calData.matrix[2][2] = max_length / (calData.maxz - calData.minz);

    data->minx = calData.minx;
    data->maxx = calData.maxx;
    data->miny = calData.miny;
    data->maxy = calData.maxy;
    data->minz = calData.minz;
    data->maxz = calData.maxz;

    for (int i = 0; i < 3; i++)
        for (int k = 0; k < 3; k++)
            data->matrix[i][k] = calData.matrix[i][k];
}
