//#define SENSOR_DBG
//#define DBG_RAW_DATA
#include "CompassSensor.h"
#include "CompassCalibration.h"
#include "mat.h"

using namespace android;

#define DS_SIZE  24
typedef mat<double, 3, DS_SIZE> mat_input_t;

#define MIN_DIFF 1.5f
#define MAX_SQR_ERR 4.5f
#define LOOKBACK_COUNT 6

/* collected sensor data. Note we collect 2 set of data,
   1 for calibration calculation, the other for verification
   of calibration result*/
static float select_points[2*DS_SIZE][3];
static int select_point_count = 0;

#ifdef DBG_RAW_DATA
#define MAX_RAW_DATA_COUNT 2000
static FILE *raw_data = NULL;
static FILE *raw_data_selected = NULL;
static int raw_data_count = 0;
int file_no = 0;
#endif

static CompassCalData cal_data;

static bool ellipsoid_fit(const mat_input_t &M, CompassCalData &data)
{
    int i, j;
    mat<double, 6, DS_SIZE> H;
    mat<double, 1, DS_SIZE> W;

    for (i = 0; i < DS_SIZE; ++i) {
        W[0][i] = M[0][i] * M[0][i];
        H[0][i] = M[0][i];
        H[1][i] = M[1][i];
        H[2][i] = M[2][i];
        H[3][i] = -1 * M[1][i] * M[1][i];
        H[4][i] = -1 * M[2][i] * M[2][i];
        H[5][i] = 1;
    }

    mat<double, DS_SIZE, 6> H_trans = transpose(H);
    mat<double, 6, 6> temp1 = invert(H_trans * H);
    mat<double, DS_SIZE, 6> temp2 = temp1 * H_trans;
    mat<double, 1, 6> P = temp2 * W;

    data.off_x = 0.5 * P[0][0];
    data.off_y = 0.5 * P[0][1] / P[0][3];
    data.off_z = 0.5 * P[0][2] / P[0][4];

    double A11, A22, A33;
    A11 = 1.0f/(P[0][5] + data.off_x * data.off_x + P[0][3] * data.off_y * data.off_y
          + P[0][4] * data.off_z * data.off_z);
    A22 = P[0][3] * A11;
    A33 = P[0][4] * A11;

    if (isnan(A11) || isinf(A11) || isnan(A22) || isinf(A22) || isnan(A33) || isinf(A33)) {
        D("CompassCalibration: calibration failed! found NaN or Inf number!");
        return false;
    }

    data.w11 = sqrt(A11);
    data.w22 = sqrt(A22);
    data.w33 = sqrt(A33);
    data.bfield = 1.0 / pow(data.w11 * data.w22 * data.w33, 1.0/3.0);
    data.w11 *= data.bfield;
    data.w22 *= data.bfield;
    data.w33 *= data.bfield;

    return true;
}

/* reset calibration algorithm */
static void reset()
{
    select_point_count = 0;
    for (int i = 0; i < 2*DS_SIZE; ++i)
        for (int j=0; j < 3; ++j)
            select_points[i][j] = 0;
}

void CompassCal_init(int caled, const CompassCalData &data)
{

#ifdef DBG_RAW_DATA
    if (raw_data) {
        fclose(raw_data);
        raw_data = NULL;
    }

    if (raw_data_selected) {
        fclose(raw_data_selected);
        raw_data_selected = NULL;
    }

    // open raw data file
    char path[64];
    snprintf(path, 64, "/data/raw_compass_data_full_%d.txt", file_no);
    raw_data = fopen(path, "w+");
    snprintf(path, 64, "/data/raw_compass_data_selected_%d.txt", file_no);
    raw_data_selected = fopen(path, "w+");
    ++file_no;
    raw_data_count = 0;
#endif

    reset();
    if (caled) {
        cal_data = data;
        D("CompassCalibration: load old data, caldata: %f %f %f %f %f %f %f",
            cal_data.off_x, cal_data.off_y, cal_data.off_z, cal_data.w11,
            cal_data.w22, cal_data.w33, cal_data.bfield);
    } else {
        cal_data.off_x = 0;
        cal_data.off_y = 0;
        cal_data.off_z = 0;
        cal_data.w11 = 1;
        cal_data.w22 = 1;
        cal_data.w33 = 1;
    }
}

/* return 0 reject value, return 1 accept value. */
int CompassCal_collectData(float rawMagX, float rawMagY, float rawMagZ, long currentTimeMSec)
{
    float data[3] = {rawMagX, rawMagY, rawMagZ};

#ifdef DBG_RAW_DATA
    if (raw_data && raw_data_count < MAX_RAW_DATA_COUNT) {
        fprintf(raw_data, "%f %f %f %d\n", (double)rawMagX, (double)rawMagY,
                 (double)rawMagZ, (int)currentTimeMSec);
        raw_data_count++;
    }

    if (raw_data && raw_data_count >= MAX_RAW_DATA_COUNT) {
        fclose(raw_data);
        raw_data = NULL;
    }
#endif

    // For the current point to be accepted, each x/y/z value must be different enough
    // to the last several collected points
    if (select_point_count > 0 && select_point_count < 2 * DS_SIZE) {
        int lookback = LOOKBACK_COUNT < select_point_count ? LOOKBACK_COUNT : select_point_count;
        for (int index = 0; index < lookback; ++index){
            for (int j = 0; j < 3; ++j) {
                if (fabsf(data[j] - select_points[select_point_count-1-index][j]) < MIN_DIFF) {
                    D("CompassCalibration:point reject: [%f,%f,%f], selected_count=%d",
                       (double)data[0], (double)data[1], (double)data[2], select_point_count);
                        return 0;
                }
            }
        }
    }

    if (select_point_count < 2 * DS_SIZE) {
        memcpy(select_points[select_point_count], data, sizeof(float) * 3);
        ++select_point_count;
        D("CompassCalibration:point collected [%f,%f,%f], selected_count=%d",
            (double)data[0], (double)data[1], (double)data[2], select_point_count);
#ifdef DBG_RAW_DATA
        if (raw_data_selected) {
            fprintf(raw_data_selected, "%f %f %f\n", (double)data[0], (double)data[1], (double)data[2]);
        }
#endif
    }
   return 1;
}


// use second data set to calculate square error
double calc_square_err(const CompassCalData &data)
{
    double err = 0;
    for (int i = DS_SIZE; i < 2 * DS_SIZE; ++i) {
        double x = (select_points[i][0] - data.off_x) * data.w11;
        double y = (select_points[i][1] - data.off_y) * data.w22;
        double z = (select_points[i][2] - data.off_z) * data.w33;
        double diff = sqrt(x*x + y*y + z*z) - data.bfield;
        err += diff * diff;
    }
    err /= DS_SIZE;
    return err;
}

/* check if calibration complete */
int CompassCal_readyCheck()
{
    mat_input_t mat;
    int ready = 0;

    if (select_point_count < 2*DS_SIZE)
        return 0;

    // enough points have been collected, do the ellipsoid calibration
    for (int i = 0; i < DS_SIZE; ++i) {
        mat[0][i] = select_points[i][0];
        mat[1][i] = select_points[i][1];
        mat[2][i] = select_points[i][2];
    }

    /* check if result is good */
    CompassCalData new_cal_data;
    if (ellipsoid_fit(mat, new_cal_data)) {
        double err_new = calc_square_err(new_cal_data);
        if (err_new < MAX_SQR_ERR) {
            double err = calc_square_err(cal_data);
            if (err_new < err) {
              // new cal_data is better, so we switch to the new
              cal_data = new_cal_data;
              ready = 1;
              D("CompassCalibration: ready check success, caldata: %f %f %f %f %f %f %f, err %f",
                cal_data.off_x, cal_data.off_y, cal_data.off_z, cal_data.w11,
                cal_data.w22, cal_data.w33, cal_data.bfield, err_new);
            }
        }
    }

    reset();
    return ready;
}

void CompassCal_computeCal(CompassCalData* data)
{
    // calculation has been done in readycheck(),
    // just return the result here
    if (data)
        *data = cal_data;
}
