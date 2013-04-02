/*
 * Copyright (C) 2008 The Android Open Source Project
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#include "GestureSensor.h"

#include <fcntl.h>
#include <errno.h>
#include <math.h>
#include <poll.h>
#include <unistd.h>
#include <dirent.h>
#include <sys/select.h>

#include <cutils/log.h>
#include <utils/AndroidThreads.h>

#include "gesture.h"

/*****************************************************************************/

#undef LOG_TAG
#define LOG_TAG "GestureSensor"

#define GS_BUF_SIZE     4096    /* gesture spotter buffer size */
#define PX_BUF_SIZE     512     /* proximity buffer size */
#define GS_SAMPLE_RATE  100     /* gesture spotter sampling rate, Hz */
#define GS_BUF_DELAY    0       /* gesture spotter buffer delay, ms */
#define PX_SAMPLE_RATE  5       /* proximity sampling rate, Hz */
#define PX_BUF_DELAY    0       /* proximity buffer delay, ms */
#define GS_DATA_LENGTH  6       /* length of g_spotter single data */

#define THREAD_NOT_STARTED 0
#define PIPE_NOT_OPENED -1
#define PSH_SESSION_NOT_OPENED NULL
#define INVALID_GESTURE_RESULT -1

GestureSensor::GestureSensor()
    : SensorBase("gesture"),
      mEnabled(0),
      mHasPendingEvent(false)
{
    mPendingEvent.version = sizeof(sensors_event_t);
    mPendingEvent.sensor = SENSORS_HANDLE_GESTURE;
    mPendingEvent.type = SENSOR_TYPE_GESTURE;
    memset(mPendingEvent.data, 0, sizeof(mPendingEvent.data));

    mPollingThreadIDGesture = THREAD_NOT_STARTED;
    mPollingThreadIDProximity = THREAD_NOT_STARTED;
    mFlagProximity = false;
    mHandleGesture = PSH_SESSION_NOT_OPENED;
    mHandleProximity = PSH_SESSION_NOT_OPENED;
    mWakeFdsGesture[0] = mWakeFdsGesture[1] = PIPE_NOT_OPENED;
    mWakeFdsProximity[0] = mWakeFdsProximity[1] = PIPE_NOT_OPENED;
    mResultPipe[0] = mResultPipe[1] = PIPE_NOT_OPENED;
    mInitGesture = false;

    // Establish result pipe to sensor HAL
    pipe(mResultPipe);

    // Start to load libgesture library
    mLibraryHandle = NULL;
    mGestureInit = NULL;
    mGestureProcessSingleData = NULL;
    mGestureClose = NULL;
    mHasGestureLibrary = false;

    mLibraryHandle = dlopen(LIBGESTURE, RTLD_NOW);
    if (mLibraryHandle != NULL) {
        mGestureInit = (FUNC_GESTURE_INIT) dlsym(mLibraryHandle,
                                                SYMBOL_GESTURE_INIT);
        mGestureProcessSingleData = (FUNC_GESTURE_PROCESS_SINGLE_DATA) dlsym(mLibraryHandle,
                                                SYMBOL_GESTURE_PROCESS_SINGLE_DATA);
        mGestureClose = (FUNC_GESTURE_CLOSE) dlsym(mLibraryHandle,
                                                SYMBOL_GESTURE_CLOSE);

        if (mGestureInit != NULL && mGestureProcessSingleData != NULL && mGestureClose != NULL) {
            mHasGestureLibrary = true;
            LOGI("Got all required functions");
        } else {
            LOGE("Can't get all required functions!!");
        }
    } else {
        LOGE("Can't find libgesture!!");
    }
}

GestureSensor::~GestureSensor()
{
    LOGI("~GestureSensor %d\n", mEnabled);
    stopGesture();
    stopProximity();

    if (mResultPipe[0] != PIPE_NOT_OPENED)
        close(mResultPipe[0]);

    if (mResultPipe[1] != PIPE_NOT_OPENED)
        close(mResultPipe[1]);

    if (mLibraryHandle != NULL) {
        dlclose(mLibraryHandle);
    }
}

int GestureSensor::enable(int32_t handle, int en)
{
    int flags = en ? 1 : 0;

    LOGI("GestureSensor - %s - enable=%d", __FUNCTION__, en);
    threadID();
    if (!mHasGestureLibrary) {
        LOGE("Gesture library error!");
        return -1;
    }

    if (mResultPipe[0] == PIPE_NOT_OPENED || mResultPipe[1] == PIPE_NOT_OPENED)
        return -1;

    if (mEnabled == en)
        return 0;

    if (en == 1) {
        bool gestureThreadStarted = startGesture();
        bool proximityThreadStarted = startProximity();

        if (!gestureThreadStarted || !proximityThreadStarted) {
            LOGE("Failed to start gesture or proximity thread");
            stopGesture();
            stopProximity();
            return -1;
        }
        mEnabled = 1;
    } else {
        stopGesture();
        stopProximity();
        mEnabled = 0;
    }

    return 0;
}

int GestureSensor::setDelay(int32_t handle, int64_t ns)
{
    LOGI("setDelay - %s - %lld", __FUNCTION__, ns);
    threadID();
    return 0;
}

bool GestureSensor::hasPendingEvents() const
{
    return mHasPendingEvent;
}

int GestureSensor::getFd() const
{
    LOGI("Fd is %d, %d", mResultPipe[0], mResultPipe[1]);
    threadID();
    return mResultPipe[0];
}

int GestureSensor::readEvents(sensors_event_t* data, int count)
{
    int size, numEventReceived = 0;
    char buf[512];
    int *p_gesture_data;
    int unit_size = sizeof(*p_gesture_data);

    LOGI("GestureSensor - readEvents");
    threadID();
    if (count < 1)
        return -EINVAL;

    if (mResultPipe[0] == PIPE_NOT_OPENED) {
        LOGI("invalid status ");
        return 0;
    }

    if ((unit_size * count) <= 512)
        size = unit_size * count;
    else
        size = (512 / unit_size) * unit_size;

    LOGI("Try to read %d", size);
    size = read(mResultPipe[0], buf, size);
    LOGI("Actually read %d", size);

    char *p = buf;
    while (size > 0) {
        p_gesture_data = (int *)(p);
        mPendingEvent.data[0] = (static_cast<float>(*p_gesture_data));
        LOGI("Event value is %d", *p_gesture_data);

        mPendingEvent.timestamp = getTimestamp();

        if (mEnabled == 1) {
           *data++ = mPendingEvent;
            numEventReceived++;
        }

        size = size - unit_size;
        p = p + unit_size;
    }

    D("GestureSensor - read %d events", numEventReceived);
    return numEventReceived;
}

static const char * gestures[] = {
    "NumberOne ", "NumberTwo ", "NumberThree ", "NumberFour ",
    "NumberFive ", "NumberSix ", "NumberSeven ", "NumberEight ",
    "NumberNine ", "NumberZero ", "EarTouch ", "EarTouchBack "
};

int GestureSensor::getGestureFromString(char *gesture)
{
    int index = -1;
    for (int i = 0; i < (static_cast<int>(ARRAY_SIZE(gestures))); i++) {
        if (strcmp(gesture, gestures[i]) == 0) {
            index = i;
            break;
        }
    }

    if (index != -1)
        return SENSOR_EVENT_TYPE_GESTURE_NUMBER_ONE + index;
    else
        return INVALID_GESTURE_RESULT;
}

/* thread to process raw accel and gyro data with libgesture */
void* GestureSensor::pollingThreadGesture(void *source)
{
    LOGD("started thread gesture: wait for spotter data");
    threadID();

    GestureSensor* sensorSrc = static_cast<GestureSensor*>(source);
    int fd = psh_get_fd(sensorSrc->mHandleGesture);
    char* buf = new char[GS_BUF_SIZE];
    fd_set read_fds;
    int max_fd;

    FD_ZERO(&read_fds);
    FD_SET(fd, &read_fds);
    FD_SET(sensorSrc->mWakeFdsGesture[0], &read_fds);

    if (fd > sensorSrc->mWakeFdsGesture[0])
        max_fd = fd;
    else
        max_fd = sensorSrc->mWakeFdsGesture[0];

    while (select(max_fd + 1, &read_fds, NULL, NULL, NULL) >= 0) {
        /* if sensor is shut down, return */
        if (FD_ISSET(sensorSrc->mWakeFdsGesture[0], &read_fds)) {
            LOGD("thread gesture should end now");
            break;
        }

        int size = 0;
        /* get gesture spotter sensor data */
        size = read(fd, buf, GS_BUF_SIZE);
        char *p = buf;
        struct gs_data *p_gs_data;

        while (size > 0) {
            p_gs_data = (struct gs_data *)p;
            bool last = false;
            int i = 0;
            LOGI("Data available for process");
            threadID();

            while (last == false) {
                if (i*GS_DATA_LENGTH*2 + GS_DATA_LENGTH*2 >= p_gs_data->size)
                    last = true;
                short * data = &p_gs_data->sample[i*GS_DATA_LENGTH];
                if (last == true)
                    LOGD("num %d: %hd %hd %hd %hd %hd %hd: last %d\n",
                            i, data[0], data[1], data[2], data[3],
                            data[4], data[5], last);

                i++;

                /* process with libgesture */
                /* CAUTION: this function is not multi-thread safe */
                sensorSrc->mMutexGesture.lock();
                char *gesture = (*sensorSrc->mGestureProcessSingleData)(data, true, last);
                sensorSrc->mMutexGesture.unlock();

                /* if gesture is detected */
                if (gesture != NULL) {
                    LOGD("event -- gesture: %s", gesture);
                    threadID();
                    /* change EarTouchL to EarTouch, same as EarTouchLBack */
                    if (strcmp(gesture, "EarTouchL ") == 0)
                        strcpy(gesture, "EarTouch ");
                    else if (strcmp(gesture, "EarTouchLBack ") == 0)
                        strcpy(gesture, "EarTouchBack ");
                    sensorSrc->mMutexProximity.lock();
                    bool proximityNear = sensorSrc->mFlagProximity;
                    sensorSrc->mMutexProximity.unlock();
                    bool eartouchDetected = (strcmp(gesture, "EarTouch ") == 0);
                    /* eartouch end with prox = 1, others end with prox = 0 */
                    if ((proximityNear && eartouchDetected)
                        || ((!proximityNear) && (!eartouchDetected))) {
                        int gestureResult = getGestureFromString(gesture);
                        if (gestureResult != INVALID_GESTURE_RESULT) {
                            LOGI("write into result pipe start");
                            write(sensorSrc->mResultPipe[1], (char*)(&gestureResult),
                                  sizeof(gestureResult));
                            LOGI("write into result pipe end");
                        }
                    }
                }
                delete [] gesture;
            }

            size = size - (sizeof(struct gs_data) + p_gs_data->size);
            p = p + sizeof(struct gs_data) + p_gs_data->size;
        }
        FD_SET(fd, &read_fds);
        FD_SET(sensorSrc->mWakeFdsGesture[0], &read_fds);
    }
    delete [] buf;
    LOGD("ending of gesture thread");
    threadID();
    pthread_exit(NULL);
    return NULL;
}

/* thread to process proximity data */
void* GestureSensor::pollingThreadProximity(void *source)
{
    LOGD("started thread proximity: wait for proximity data");
    threadID();

    GestureSensor* sensorSrc = static_cast<GestureSensor*>(source);
    int fd = psh_get_fd(sensorSrc->mHandleProximity);
    char* buf = new char[PX_BUF_SIZE];
    int size = 0;
    struct ps_phy_data *p_ps_phy_data;
    fd_set read_fds;
    int max_fd;

    FD_ZERO(&read_fds);
    FD_SET(fd, &read_fds);
    FD_SET(sensorSrc->mWakeFdsProximity[0], &read_fds);

    if (fd > sensorSrc->mWakeFdsProximity[0])
        max_fd = fd;
    else
        max_fd = sensorSrc->mWakeFdsProximity[0];

    while (select(max_fd + 1, &read_fds, NULL, NULL, NULL) >= 0) {
        /* if sensor is shut down, return */
        if (FD_ISSET(sensorSrc->mWakeFdsProximity[0], &read_fds)) {
            LOGD("thread proximity should end now");
            break;
        }
        /* get proximity sensor data */
        size = read(fd, buf, PX_BUF_SIZE);
        char *p = buf;
        while (size > 0) {
            p_ps_phy_data = (struct ps_phy_data *) p;
            LOGD("event -- proximity: %d", p_ps_phy_data->near);
            threadID();

            sensorSrc->mMutexProximity.lock();
            if (p_ps_phy_data->near == 1)
                sensorSrc->mFlagProximity = true;
            else
                sensorSrc->mFlagProximity = false;
            sensorSrc->mMutexProximity.unlock();

            size = size - (sizeof(struct ps_phy_data));
            p = p + sizeof(struct ps_phy_data);
        }
        FD_SET(fd, &read_fds);
        FD_SET(sensorSrc->mWakeFdsProximity[0], &read_fds);
    }
    delete [] buf;
    LOGD("ending of proximity thread");
    threadID();
    pthread_exit(NULL);
    return NULL;
}

/* start gesture algorithm, and start gesture_spotting in psh */
/* start gesture polling thread */
bool GestureSensor::startGesture()
{
    LOGI("startGesture");
    threadID();
    mMutexGesture.lock();
    bool r = (*mGestureInit)(0, NULL);  /* use default model */
    mMutexGesture.unlock();
    mInitGesture = true;
    if (r == true) {
        LOGD("session open - gesture");
        mHandleGesture = psh_open_session(SENSOR_GS);
        if (mHandleGesture != PSH_SESSION_NOT_OPENED) {
            error_t ret;
            ret = psh_start_streaming(mHandleGesture,
                                    GS_SAMPLE_RATE, GS_BUF_DELAY);
            if (ret == ERROR_NONE) {
                LOGD("start pthread - gesture");
                pipe(mWakeFdsGesture);
                if (mWakeFdsGesture[0] == PIPE_NOT_OPENED
                    || mWakeFdsGesture[1] == PIPE_NOT_OPENED)
                    return false;

                int err = pthread_create(&mPollingThreadIDGesture, NULL,
                                    pollingThreadGesture, this);
                if (err == 0)
                    return true;
            }
        }
    }
    stopGesture();
    LOGE("psh & algorithm start return false - gesture");
    return false;
}

void GestureSensor::threadID()
{
    LOGD("Thread Id is %x", (unsigned int)android::getThreadId());
}

/* stop gesture algorithm, and stop gesture in psh */
/* stop gesture polling thread */
void GestureSensor::stopGesture()
{
    LOGI("stopGesture");
    threadID();
    if (mInitGesture == true) {
        LOGD("stop gesture algorithm");
        mMutexGesture.lock();
        (*mGestureClose)();
        mMutexGesture.unlock();
        mInitGesture = false;
    }
    if (mHandleGesture != PSH_SESSION_NOT_OPENED) {
        LOGD("stop gesture session");
        psh_stop_streaming(mHandleGesture);
        psh_close_session(mHandleGesture);
        mHandleGesture = PSH_SESSION_NOT_OPENED;
    }
    if (mPollingThreadIDGesture != THREAD_NOT_STARTED) {
        LOGD("stop gesture thread");
        write(mWakeFdsGesture[1], "a", 1);
        if (mPollingThreadIDGesture != THREAD_NOT_STARTED) {
            pthread_join(mPollingThreadIDGesture, NULL);
            mPollingThreadIDGesture = THREAD_NOT_STARTED;
        }
        LOGD("finish waiting for gesture thread to exit");
    }
    if (mWakeFdsGesture[0] != PIPE_NOT_OPENED)
        close(mWakeFdsGesture[0]);
    if (mWakeFdsGesture[1] != PIPE_NOT_OPENED)
        close(mWakeFdsGesture[1]);
    mWakeFdsGesture[0] = mWakeFdsGesture[1] = PIPE_NOT_OPENED;
}

/* start proximity in psh */
/* start proximity polling thread */
bool GestureSensor::startProximity()
{
    LOGD("startProximity");
    threadID();
    mHandleProximity = psh_open_session(SENSOR_PROXIMITY);
    if (mHandleProximity != PSH_SESSION_NOT_OPENED) {
        error_t ret;
        ret = psh_start_streaming(mHandleProximity,
                                    PX_SAMPLE_RATE, PX_BUF_DELAY);
        if (ret == ERROR_NONE) {
            LOGD("start proximity thread");
            pipe(mWakeFdsProximity);
            if (mWakeFdsProximity[0] == PIPE_NOT_OPENED
                || mWakeFdsProximity[1] == PIPE_NOT_OPENED)
                return false;

            int err = pthread_create(&mPollingThreadIDProximity, NULL,
                                    pollingThreadProximity, this);
            if (err == 0)
                return true;
        }
    }
    stopProximity();
    LOGE("proximity connection to psh failed");
    return false;
}

/* stop proximity in psh */
/* stop proximity polling thread */
void GestureSensor::stopProximity()
{
    LOGI("stopProximity");
    threadID();
    if (mHandleProximity != PSH_SESSION_NOT_OPENED) {
        LOGD("stop proximity psh connection");
        psh_stop_streaming(mHandleProximity);
        psh_close_session(mHandleProximity);
        mHandleProximity = PSH_SESSION_NOT_OPENED;
    }
    if (mPollingThreadIDProximity != THREAD_NOT_STARTED) {
        LOGD("stop proximity thread");
        write(mWakeFdsProximity[1], "a", 1);
        if (mPollingThreadIDProximity != THREAD_NOT_STARTED) {
            pthread_join(mPollingThreadIDProximity, NULL);
            mPollingThreadIDProximity = THREAD_NOT_STARTED;
        }
        LOGD("finish waiting for proximity thread to exit");
    }
    if (mWakeFdsProximity[0] != PIPE_NOT_OPENED)
        close(mWakeFdsProximity[0]);
    if (mWakeFdsProximity[1] != PIPE_NOT_OPENED)
        close(mWakeFdsProximity[1]);
    mWakeFdsProximity[0] = mWakeFdsProximity[1] = PIPE_NOT_OPENED;
}
