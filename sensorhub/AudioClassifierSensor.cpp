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
#include <fcntl.h>
#include <errno.h>
#include <poll.h>
#include <unistd.h>
#include <dirent.h>
#include <sys/select.h>
#include <assert.h>

#include <cutils/log.h>
#include <utils/AndroidThreads.h>
#include <libsensorhub.h>
#include "AudioClassifierSensor.h"
#define PSH_SESSION_NOT_OPENED       NULL
#define PIPE_NOT_OPENED              -1
#define THREAD_NOT_STARTED           0
#define SLEEP_ON_FAIL_USEC           500000
#define AUDIO_FLAG                   NO_STOP_NO_REPORT_WHEN_SCREEN_OFF
#define TIME_DELAY_FOR_PSH           5000
#define FILT_COEFF_QFACTOR           15
#define ALPHA_QFACTOR                14
#define PIPE_AWARE_OUT               0x10
#define AWARE_ALGO_MOD_ID            0x80
#define AUDIO_INSTANT                0
#define AUDIO_SHORT                  30 //seconds
#define AUDIO_MEDIUM                 120//seconds
#define AUDIO_LONG                   300//seconds
#define CONVERSION_USEC_TO_SEC       1000
/*****************************************************************************/
#undef LOG_TAG
#define LOG_TAG "AudioClassifierSensor"

AudioClassifierSensor::AudioClassifierSensor() :
        SensorBase("audioclassifier"), mEnabled(0), mHasPendingEvent(false) {
    LOGI("AudioClassifierSensor");
    mPendingEvent.version = sizeof(sensors_event_t);
    mPendingEvent.sensor = SENSORS_HANDLE_AUDIO_CLASSIFICATION;
    mPendingEvent.type = SENSOR_TYPE_AUDIO_CLASSIFICATION;
    memset(mPendingEvent.data, 0, sizeof(mPendingEvent.data));
    pthread_cond_init(&mCondTimeOut, NULL);
    pthread_mutex_init(&mMutexTimeOut, NULL);
    mResultPipe[0] = PIPE_NOT_OPENED;
    mResultPipe[1] = PIPE_NOT_OPENED;
    mCurrentDelay = 0;
    mAudioHal = new AudioHAL();
    mWakeFDs[0] = PIPE_NOT_OPENED;
    mWakeFDs[1] = PIPE_NOT_OPENED;
    mAudioHandle = PSH_SESSION_NOT_OPENED;
    mWorkerThread = THREAD_NOT_STARTED;
    setupResultPipe();
}

AudioClassifierSensor::~AudioClassifierSensor() {
    LOGI("~AudioClassifierSensor");
    stopWorker();
    // close connection
    disconnectFromPSH();
    // close pipes
    tearDownResultPipe();
}
inline bool AudioClassifierSensor::setupResultPipe() {
    if (!isResultPipeSetup())
        ;
    pipe(mResultPipe);
    return isResultPipeSetup();
}
inline bool AudioClassifierSensor::isResultPipeSetup() {
    if (mResultPipe[0] != PIPE_NOT_OPENED && mResultPipe[1] != PIPE_NOT_OPENED)
        return true;
    else
        return false;
}

inline void AudioClassifierSensor::tearDownResultPipe() {
    if (mResultPipe[0] != PIPE_NOT_OPENED)
        close(mResultPipe[0]);

    if (mResultPipe[1] != PIPE_NOT_OPENED)
        close(mResultPipe[1]);

    mResultPipe[0] = mResultPipe[1] = PIPE_NOT_OPENED;
}

inline bool AudioClassifierSensor::setUpThreadWakeupPipe() {
    if (!isThreadWakeupPipeSetup())
        pipe(mWakeFDs);
    return isThreadWakeupPipeSetup();
}

inline bool AudioClassifierSensor::isThreadWakeupPipeSetup() {
    if (mWakeFDs[0] != PIPE_NOT_OPENED && mWakeFDs[1] != PIPE_NOT_OPENED)
        return true;
    else
        return false;
}

inline void AudioClassifierSensor::tearDownThreadWakeupPipe() {
    if (mWakeFDs[0] != PIPE_NOT_OPENED)
        close(mWakeFDs[0]);

    if (mWakeFDs[1] != PIPE_NOT_OPENED)
        close(mWakeFDs[1]);

    mWakeFDs[0] = mWakeFDs[1] = PIPE_NOT_OPENED;
}
inline void AudioClassifierSensor::connectToPSH() {
    if (isConnectToPSH()) {
        LOGE("psh session is already opened");
        return;
    }
    // Establish audio classifier connection to PSH
    mAudioHandle = psh_open_session(SENSOR_LPE);
    if (mAudioHandle == PSH_SESSION_NOT_OPENED) {
        LOGE("psh_open_session failed. retry once");
        usleep(SLEEP_ON_FAIL_USEC);
        mAudioHandle = psh_open_session(SENSOR_LPE);
        if (mAudioHandle == PSH_SESSION_NOT_OPENED) {
            LOGE("psh_open_session failed.");
            return;
        }
    }
}

inline int AudioClassifierSensor::startStreamForPSH() {
    if (!isConnectToPSH()) {
        LOGE("session not started.");
        return -1;
    }
    //audio classifier 1 and 0 are just place holders
    if (psh_start_streaming_with_flag(mAudioHandle, 1, 0, AUDIO_FLAG)
            != ERROR_NONE) {
        LOGE("psh_start_streaming_with_flag failed.");
        usleep(SLEEP_ON_FAIL_USEC);
        if (psh_start_streaming_with_flag(mAudioHandle, 1, 0, AUDIO_FLAG)
                != ERROR_NONE) {
            LOGE("psh_start_streaming_with_flag failed.");
            return -1;
        }
    }
    return 0;
}

inline int AudioClassifierSensor::stopStreamForPSH() {
    LOGD("before psh_stop_streaming 0");
    int ret = 0;
    if (isConnectToPSH()) {
        LOGD("before psh_stop_streaming");
        if (psh_stop_streaming(mAudioHandle) != ERROR_NONE) {
            LOGE("psh_stop_streaming failed.");
            usleep(SLEEP_ON_FAIL_USEC);
            if (psh_stop_streaming(mAudioHandle) != ERROR_NONE) {
                LOGE("psh_stop_streaming failed.");
                ret = -1;
            }
        }
        LOGD("after psh_stop_streaming");
        if (ret == 0)
            LOGI("PSH streaming stopped.");
    }
    LOGD("after psh_stop_streaming 0");
    return ret;
}

inline int AudioClassifierSensor::propertySetForPSH(int delay) {
    if (!isConnectToPSH()) {
        LOGE("session not started.");
        return -1;
    }
    if (psh_set_property(mAudioHandle, PROP_STOP_REPORTING, &delay)
            != ERROR_NONE) {
        return -1;
    }
    return 0;
}
inline bool AudioClassifierSensor::isConnectToPSH() {
    if (mAudioHandle != PSH_SESSION_NOT_OPENED)
        return true;
    else
        return false;
}

inline void AudioClassifierSensor::disconnectFromPSH() {
    // close connections
    if (mAudioHandle != PSH_SESSION_NOT_OPENED)
        psh_close_session(mAudioHandle);
    mAudioHandle = PSH_SESSION_NOT_OPENED;
}
AudioClassifierSensor::AudioHAL::AudioHAL() {
    device = NULL;
}
AudioClassifierSensor::AudioHAL::~AudioHAL() {
    audioHalDeActivate();
    device = NULL;
}
bool AudioClassifierSensor::AudioHAL::isInited() {
    return !(device == NULL);
}
bool AudioClassifierSensor::AudioHAL::audioHalInit() {
    /*Aware classifier init*/
    if (isInited()) {
        LOGD("It is already inited!");
        return true;
    }
    int result;
    const hw_module_t *module;
    result = hw_get_module_by_class(AWARE_HARDWARE_MODULE_ID,
            AWARE_HARDWARE_MODULE_ID_PRIMARY, &module);
    LOGD("%s load audio hw module %s.%s (%s)", __func__,
            AWARE_HARDWARE_MODULE_ID, AWARE_HARDWARE_MODULE_ID_PRIMARY,
            strerror(-result));

    if (result == 0) {
        result = aware_hw_device_open(module, &device);
        LOGD("device open result %d\n", result);
        if (result != 0) {
            return false;
        }
        LOGD("device version %d\n", device->common.version);
        if (device->common.version != AWARE_DEVICE_API_VERSION_CURRENT) {
            return false;
        }
    } else {
        return false;
    }
    return true;
}

void AudioClassifierSensor::AudioHAL::audioHalActivate() {
    /* Aware classifier activate*/
    if (!audioHalInit()) {
        LOGE("init audio hal failed");
        return;
    }
    aware_classifier_param_t * aware_classifier =
            (aware_classifier_param_t *) malloc(
                    sizeof(aware_classifier_param_t));
    aware_classifier->framesize = 128;
    aware_classifier->fborder = 24;
    aware_classifier->mfccorder = 12;
    aware_classifier->gmmorder = 20;
    aware_classifier->rastacoeff_b0 = 0.2 * (1 << FILT_COEFF_QFACTOR);
    aware_classifier->rastacoeff_b1 = 0.1 * (1 << FILT_COEFF_QFACTOR);
    aware_classifier->rastacoeff_a = 0.98 * (1 << FILT_COEFF_QFACTOR);

    aware_classifier->numframesperblock = 64;
    aware_classifier->numblocksperanalysis = 4;
    aware_classifier->minNumFrms = 30 * aware_classifier->numblocksperanalysis
            * aware_classifier->numframesperblock / 100;
    aware_classifier->thresholdSpeech = -2;
    aware_classifier->thresholdNonspeech = -1;
    aware_classifier->dB_offset = 21;
    aware_classifier->alpha = 0.95 * (1 << ALPHA_QFACTOR);

    LOGD("activating aware... %x\n", device);
    if (device->activate_aware_session((hw_device_t*) device, aware_classifier)
            == 0)
        LOGD("activated aware successful...\n");
    else
        LOGE("activated aware failed\n");
}
void AudioClassifierSensor::AudioHAL::audioHalDeActivate() {
    /*Aware classifier deactivate*/
    if (!audioHalInit()) {
        LOGE("init audio hal failed");
        return;
    }
    LOGD("deactivating aware session...");
    if (device->deactivate_aware_session((hw_device_t*) device) == 0)
        LOGD("deactivated aware session successful... \n");
    else
        LOGE("deactivated aware session failed \n");
}
int AudioClassifierSensor::enable(int32_t handle, int en) {
    LOGI("enable");
    if (!isResultPipeSetup()) {
        LOGE("Invalid status while enable");
        return -1;
    }
    if (mEnabled == en) {
        LOGI("Duplicate request");
        return -1;
    }
    mEnabled = en;
    if (0 == mEnabled) {
        stopWorker();
        mCurrentDelay = 0;
    }  // we start worker when setDelay
    return 0;
}

int AudioClassifierSensor::setDelay(int32_t handle, int64_t ns) {
    LOGI("setDelay");
    if (ns != SENSOR_DELAY_TYPE_AUDIO_CLASSIFICATION_INSTANT * 1000
            && ns != SENSOR_DELAY_TYPE_AUDIO_CLASSIFICATION_SHORT * 1000
            && ns != SENSOR_DELAY_TYPE_AUDIO_CLASSIFICATION_MEDIUM * 1000
            && ns != SENSOR_DELAY_TYPE_AUDIO_CLASSIFICATION_LONG * 1000) {
        LOGE("Delay not supported");
        return -1;
    }
    if (mEnabled == 0) {
        LOGE("setDelay while not enabled");
        return -1;
    }
    if (mCurrentDelay == ns) {
        LOGI("Setting the same delay, do nothing");
        return 0;
    }
    // Restart worker thread
    if (restartWorker(ns)) {
        return 0;
    } else {
        mCurrentDelay = 0;
        return -1;
    }
    return 0;
}

int AudioClassifierSensor::getAudioDelay(int64_t ns) {
    int audioDelay = 0;

    if (SENSOR_DELAY_TYPE_AUDIO_CLASSIFICATION_INSTANT * 1000 == ns) {
        audioDelay = AUDIO_INSTANT;
    } else if (SENSOR_DELAY_TYPE_AUDIO_CLASSIFICATION_SHORT * 1000 == ns) {
        audioDelay = AUDIO_SHORT;
    } else if (SENSOR_DELAY_TYPE_AUDIO_CLASSIFICATION_MEDIUM * 1000 == ns) {
        audioDelay = AUDIO_MEDIUM;
    } else if (SENSOR_DELAY_TYPE_AUDIO_CLASSIFICATION_LONG * 1000 == ns) {
        audioDelay = AUDIO_LONG;
    } else {
        audioDelay = 1;
    }
    return audioDelay;
}
bool AudioClassifierSensor::restartWorker(int64_t newDelay) {
    {
        Mutex::Autolock _l(mDelayMutex);
        mCurrentDelay = newDelay;
    }
    stopWorker();
    return startWorker();
}

void AudioClassifierSensor::threadID() {
    LOGD("Thread Id is %x", (unsigned int) android::getThreadId());
}
bool AudioClassifierSensor::startWorker() {
    // set up wake up pipes
    if (!setUpThreadWakeupPipe())
        goto error_handle;
    threadID();
    if (0 == pthread_create(&mWorkerThread, NULL, workerThread, this)) {
        LOGI("create thread succeed");
        return true;
    } else {
        LOGE("create thread failed");
    }
    error_handle: tearDownThreadWakeupPipe();
    mWorkerThread = THREAD_NOT_STARTED;
    return false;
}

void AudioClassifierSensor::stopWorker() {
    if (mWorkerThread == THREAD_NOT_STARTED) {
        LOGI("Stop worker thread while thread is not running");
        return;
    }
    // when thread is active, the wake up pipes should be valid
    if (!isThreadWakeupPipeSetup()) {
        LOGE("isThreadWakeupPipeSetup = false");
    }
    // notify thread to exit
    pthreadStopWait();
    write(mWakeFDs[1], "a", 1);
    pthread_join(mWorkerThread, NULL);
    LOGI("Worker Thread Ended");
    // reset status
    tearDownThreadWakeupPipe();
    mWorkerThread = THREAD_NOT_STARTED;
}

bool AudioClassifierSensor::hasPendingEvents() const {
    return false;
}

int AudioClassifierSensor::getFd() const {
    LOGI("Fd is %d, %d", mResultPipe[0], mResultPipe[1]);
    return mResultPipe[0];
}

int AudioClassifierSensor::readEvents(sensors_event_t* data, int count) {
    LOGI("readEvents");
    int *p_audio_data;
    int size, numEventReceived = 0;
    char buf[512];
    int unit_size = sizeof(*p_audio_data);
    if (count < 1)
        return -EINVAL;

    if (!isResultPipeSetup()) {
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
        p_audio_data = (int *) p;
        mPendingEvent.data[0] = (static_cast<float>(*p_audio_data));
        LOGI("Event value is %d", *p_audio_data);

        mPendingEvent.timestamp = getTimestamp();

        if (mEnabled == 1) {
            *data++ = mPendingEvent;
            numEventReceived++;
        }
        size = size - unit_size;
        p = p + unit_size;
    }

    D("read %d events", numEventReceived);
    return numEventReceived;

}
void* AudioClassifierSensor::workerThread(void *data) {
    AudioClassifierSensor *src = (AudioClassifierSensor*) data;
    int psh_fd = -1;
    struct lpe_phy_data audioData;
    // Get current delay
    int64_t delay;
    {
        Mutex::Autolock _l(src->mDelayMutex);
        delay = src->mCurrentDelay;
    }
    // start streaming and get read fd
    int nTimeDelay = TIME_DELAY_FOR_PSH;
    src->pthreadWait(
            src->getAudioDelay(src->mCurrentDelay) - TIME_DELAY_FOR_PSH / 1000);
    src->mAudioHal->audioHalActivate();
    src->connectToPSH();
    if (src->startStreamForPSH() == -1) {
        LOGE("psh_start_stream failed.");
        return NULL;
    }
    if (src->propertySetForPSH(nTimeDelay) == -1) {
        LOGE("psh_set_property failed.");
        return NULL;
    }
    psh_fd = psh_get_fd(src->mAudioHandle);

    // set up polling fds
    struct pollfd polls[2];
    polls[0].fd = src->mWakeFDs[0];
    polls[0].events = POLLIN;
    polls[0].revents = 0;
    polls[1].fd = psh_fd;
    polls[1].events = POLLIN;
    polls[1].revents = 0;
    // let's poll
    LOGI("before poll");
    while (poll(polls, 2, -1) > 0) {

        if ((polls[0].revents & POLLIN) != 0) {
            LOGI("Wake FD Polled.");
            break;
        }

        int size = read(psh_fd, &audioData, sizeof(audioData));

        if (size != sizeof(audioData)) {
            LOGE("Read End Unexpectedly, Size: %d", size);
            break;
        }
        int finalResult = -1;
        switch (audioData.lpe_msg & FOR_CLASSIFIER_MASK) {
        case 0:
            finalResult = SENSOR_EVENT_TYPE_AUDIO_CLASSIFICATION_CROWD;
            break;
        case 1:
            finalResult = SENSOR_EVENT_TYPE_AUDIO_CLASSIFICATION_SOFT_MUSIC;
            break;
        case 2:
            finalResult = SENSOR_EVENT_TYPE_AUDIO_CLASSIFICATION_MECHANICAL;
            break;
        case 3:
            finalResult = SENSOR_EVENT_TYPE_AUDIO_CLASSIFICATION_MOTION;
            break;
        case 5:
            finalResult = SENSOR_EVENT_TYPE_AUDIO_CLASSIFICATION_FEMALE_SPEECH;
            break;
        case 6:
            finalResult = SENSOR_EVENT_TYPE_AUDIO_CLASSIFICATION_SILENT;
            break;
        default:
            finalResult = SENSOR_EVENT_TYPE_AUDIO_CLASSIFICATION_UNKNOWN;
            break;
        }
        LOGI("Update %d", audioData.lpe_msg & FOR_CLASSIFIER_MASK);
        // write to result pipe
        write(src->mResultPipe[1], &finalResult, sizeof(finalResult));
        // re-open PSH
        if (src->stopStreamForPSH() == -1) {
            LOGE("stopStreamForPSH failed");
        }
        src->disconnectFromPSH();
        src->mAudioHal->audioHalDeActivate();
        //wait
        src->pthreadWait(
                src->getAudioDelay(src->mCurrentDelay)
                        - TIME_DELAY_FOR_PSH / 1000);
        //wait end
        src->mAudioHal->audioHalActivate();
        src->connectToPSH();
        if (src->startStreamForPSH() == -1) {
            LOGE("psh_start_stream failed.");
            return NULL;
        }
        if (src->propertySetForPSH(nTimeDelay) == -1) {
            LOGE("psh_set_property failed.");
            return NULL;
        }
        psh_fd = psh_get_fd(src->mAudioHandle);
        // set up polling fds
        polls[1].fd = psh_fd;
        LOGI("before poll");
    }
    // close streaming
    if (src->stopStreamForPSH() == -1) {
        LOGE("stopStreamForPSH failed");
    }
    src->disconnectFromPSH();
    src->mAudioHal->audioHalDeActivate();
    pthread_exit (NULL);
    return NULL;
}

/*
 @Description : the pthread wait with time out
 @Parameters  : timeout
 @Return      : void
 @Note        :
 */
void AudioClassifierSensor::pthreadWait(int timeout) {
    LOGD("PthreadWait start");
    if (timeout < 0)
        timeout = 0;
    struct timeval now;
    struct timespec outtime;
    pthread_mutex_lock(&mMutexTimeOut);
    gettimeofday(&now, NULL);
    outtime.tv_sec = now.tv_sec + timeout;
    outtime.tv_nsec = now.tv_usec * CONVERSION_USEC_TO_SEC;
    pthread_cond_timedwait(&mCondTimeOut, &mMutexTimeOut, &outtime);
    pthread_mutex_unlock(&mMutexTimeOut);
    LOGD("PthreadWait stop");
}

void AudioClassifierSensor::pthreadStopWait() {
    LOGD("PthreadStopWait");
    pthread_mutex_lock(&mMutexTimeOut);
    pthread_cond_signal(&mCondTimeOut);
    pthread_mutex_unlock(&mMutexTimeOut);
    LOGD("PthreadStopWait stop");
}
