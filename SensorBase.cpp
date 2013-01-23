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
#define LOG_TAG "Sensors"

#include "SensorBase.h"

SensorBase::SensorBase(const sensor_platform_config_t *config)
    : mConfig(config), data_fd(-1)
{ }

SensorBase::~SensorBase()
{
    if (data_fd >= 0)
        close(data_fd);
}

int SensorBase::getFd() const
{
    return data_fd;
}

int SensorBase::setDelay(int32_t handle, int64_t ns)
{
    return 0;
}

bool SensorBase::hasPendingEvents() const
{
    return false;
}

int64_t SensorBase::getTimestamp()
{
    struct timespec t;
    t.tv_sec = t.tv_nsec = 0;
    clock_gettime(CLOCK_MONOTONIC, &t);

    return int64_t(t.tv_sec)*1000000000LL + t.tv_nsec;
}

int SensorBase::openInputDev(const char* inputName)
{
    int fd = -1;
    const char *dirname = "/dev/input";
    char devname[PATH_MAX];
    char *filename;
    DIR *dir;
    struct dirent *de;
    dir = opendir(dirname);
    if(dir == NULL)
        return -1;

    strcpy(devname, dirname);
    filename = devname + strlen(devname);
    *filename++ = '/';
    while ((de = readdir(dir))) {
        if (de->d_name[0] == '.' &&
            (de->d_name[1] == '\0' ||
             (de->d_name[1] == '.' && de->d_name[2] == '\0')))
            continue;

        strcpy(filename, de->d_name);
        fd = open(devname, O_RDONLY);
        if (fd >= 0) {
            char name[80];
            if (ioctl(fd, EVIOCGNAME(sizeof(name) - 1), &name) < 1) {
                name[0] = '\0';
            }
            if (!strcmp(name, inputName))
                break;

            close(fd);
            fd = -1;
        }
    }
    closedir(dir);
    return fd;
}

static char *trim_space(char *str)
{
    char *end;

    while (isspace(*str))
        str++;
    if (*str == 0)
        return 0;
    end = str + strlen(str) - 1;
    while (end > str && isspace(*end))
        *end-- = 0;
    return str;
}

int SensorBase::openFile(const char *all_path, int flags)
{
    int fd;
    char *str, *str_left;
    char path[PATH_MAX_LEN] = { 0 };

    strncpy(path, all_path, MIN(strlen(all_path), sizeof(path)));
    str_left = path;
    while ((str = strsep(&str_left, ";")) != NULL) {
        str = trim_space(str);
        if (!str)
            break;
        if (access(str, F_OK) == 0) {
            if ((fd = open(str, flags)) < 0) {
                return -1;
            } else {
                LOGI("Sensor HAL: Open file %s", str);
                return fd;
            }
        }
    }

    return -1;
}
