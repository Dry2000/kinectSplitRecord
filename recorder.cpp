// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.
//#define BOOST_PYTHON_STATIC_LIB
#include "recorder.h"
#include <ctime>
#include <chrono>
#include <atomic>
#include <iostream>
#include <algorithm>
#include <string>
#include <k4a/k4a.h>
#include <k4arecord/record.h>
//#include <boost/python.hpp>

using namespace std::chrono;

inline static uint32_t k4a_convert_fps_to_uint(k4a_fps_t fps)
{
    uint32_t fps_int;
    switch (fps)
    {
    case K4A_FRAMES_PER_SECOND_5:
        fps_int = 5;
        break;
    case K4A_FRAMES_PER_SECOND_15:
        fps_int = 15;
        break;
    case K4A_FRAMES_PER_SECOND_30:
        fps_int = 30;
        break;
    default:
        fps_int = 0;
        break;
    }
    return fps_int;
}

// call k4a_device_close on every failed CHECK
#define CHECK(x, device)                                                                                               \
    {                                                                                                                  \
        auto retval = (x);                                                                                             \
        if (retval)                                                                                                    \
        {                                                                                                              \
            std::cerr << "Runtime error: " << #x << " returned " << retval << std::endl;                               \
            k4a_device_close(device);                                                                                  \
            return 1;                                                                                                  \
        }                                                                                                              \
    }

std::atomic_bool exiting(false);

int do_recording(uint8_t device_index,
                 char *recording_filename,
                 int recording_length,
                 k4a_device_configuration_t *device_config,
                 bool record_imu,
                 int32_t absoluteExposureValue,
                 int32_t gain)
{   
    //system("python recordAudio.py");
    seconds recording_length_seconds(recording_length);
    //seconds infinit_record_seconds(1000000000);
    const uint32_t installed_devices = k4a_device_get_installed_count();

    if (device_index >= installed_devices)
    {
        std::cerr << "Device not found." << std::endl;
        return 1;
    }
    k4a_device_t device;
    if (K4A_FAILED(k4a_device_open(device_index, &device)))
    {
        std::cerr << "Runtime error: k4a_device_open() failed " << std::endl;
    }
    // 記録するファイル名を引数のものに設定する
    std::string filename = std::string(recording_filename);
    size_t i = filename.find(".");
    std::string fileName = filename.substr(0,i);
    const char* recordFileName = fileName.c_str();
    char serial_number_buffer[256];
    size_t serial_number_buffer_size = sizeof(serial_number_buffer);
    CHECK(k4a_device_get_serialnum(device, serial_number_buffer, &serial_number_buffer_size), device);

    std::cout << "Device serial number: " << serial_number_buffer << std::endl;

    k4a_hardware_version_t version_info;
    CHECK(k4a_device_get_version(device, &version_info), device);

    std::cout << "Device version: " << (version_info.firmware_build == K4A_FIRMWARE_BUILD_RELEASE ? "Rel" : "Dbg")
              << "; C: " << version_info.rgb.major << "." << version_info.rgb.minor << "." << version_info.rgb.iteration
              << "; D: " << version_info.depth.major << "." << version_info.depth.minor << "."
              << version_info.depth.iteration << "[" << version_info.depth_sensor.major << "."
              << version_info.depth_sensor.minor << "]"
              << "; A: " << version_info.audio.major << "." << version_info.audio.minor << "."
              << version_info.audio.iteration << std::endl;

    uint32_t camera_fps = k4a_convert_fps_to_uint(device_config->camera_fps);

    if (camera_fps <= 0 || (device_config->color_resolution == K4A_COLOR_RESOLUTION_OFF &&
                            device_config->depth_mode == K4A_DEPTH_MODE_OFF))
    {
        std::cerr << "Either the color or depth modes must be enabled to record." << std::endl;
        return 1;
    }




    
    if (absoluteExposureValue != defaultExposureAuto)
    {
        if (K4A_FAILED(k4a_device_set_color_control(device,
                                                    K4A_COLOR_CONTROL_EXPOSURE_TIME_ABSOLUTE,
                                                    K4A_COLOR_CONTROL_MODE_MANUAL,
                                                    absoluteExposureValue)))
        {
            std::cerr << "Runtime error: k4a_device_set_color_control() for manual exposure failed " << std::endl;
        }
    }
    else
    {
        if (K4A_FAILED(k4a_device_set_color_control(device,
                                                    K4A_COLOR_CONTROL_EXPOSURE_TIME_ABSOLUTE,
                                                    K4A_COLOR_CONTROL_MODE_AUTO,
                                                    0)))
        {
            std::cerr << "Runtime error: k4a_device_set_color_control() for auto exposure failed " << std::endl;
        }
    }

    if (gain != defaultGainAuto)
    {
        if (K4A_FAILED(
                k4a_device_set_color_control(device, K4A_COLOR_CONTROL_GAIN, K4A_COLOR_CONTROL_MODE_MANUAL, gain)))
        {
            std::cerr << "Runtime error: k4a_device_set_color_control() for manual gain failed " << std::endl;
        }
    }

    CHECK(k4a_device_start_cameras(device, device_config), device);
    if (record_imu)
    {
        CHECK(k4a_device_start_imu(device), device);
    }

    std::cout << "Device started" << std::endl;

    k4a_record_t recording;
    if (K4A_FAILED(k4a_record_create(recording_filename, device, *device_config, &recording)))
    {
        std::cerr << "Unable to create recording file: " << recording_filename << std::endl;
        return 1;
    }

    if (record_imu)
    {
        CHECK(k4a_record_add_imu_track(recording), device);
    }
    CHECK(k4a_record_write_header(recording), device);

    // Wait for the first capture before starting recording.
    k4a_capture_t capture;

    seconds timeout_sec_for_first_capture(60);
    if (device_config->wired_sync_mode == K4A_WIRED_SYNC_MODE_SUBORDINATE)
    {
        timeout_sec_for_first_capture = seconds(360);
        std::cout << "[subordinate mode] Waiting for signal from master" << std::endl;
    }
    
    steady_clock::time_point first_capture_start = steady_clock::now();
    k4a_wait_result_t result = K4A_WAIT_RESULT_TIMEOUT;
    // Wait for the first capture in a loop so Ctrl-C will still exit.
    while (!exiting && (steady_clock::now() - first_capture_start) < timeout_sec_for_first_capture)
    {
        result = k4a_device_get_capture(device, &capture, 100);
        if (result == K4A_WAIT_RESULT_SUCCEEDED)
        {
            k4a_capture_release(capture);
            break;
        }
        else if (result == K4A_WAIT_RESULT_FAILED)
        {
            std::cerr << "Runtime error: k4a_device_get_capture() returned error: " << result << std::endl;
            return 1;
        }
    }
    // 録画の終了中なら、kinectとの接続を解除する
    if (exiting)
    {
        k4a_device_close(device);
        return 0;
    }
    else if (result == K4A_WAIT_RESULT_TIMEOUT)
    {
        std::cerr << "Timed out waiting for first capture." << std::endl;
        return 1;
    }

    std::cout << "Started recording" << std::endl;
    if (recording_length <= 0)
    {
        std::cout << "Press Ctrl-C to stop recording." << std::endl;
    }
    int counter = 1;
    // 記録を開始したタイミングの時刻を記録する
    steady_clock::time_point recording_start = steady_clock::now();
    steady_clock::time_point recording_start_first_time = steady_clock::now();
    int32_t timeout_ms = 1000 / camera_fps;
    do{
        do
        {

            result = k4a_device_get_capture(device, &capture, timeout_ms);
            if (result == K4A_WAIT_RESULT_TIMEOUT)
            {
                continue;
            }
            else if (result != K4A_WAIT_RESULT_SUCCEEDED)
            {
                std::cerr << "Runtime error: k4a_device_get_capture() returned " << result << std::endl;
                break;
            }
            CHECK(k4a_record_write_capture(recording, capture), device);
            k4a_capture_release(capture);
            //std::cout << "roooooooooooooped!" << std::endl;
            if (record_imu)
            {
                while (true) {
                    //std::cout << "rooped!" << std::endl;
                    //録画に失敗するか、録画終了中でない時、または、規定時間録画するまで録画を実行する
                    do
                    {
                        k4a_imu_sample_t sample;
                        result = k4a_device_get_imu_sample(device, &sample, 0);
                        if (result == K4A_WAIT_RESULT_TIMEOUT)
                        {
                            break;
                        }
                        else if (result != K4A_WAIT_RESULT_SUCCEEDED)
                        {
                            std::cerr << "Runtime error: k4a_imu_get_sample() returned " << result << std::endl;
                            break;
                        }
                        k4a_result_t write_result = k4a_record_write_imu_sample(recording, sample);
                        if (K4A_FAILED(write_result))
                        {
                            std::cerr << "Runtime error: k4a_record_write_imu_sample() returned " << write_result << std::endl;
                            break;
                        }
                    } while (!exiting && result != K4A_WAIT_RESULT_FAILED &&
                        (recording_length < 0 || (steady_clock::now() - recording_start < recording_length_seconds)));
                        
                   
                }
            }
        } while (!exiting && result != K4A_WAIT_RESULT_FAILED &&
            (recording_length < 0 || (steady_clock::now() - recording_start < recording_length_seconds)));
        std::cout << "Saving recording..." << std::endl;
        CHECK(k4a_record_flush(recording), device);
        k4a_record_close(recording); 
        // ファイル名に録画開始からの経過時間を追加する
        seconds sec = duration_cast<seconds>(steady_clock::now() - recording_start_first_time);
        std::string secStr = std::to_string(sec.count());
        std::string filename = std::string(recordFileName) + "_" + secStr + ".mkv";
        const char* recFile = filename.c_str();

        //recording_filename = *recording_filename+secStr.c_str();
        if (K4A_FAILED(k4a_record_create(recFile, device, *device_config, &recording)))
        {
            std::cerr << "Unable to create recording file: " << recFile << std::endl;
            return 1;
        }

        if (record_imu)
        {
            CHECK(k4a_record_add_imu_track(recording), device);
        }
        CHECK(k4a_record_write_header(recording), device);
        recording_start = steady_clock::now();
    } while (!exiting && result != K4A_WAIT_RESULT_FAILED);
    
    if (!exiting)
    {
        exiting = true;
        std::cout << "Stopping recording..." << std::endl;
    }

    if (record_imu)
    {
        k4a_device_stop_imu(device);
    }
    k4a_device_stop_cameras(device);

    std::cout << "Saving recording..." << std::endl;
    CHECK(k4a_record_flush(recording), device);
    k4a_record_close(recording);

    std::cout << "Done" << std::endl;

    k4a_device_close(device);

    return 0;
}
