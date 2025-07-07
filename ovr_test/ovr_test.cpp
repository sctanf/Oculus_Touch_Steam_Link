// ovr_test.cpp : This file contains the 'main' function. Program execution begins and ends there.
//


#include "definitions.h"
#include <fstream>
#include <mutex>

ovrSession mSession;

void vibration_thread(ovrSession mSession, int hand);
void main_loop(ovrSession mSession, HANDLE comm_mutex, shared_buffer* comm_buffer, uint64_t frame_count) {

    ovrTrackingState ss = ovr_GetTrackingState(mSession, 0, false);

    if (comm_buffer->logging_offset) {
        for (int i = 0; i < comm_buffer->logging_offset; i++) {
            putc(comm_buffer->logging_buffer[i], stdout);
        }
        comm_buffer->logging_offset = 0;
    }
    if (comm_buffer->vrEvent_type) {
        //std::cout << "VR Event 0x" << comm_buffer->vrEvent_type << std::endl;
        comm_buffer->vrEvent_type = 0;
    }

    if (comm_buffer->config.external_tracking) {
        comm_buffer->tracking_state = ovr_GetTrackingState(mSession, (ovr_GetTimeInSeconds() + (comm_buffer->config.extra_prediction_ms * 0.001)), ovrTrue);
    }

    for (int i = 0; i < comm_buffer->config.num_objects; i++) {
        ovrTrackedDeviceType deviceType = (ovrTrackedDeviceType)(ovrTrackedDevice_Object0 + i);
        ovrPoseStatef ovr_pose;

        ovr_GetDevicePoses(mSession, &deviceType, 1, (ovr_GetTimeInSeconds() + (comm_buffer->config.extra_prediction_ms * 0.001)), &ovr_pose);
        if ((ovr_pose.ThePose.Orientation.x != 0) || (ovr_pose.ThePose.Orientation.y != 0) || (ovr_pose.ThePose.Orientation.z != 0)){
            comm_buffer->object_poses[i] = ovr_pose;
        }
        if ((frame_count & 0x7FF) == 0) {
            std::cout.precision(4);

            std::cout << "Object" << i << std::dec << " x " <<
                ovr_pose.ThePose.Position.x << " y " <<
                ovr_pose.ThePose.Position.y << " z " <<
                ovr_pose.ThePose.Position.z << std::endl;
        }
    }

    for (int i = 0; i < comm_buffer->num_sensors; i++) comm_buffer->sensor_poses[i] = ovr_GetTrackerPose(mSession, i);

    ovrResult input_res[2];
    ovrInputState inputState[2];
    for (int i = 0; i < 2; i++) {

        if (i == 1) {
            input_res[i] = ovr_GetInputState(mSession, ovrControllerType::ovrControllerType_RTouch, &inputState[i]);
            comm_buffer->input_state.Buttons &= ~(ovrButton_RMask | ovrButton_Home);
            comm_buffer->input_state.Buttons |= ((ovrButton_RMask | ovrButton_Home) & inputState[i].Buttons);
            comm_buffer->input_state.Touches &= ~(ovrTouch_RButtonMask | ovrTouch_RPoseMask);
            comm_buffer->input_state.Touches |= ((ovrTouch_RButtonMask | ovrTouch_RPoseMask) & inputState[i].Touches);
        }
        else {
            input_res[i] = ovr_GetInputState(mSession, ovrControllerType::ovrControllerType_LTouch, &inputState[i]);
            comm_buffer->input_state.Buttons &= ~ovrButton_LMask;
            comm_buffer->input_state.Buttons |= (ovrButton_LMask & inputState[i].Buttons);
            comm_buffer->input_state.Touches &= ~(ovrTouch_LButtonMask | ovrTouch_LPoseMask);
            comm_buffer->input_state.Touches |= ((ovrTouch_LButtonMask | ovrTouch_LPoseMask) & inputState[i].Touches);
        }
        comm_buffer->input_state.HandTrigger[i] = inputState[i].HandTrigger[i];
        comm_buffer->input_state.IndexTrigger[i] = inputState[i].IndexTrigger[i];
        comm_buffer->input_state.Thumbstick[i] = inputState[i].Thumbstick[i];
    }

}


void no_graphics_start(shared_buffer* comm_buffer, HANDLE comm_mutex) {
    mSession = nullptr;
    //  ovrSession hmd2 = nullptr;
    ovrGraphicsLuid luid{};
    //  ovrGraphicsLuid luid2{};
    ovrInitParams initParams = { ovrInit_RequestVersion | ovrInit_FocusAware | ovrInit_Invisible, OVR_MINOR_VERSION, NULL, 0, 0 };
    /*ovrResult result = ovr_Initialize(&initParams);   */
    if (!mSession) {
#if 0
        if (OVR_FAILURE(ovr_Initialize(nullptr))) std::cout << "ovr_Initialize error" << std::endl;

        if (OVR_FAILURE(ovr_Create(&hmd2, &luid2)))  std::cout << "ovr_Create error" << std::endl;
#endif          
        if (OVR_FAILURE(ovr_Initialize(&initParams)/*ovr_Initialize(nullptr)*/)) {
            std::cout << "ovr_Initialize error" << std::endl;
            return;
        }

        if (OVR_FAILURE(ovr_Create(&mSession, &luid))) {
            std::cout << "ovr_Create error" << std::endl;
            return;
        }

        if (OVR_FAILURE(ovr_SetTrackingOriginType(mSession, ovrTrackingOrigin_FloorLevel))) {
            std::cout << "ovr_SetTrackingOriginType error" << std::endl;
            return;
        }
    }


    comm_buffer->config.num_objects = (ovr_GetConnectedControllerTypes(mSession) >> 8) & 0xf;
    comm_buffer->num_sensors = ovr_GetTrackerCount(mSession);

    std::thread vib_thread_l;
    vib_thread_l = std::thread(vibration_thread, mSession, 0);

    std::thread vib_thread_r;
    vib_thread_r = std::thread(vibration_thread, mSession, 1);

    // Main Loop
    uint64_t frame_count = 0;

    //ovr_RecenterTrackingOrigin(hmd);
    while (1) {
        ovrSessionStatus sessionStatus;
        ovr_GetSessionStatus(mSession, &sessionStatus);
        if (sessionStatus.ShouldQuit)
            break;

        main_loop(mSession, comm_mutex, comm_buffer, frame_count);

        frame_count++;
        Sleep(1);
    }
    ovr_Destroy(mSession);
    ovr_Shutdown();
}


shared_buffer* comm_buffer;


GUI_Manager* p_gui_manager = nullptr;

void reset_config_settings(config_data& config) {
    config.vr_universe = 1;
    config.be_objects = false;
    config.extra_prediction_ms = 0.0f;
    strncpy_s(comm_buffer->config.manufacturer_name, "Oculus", 127);
    strncpy_s(comm_buffer->config.tracking_space_name, "oculus_link", 127);
    comm_buffer->config.num_objects = (ovr_GetConnectedControllerTypes(mSession) >> 8) & 0xf;
    comm_buffer->num_sensors = ovr_GetTrackerCount(mSession);
    config.external_tracking = true;
    config.track_hmd = false;
    config.show_sensors_steam = true;
    config.disable_controllers = false;
    config.min_amplitude = 0;
    config.amplitude_scale = 1.0;
    config.sqrt_pre_filter = false;
    config.sqrt_post_filter = false;
    config.do_rendering = false;
    config.do_world_transformation = false;
    config.world_translation[0] = 0.0;
    config.world_translation[1] = 0.0;
    config.world_translation[2] = 0.0;
    config.world_orientation_q.w = 1.0;
    config.world_orientation_q.x = 0.0;
    config.world_orientation_q.y = 0.0;
    config.world_orientation_q.z = 0.0;
    config.world_orientation_euler[0] = 0.0;
    config.world_orientation_euler[1] = 0.0;
    config.world_orientation_euler[2] = 0.0;
}

void save_config_to_file(config_data& config) {
    char exe_filename[512];
    auto str_len = GetModuleFileNameA( NULL, (LPSTR) exe_filename, 512);
    std::cout << "exe_filename = " << exe_filename << std::endl;
    while (str_len && (exe_filename[str_len] != '\\')) str_len--;
    exe_filename[str_len] = '\0';
    std::cout << "exe_filepath = " << exe_filename << std::endl;
    char config_filename[] = "\\config.dat";
    for (size_t i = 0; i < sizeof(config_filename); i++) {
        exe_filename[str_len] = config_filename[i];
        str_len++;
    }
    exe_filename[str_len] = '\0';
    std::cout << "config filename = " << exe_filename << std::endl;

    std::ofstream ofs;
    ofs.open(exe_filename);

    ofs << config.vr_universe << std::endl;
    ofs << config.be_objects << std::endl;
    ofs << config.extra_prediction_ms << std::endl;
    ofs << config.tracking_space_name << std::endl;
    ofs << config.manufacturer_name << std::endl;
    ofs << config.num_objects << std::endl;
    ofs << config.external_tracking << std::endl;
    ofs << config.track_hmd << std::endl;
    ofs << config.show_sensors_steam << std::endl;
    ofs << (unsigned)config.min_amplitude << std::endl;
    ofs << config.amplitude_scale << std::endl;
    ofs << config.sqrt_pre_filter << std::endl;
    ofs << config.sqrt_post_filter << std::endl;
    ofs << config.do_rendering << std::endl;
    ofs << config.do_world_transformation << std::endl;
    ofs << config.world_translation[0] << std::endl;
    ofs << config.world_translation[1] << std::endl;
    ofs << config.world_translation[2] << std::endl;
    ofs << config.world_orientation_q.w << std::endl;
    ofs << config.world_orientation_q.x << std::endl;
    ofs << config.world_orientation_q.y << std::endl;
    ofs << config.world_orientation_q.z << std::endl;
    ofs << config.world_orientation_euler[0] << std::endl;
    ofs << config.world_orientation_euler[1] << std::endl;
    ofs << config.world_orientation_euler[2] << std::endl;
    ofs << config.disable_controllers << std::endl;
    ofs.close();
}


void load_config_from_file(config_data& config) {
    char exe_filename[512];
    auto str_len = GetModuleFileNameA(NULL, (LPSTR)exe_filename, 512);
    std::cout << "exe_filename = " << exe_filename << std::endl;
    while (str_len && (exe_filename[str_len] != '\\')) str_len--;
    exe_filename[str_len] = '\0';
    std::cout << "exe_filepath = " << exe_filename << std::endl;
    char config_filename[] = "\\config.dat";
    for (size_t i = 0; i < sizeof(config_filename); i++) {
        exe_filename[str_len] = config_filename[i];
        str_len++;
    }
    exe_filename[str_len] = '\0';
    std::cout << "config filename = " << exe_filename << std::endl;

    std::ifstream ifs;
    ifs.open(exe_filename);
    if (ifs.is_open()) {
        ifs >> config.vr_universe;
        ifs >> config.be_objects;
        ifs >> config.extra_prediction_ms;
        ifs >> config.tracking_space_name;
        ifs >> config.manufacturer_name;
        ifs >> config.num_objects;
        ifs >> config.external_tracking;
        ifs >> config.track_hmd;
        ifs >> config.show_sensors_steam;
        unsigned min_amp;
        ifs >> min_amp;
        config.min_amplitude = min_amp;
        
        ifs >> config.amplitude_scale;
        ifs >> config.sqrt_pre_filter;
        ifs >> config.sqrt_post_filter;
        ifs >> config.do_rendering;
        ifs >> config.do_world_transformation;
        ifs >> config.world_translation[0];
        ifs >> config.world_translation[1];
        ifs >> config.world_translation[2];
        ifs >> config.world_orientation_q.w;
        ifs >> config.world_orientation_q.x;
        ifs >> config.world_orientation_q.y;
        ifs >> config.world_orientation_q.z;
        ifs >> config.world_orientation_euler[0];
        ifs >> config.world_orientation_euler[1];
        ifs >> config.world_orientation_euler[2];

        ifs >> config.disable_controllers;
        ifs.close();
    }
}

int main(int argc, char** argsv)
{
    std::cout << "Welcome to oculus_touch_link, this program provides the input and haptic link to the stream driver, as well as passing confuguration data" << std::endl;
    std::cout << "You can provide this program with arguments to specify:" << std::endl;
    std::cout << "Render to Oculus headset y/n  (\"n\" must be use with ovr_dummy.exe)" << std::endl;
    std::cout << "VR \"Universe\" ID, 0=Invalid, 1=Oculus, 31=Recommended with VirtualDesktop/ALVR" << std::endl;
    std::cout << "Manufacturer name: Oculus or Oculus_link (or HTC for trackers)" << std::endl;
    std::cout << "Tracking system name: oculus or oculus_link (or lighthouse for trackers)" << std::endl;
    std::cout << "perform tracking prediction manually in-driver (n= ask oculus do the prediction) y/n" << std::endl;
    std::cout << "Extra prediction time (ms) for example 11.1 for 1 frame at 90fps" << std::endl;
    std::cout << "All controllers are tracked objects instead of controllers y/n" << std::endl;
    std::cout << "Perform tracking in ovr_test instead of steamvr driver y/n" << std::endl;
    std::cout << "Track the headset as a tracking object y/n" << std::endl;
    std::cout << "Minumim haptic amplitude 0-255 (64)" << std::endl;
    std::cout << "haptic scale multiplier 0-inf (1.0)" << std::endl;
    std::cout << "Show Oculus Vr sensors in steam?  y/n" << std::endl;
    std::cout << "" << std::endl;
    std::cout << "This program is super dumb and expects all of the arguments or none (for defaults), suggested invocations:" << std::endl;
    std::cout << "ovr_test.exe n 1 Oculus oculus n 10 n n 64 1.0(must be use with ovr_dummy.exe)" << std::endl;
    std::cout << "ovr_test.exe y 1 Oculus oculus y 10 n n 64 1.0" << std::endl;
    std::cout << "ovr_test.exe y 31 Oculus_link oculus_link n 10 n y n 64 1.0" << std::endl;
    std::cout << "ovr_test.exe n 31 Oculus_link oculus_link n 10 n n y 64 1.0(default)" << std::endl;


    HANDLE hMapFile;
 
#if 1
    hMapFile = CreateFileMapping(
        INVALID_HANDLE_VALUE,    // use paging file
        NULL,                    // default security
        PAGE_READWRITE,          // read/write access
        0,                       // maximum object size (high-order DWORD)
        sizeof(shared_buffer),                // maximum object size (low-order DWORD)
        L"Local\\oculus_steamvr_touch_controller_data_channel");                 // name of mapping object
#else
    hMapFile = OpenFileMapping(
        FILE_MAP_ALL_ACCESS,   // read/write access
        FALSE,                 // do not inherit the name
        L"Global\\oculus_steamvr_touch_controller_data_channel");               // name of mapping object
#endif
    if (hMapFile == NULL)
    {
        std::cout << "Could not open file mapping object " << GetLastError() << std::endl;
        return -1;
    }

    comm_buffer = new(MapViewOfFile(hMapFile, // handle to map object
        FILE_MAP_ALL_ACCESS,  // read/write permission
        0,
        0,
        sizeof(shared_buffer)))shared_buffer();


    if (comm_buffer == NULL)
    {
        std::cout << "Could not map view of file " << GetLastError() << std::endl;

        CloseHandle(hMapFile);

        return -1;
    }
    comm_buffer->vib_buffers[0].reset();
    comm_buffer->vib_buffers[1].reset();
    reset_config_settings(comm_buffer->config);
    if (argc != 12) {
        std::cout << " <12 arguments, using defaults: n 31 Oculus_link oculus_link n 5 n n y n" << std::endl;
        load_config_from_file(comm_buffer->config);
    } else {
        comm_buffer->config.do_rendering = (std::string(argsv[1]) == "y");
        comm_buffer->config.vr_universe = atoi(argsv[2]);
        strncpy_s(comm_buffer->config.manufacturer_name, argsv[3], 127);
        strncpy_s(comm_buffer->config.tracking_space_name, argsv[4], 127);
        comm_buffer->config.extra_prediction_ms = atof(argsv[6]);
        comm_buffer->config.be_objects = (std::string(argsv[7]) == "y");
        comm_buffer->config.external_tracking = (std::string(argsv[8]) == "y");
        comm_buffer->config.track_hmd = (std::string(argsv[9]) == "y");
        comm_buffer->config.min_amplitude = strtoul(argsv[10],0, 10);
        comm_buffer->config.amplitude_scale = strtof(argsv[11],0);
        comm_buffer->config.show_sensors_steam = (std::string(argsv[12]) == "y");
    }

    HANDLE comm_mutex = CreateMutex(0, true, L"Local\\oculus_steamvr_touch_controller_mutex");
    //MessageBox(NULL, pBuf, TEXT("Process2"), MB_OK);


       
    std::thread gui_thread([&]() {
        p_gui_manager = new GUI_Manager(comm_buffer);
        p_gui_manager->handle_loop();
        });
   
#if 1
    if (comm_buffer->config.do_rendering) {
        GuardianSystemDemo* instance = new (_aligned_malloc(sizeof(GuardianSystemDemo), 16)) GuardianSystemDemo();
        instance->Start(0, comm_buffer, comm_mutex);
        delete instance;

    } else {
        no_graphics_start(comm_buffer, comm_mutex);
    }
#endif
    gui_thread.join();

    UnmapViewOfFile(comm_buffer);

    CloseHandle(hMapFile);
    CloseHandle(comm_mutex);
    return 0;
}

uint8_t clamp_scale(uint8_t sample, float amplitude) {
    float scale = amplitude;
    uint64_t output = static_cast<uint64_t> ( static_cast<float>(sample) * scale );
    if (output > 255) output = 255;
    return static_cast<uint8_t>(output & 0xFF);
}


constexpr auto OVR_HAPTIC_SAMPLES = 40; // desc.QueueMinSizeToAvoidStarvation + desc.SubmitOptimalSamples

void vibration_thread(ovrSession mSession, int hand) {
    unsigned int vib_interval = 0;
    float vib_amplitude = 0;
    double sample_end_time = 0;
    unsigned char buf[OVR_HAPTIC_SAMPLES];
    ovrHapticsBuffer vibuffer;
    vibuffer.SamplesCount = 8;
    vibuffer.Samples = buf;
    vibuffer.SubmitMode = ovrHapticsBufferSubmit_Enqueue;
    unsigned int last_vib_interval = 0;
    unsigned int vib_interval_counter = 0;
    ovrTouchHapticsDesc desc = ovr_GetTouchHapticsDesc(mSession, ovrControllerType_LTouch);
    while (1) {

        Sleep(1);
        vib_sample s;
        double now = ovr_GetTimeInSeconds();
        while (comm_buffer->vib_buffers[hand].pop(s)) {
            // stolen from ALVR
            comm_buffer->vib_buffers[hand];
            float duration = s.duration;
            float frequency = s.frequency;
            float amplitude = s.amplitude;
            float m_hapticsIntensity = 1;
            float m_hapticsAmplitudeCurve = 0.4;
            float m_hapticsMinDuration = 0.01;
            float m_hapticsLowDurationAmplitudeMultiplier = 2.5;
            float m_hapticsLowDurationRange = 0.5;
            if (duration < m_hapticsMinDuration * 0.5) duration = m_hapticsMinDuration * 0.5;
            amplitude = pow(amplitude * ((m_hapticsLowDurationAmplitudeMultiplier - 1.0f) * m_hapticsMinDuration * m_hapticsLowDurationRange / (pow(m_hapticsMinDuration * m_hapticsLowDurationRange, 2.0f) * 0.25f / (duration - 0.5f * m_hapticsMinDuration * (1.0f - m_hapticsLowDurationRange)) + (duration - 0.5f * m_hapticsMinDuration * (1.0f - m_hapticsLowDurationRange))) + 1.0f), 1.0f - m_hapticsAmplitudeCurve);
            duration = pow(m_hapticsMinDuration, 2.0f) * 0.25f / duration + duration;
            float amp = amplitude;
            float freq = frequency; // * 320.0f;
            uint32_t requested_duration = duration * 320; // 320 Hz processing rate? seems to be 160
            if (requested_duration < 1) requested_duration = 1;
            uint32_t min_duration = 1;
            min_duration = 320 / freq;
            if (min_duration < 1) min_duration = 1;
            if (min_duration > 2) min_duration = 2; // low duty cycle sucks
            if (min_duration * 2 > requested_duration) min_duration = 1; // short pulses are "max frequency"

//            sample_end_time = now + duration;
            sample_end_time = duration;
            vib_amplitude = amp;
            vib_interval = min_duration;
            vib_interval_counter = 0;
        }
//        double time_left = sample_end_time - now;
        double time_left = sample_end_time;
        float amplitude = vib_amplitude;
        if (time_left <= 0) {
            vib_interval_counter = 0;
            time_left = 0;
        }
        if (last_vib_interval != vib_interval) {
            last_vib_interval = vib_interval;
            vib_interval_counter = 0;
        }

        ovrHapticsPlaybackState pbState;
        ovr_GetControllerVibrationState(mSession, (hand == 0) ? ovrControllerType_LTouch : ovrControllerType_RTouch, &pbState);

        // somehow need to keep buffer filled, but it also does not like being "full?" which causes massive latency..
        // smaller buffer lowers latency, larger buffer also lowers latency :P
        uint32_t need_count = 0;
        if (pbState.SamplesQueued < OVR_HAPTIC_SAMPLES) need_count = OVR_HAPTIC_SAMPLES - pbState.SamplesQueued;
        uint64_t samples_remaining = lround(time_left * desc.SampleRateHz);
        if (need_count >= desc.SubmitOptimalSamples) {
            for (unsigned int i = 0; i < need_count; i++) {
                sample_end_time -= 1.0 / desc.SampleRateHz;
                unsigned int amp = clamp_scale(255, amplitude);
                if (i >= samples_remaining) {
                    buf[i] = 0;
                }
                else if (vib_interval_counter == 0) {
                    buf[i] = amp;
                    if (last_vib_interval > 1) {
                        vib_interval_counter = last_vib_interval - 1;
                    }
                }
                else {
                    vib_interval_counter--;
                    buf[i] = 0;
                }
            }
            vibuffer.Samples = buf;
            vibuffer.SamplesCount = need_count;
            vibuffer.SubmitMode = ovrHapticsBufferSubmit_Enqueue;
            ovr_SubmitControllerVibration(mSession, (hand == 0) ? ovrControllerType_LTouch : ovrControllerType_RTouch, &vibuffer);
        }
    }
}