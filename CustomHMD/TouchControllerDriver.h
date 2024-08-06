#pragma once

#include "Common.h"

//-----------------------------------------------------------------------------
// Purpose:
//-----------------------------------------------------------------------------
class CTouchControllerDriver : public vr::ITrackedDeviceServerDriver
{                                                                                                                                             // hand_offset({ 0.01071,0.04078,-0.04731 }), hand_offset2({-0.003,-0.101,0.0089 })
public:                                                                                                                                      //x = 0.00571 y = 0.04078 z = -0.03531 x2 =-0.000999998 y2 = -0.1 z = 0.0019
    CTouchControllerDriver(ovrSession mSession, bool isRightHand/*, ovrVector3f overall_offset, ovrQuatf overall_rotation*/) :
        mSession(mSession), isRightHand(isRightHand)
        , hand_offset({ 0.00571,0.04078,-0.03531 })
        , hand_offset2({ -0.000999998,-0.1, 0.0019 }
        )/*, overall_offset(overall_offset), overall_rotation(overall_rotation)*/
    {
        m_unObjectId = vr::k_unTrackedDeviceIndexInvalid;
        m_ulPropertyContainer = vr::k_ulInvalidPropertyContainer;
        if (isRightHand) {
            m_sSerialNumber = "ODT-00000003";
            m_sModelNumber = "Oculus Rift CV1(Right Controller)";
        }
        else {
            m_sSerialNumber = "ODT-00000004";
            m_sModelNumber = "Oculus Rift CV1(Left Controller)";
        }
        log_to_buffer(__func__);
    }

    virtual ~CTouchControllerDriver()
    {
    }


    virtual EVRInitError Activate(vr::TrackedDeviceIndex_t unObjectId)
    {
        log_to_buffer(__func__);

        m_unObjectId = unObjectId;
        m_ulPropertyContainer = vr::VRProperties()->TrackedDeviceToPropertyContainer(m_unObjectId);
        /** initializes the driver. This will be called before any other methods are called.
        * If Init returns anything other than VRInitError_None the driver DLL will be unloaded.
        *
        * pDriverHost will never be NULL, and will always be a pointer to a IServerDriverHost interface
        *
        * pchUserDriverConfigDir - The absolute path of the directory where the driver should store user
        *	config files.
        * pchDriverInstallDir - The absolute path of the root directory for the driver.
        */
  

        vr::VRProperties()->SetUint64Property(m_ulPropertyContainer, Prop_CurrentUniverseId_Uint64, comm_buffer->config.vr_universe);

        vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, vr::Prop_ModelNumber_String, m_sModelNumber.c_str());
        vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, vr::Prop_SerialNumber_String, m_sSerialNumber.c_str());

        vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, vr::Prop_TrackingSystemName_String, comm_buffer->config.tracking_space_name);
        vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, vr::Prop_ManufacturerName_String, comm_buffer->config.manufacturer_name);

        vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, vr::Prop_HardwareRevision_String, "14");
        vr::VRProperties()->SetUint64Property(m_ulPropertyContainer, vr::Prop_HardwareRevision_Uint64, 14U);
        //vr::VRProperties()->SetUint64Property(m_ulPropertyContainer, vr::Prop_ParentDriver_Uint64, 8589934599U); // Strange value from dump
        vr::VRProperties()->SetBoolProperty(m_ulPropertyContainer, vr::Prop_HasDisplayComponent_Bool, false);
        vr::VRProperties()->SetBoolProperty(m_ulPropertyContainer, vr::Prop_HasCameraComponent_Bool, false);
        vr::VRProperties()->SetBoolProperty(m_ulPropertyContainer, vr::Prop_HasDriverDirectModeComponent_Bool, false);
        vr::VRProperties()->SetBoolProperty(m_ulPropertyContainer, vr::Prop_HasVirtualDisplayComponent_Bool, false);

        if (comm_buffer->config.be_objects) {
            VRProperties()->SetStringProperty(m_ulPropertyContainer, Prop_RenderModelName_String, "{htc}vr_tracker_vive_1_0");
            vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, vr::Prop_ModelNumber_String, "Vive Tracker Pro MV");
            vr::VRProperties()->SetInt32Property(m_ulPropertyContainer, vr::Prop_DeviceClass_Int32, vr::TrackedDeviceClass_GenericTracker);
            //vr::VRProperties()->SetUint64Property(m_ulPropertyContainer, vr::Prop_ParentDriver_Uint64, 8589934599U); // Strange value from dump
            vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, vr::Prop_ResourceRoot_String, "htc");

            vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, vr::Prop_RegisteredDeviceType_String, (isRightHand) ? "htc/vive_trackerLHR-OCULUS_RIGHT" : "htc/vive_trackerLHR-OCULUS_LEFT");

            vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, vr::Prop_InputProfilePath_String, "{htc}/input/vive_tracker_profile.json");
            // vr::VRProperties()->SetUint64Property(m_ulPropertyContainer, vr::Prop_SupportedButtons_Uint64, 30064771207U);
            vr::VRProperties()->SetInt32Property(m_ulPropertyContainer, vr::Prop_ControllerRoleHint_Int32, vr::TrackedControllerRole_Invalid);
            vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, vr::Prop_ControllerType_String, "vive_tracker");

            vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, vr::Prop_NamedIconPathDeviceOff_String, "{htc}/icons/tracker_status_off.png");
            vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, vr::Prop_NamedIconPathDeviceSearching_String, "{htc}/icons/tracker_status_searching.gif");
            vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, vr::Prop_NamedIconPathDeviceSearchingAlert_String, "{htc}/icons/tracker_status_searching_alert.gif");
            vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, vr::Prop_NamedIconPathDeviceReady_String, "{htc}/icons/tracker_status_ready.png");
            vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, vr::Prop_NamedIconPathDeviceReadyAlert_String, "{htc}/icons/tracker_status_ready_alert.png");
            vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, vr::Prop_NamedIconPathDeviceNotReady_String, "{htc}/icons/tracker_status_error.png");
            vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, vr::Prop_NamedIconPathDeviceStandby_String, "{htc}/icons/tracker_status_standby.png");
            vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, vr::Prop_NamedIconPathDeviceAlertLow_String, "{htc}/icons/tracker_status_ready_low.png");
        } else {
            vr::VRProperties()->SetInt32Property(m_ulPropertyContainer, vr::Prop_DeviceClass_Int32, vr::TrackedDeviceClass_Controller);
            vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, vr::Prop_ResourceRoot_String, "oculus");

            vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, vr::Prop_RegisteredDeviceType_String, (isRightHand) ? "oculus/WMHD316J600000_Controller_Right" : "oculus/WMHD316J600000_Controller_Left");

            vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, vr::Prop_InputProfilePath_String, "{oculus}/input/touch_profile.json");
            vr::VRProperties()->SetUint64Property(m_ulPropertyContainer, vr::Prop_SupportedButtons_Uint64, 30064771207U);
            vr::VRProperties()->SetInt32Property(m_ulPropertyContainer, vr::Prop_Axis0Type_Int32, vr::k_eControllerAxis_Joystick);
            vr::VRProperties()->SetInt32Property(m_ulPropertyContainer, vr::Prop_Axis1Type_Int32, vr::k_eControllerAxis_Trigger);
            vr::VRProperties()->SetInt32Property(m_ulPropertyContainer, vr::Prop_Axis2Type_Int32, vr::k_eControllerAxis_Trigger);
            vr::VRProperties()->SetInt32Property(m_ulPropertyContainer, vr::Prop_ControllerRoleHint_Int32, (isRightHand) ? vr::TrackedControllerRole_RightHand : vr::TrackedControllerRole_LeftHand);
            vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, vr::Prop_ControllerType_String, "oculus_touch");
            vr::VRProperties()->SetInt32Property(m_ulPropertyContainer, vr::Prop_ControllerHandSelectionPriority_Int32, 0);

            vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, vr::Prop_NamedIconPathDeviceOff_String, (!isRightHand) ? "{oculus}/icons/cv1_left_controller_off.png" : "{oculus}/icons/cv1_right_controller_off.png");
            vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, vr::Prop_NamedIconPathDeviceSearching_String, (!isRightHand) ? "{oculus}/icons/cv1_left_controller_searching.gif" : "{oculus}/icons/cv1_right_controller_searching.gif");
            vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, vr::Prop_NamedIconPathDeviceSearchingAlert_String, (!isRightHand) ? "{oculus}/icons/cv1_left_controller_searching_alert.gif" : "{oculus}/icons/cv1_right_controller_searching_alert.gif");
            vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, vr::Prop_NamedIconPathDeviceReady_String, (!isRightHand) ? "{oculus}/icons/cv1_left_controller_ready.png" : "{oculus}/icons/cv1_right_controller_ready.png");
            vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, vr::Prop_NamedIconPathDeviceReadyAlert_String, (!isRightHand) ? "{oculus}/icons/cv1_left_controller_ready_alert.png" : "{oculus}/icons/cv1_right_controller_ready_alert.png");
            vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, vr::Prop_NamedIconPathDeviceNotReady_String, (!isRightHand) ? "{oculus}/icons/cv1_left_controller_error.png" : "{oculus}/icons/cv1_right_controller_error.png");
            vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, vr::Prop_NamedIconPathDeviceStandby_String, (!isRightHand) ? "{oculus}/icons/cv1_left_controller_searching.gif" : "{oculus}/icons/cv1_right_controller_searching.gif");
            vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, vr::Prop_NamedIconPathDeviceAlertLow_String, (!isRightHand) ? "{oculus}/icons/cv1_left_controller_ready_low.png" : "{oculus}/icons/cv1_right_controller_ready_low.png");
            vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, vr::Prop_NamedIconPathDeviceStandbyAlert_String, (!isRightHand) ? "{oculus}/icons/cv1_left_controller_standby.png" : "{oculus}/icons/cv1_right_controller_standby.png");


            ovrHmdDesc hmd_desc;
            if ((!comm_buffer->config.external_tracking) && mSession) {
                hmd_desc = ovr_GetHmdDesc(mSession);
            } else {
                hmd_desc.Type = ovrHmd_CV1;
            }
            switch (hmd_desc.Type)
            {
            case ovrHmd_CV1:
                if (isRightHand) {
                    VRProperties()->SetStringProperty(m_ulPropertyContainer, Prop_RenderModelName_String, "oculus_cv1_controller_right");
                }
                else {
                    VRProperties()->SetStringProperty(m_ulPropertyContainer, Prop_RenderModelName_String, "oculus_cv1_controller_left");
                }
                break;
            case ovrHmd_RiftS:
                if (isRightHand) {
                    VRProperties()->SetStringProperty(m_ulPropertyContainer, Prop_RenderModelName_String, "oculus_rifts_controller_right");
                }
                else {
                    VRProperties()->SetStringProperty(m_ulPropertyContainer, Prop_RenderModelName_String, "oculus_rifts_controller_left");
                }
            case ovrHmd_Quest:
                if (isRightHand) {
                    VRProperties()->SetStringProperty(m_ulPropertyContainer, Prop_RenderModelName_String, "oculus_quest_controller_right");
                }
                else {
                    VRProperties()->SetStringProperty(m_ulPropertyContainer, Prop_RenderModelName_String, "oculus_quest_controller_left");
                }
            case ovrHmd_Quest2:
                if (isRightHand) {
                    VRProperties()->SetStringProperty(m_ulPropertyContainer, Prop_RenderModelName_String, "oculus_quest2_controller_right");
                }
                else {
                    VRProperties()->SetStringProperty(m_ulPropertyContainer, Prop_RenderModelName_String, "oculus_quest2_controller_left");
                }
            default:
                if (isRightHand) {
                    VRProperties()->SetStringProperty(m_ulPropertyContainer, Prop_RenderModelName_String, "oculus_cv1_controller_right");
                }
                else {
                    VRProperties()->SetStringProperty(m_ulPropertyContainer, Prop_RenderModelName_String, "oculus_cv1_controller_left");
                }
            }
   
            // Register our input bindings
            if (isRightHand) {
                vr::VRDriverInput()->CreateBooleanComponent(m_ulPropertyContainer, "/input/a/click", &m_compAc);
                vr::VRDriverInput()->CreateBooleanComponent(m_ulPropertyContainer, "/input/b/click", &m_compBc);
                vr::VRDriverInput()->CreateBooleanComponent(m_ulPropertyContainer, "/input/a/touch", &m_compAt);
                vr::VRDriverInput()->CreateBooleanComponent(m_ulPropertyContainer, "/input/b/touch", &m_compBt);
                vr::VRDriverInput()->CreateSkeletonComponent(m_ulPropertyContainer, "/input/skeleton/right", "/skeleton/hand/right", "/pose/raw", VRSkeletalTracking_Estimated, nullptr, 0, &m_compSkel);
            }
            else {
                vr::VRDriverInput()->CreateBooleanComponent(m_ulPropertyContainer, "/input/x/click", &m_compXc);
                vr::VRDriverInput()->CreateBooleanComponent(m_ulPropertyContainer, "/input/y/click", &m_compYc);
                vr::VRDriverInput()->CreateBooleanComponent(m_ulPropertyContainer, "/input/x/touch", &m_compXt);
                vr::VRDriverInput()->CreateBooleanComponent(m_ulPropertyContainer, "/input/y/touch", &m_compYt);
                vr::VRDriverInput()->CreateSkeletonComponent(m_ulPropertyContainer, "/input/skeleton/left", "/skeleton/hand/left", "/pose/raw", VRSkeletalTracking_Estimated, nullptr, 0, &m_compSkel);
            }

            vr::VRDriverInput()->CreateBooleanComponent(m_ulPropertyContainer, "/input/system/click", &m_compSysc);
            vr::VRDriverInput()->CreateBooleanComponent(m_ulPropertyContainer, "/input/system/touch", &m_compSyst);
            vr::VRDriverInput()->CreateScalarComponent(m_ulPropertyContainer, "/input/grip/value", &m_compGripv, VRScalarType_Absolute, VRScalarUnits_NormalizedOneSided);
            vr::VRDriverInput()->CreateBooleanComponent(m_ulPropertyContainer, "/input/grip/button", &m_compGripb);
            vr::VRDriverInput()->CreateBooleanComponent(m_ulPropertyContainer, "/input/grip/click", &m_compGripc);
            vr::VRDriverInput()->CreateBooleanComponent(m_ulPropertyContainer, "/input/grip/touch", &m_compGript);
            vr::VRDriverInput()->CreateScalarComponent(m_ulPropertyContainer, "/input/trigger/value", &m_compTrigv, VRScalarType_Absolute, VRScalarUnits_NormalizedOneSided);
            vr::VRDriverInput()->CreateBooleanComponent(m_ulPropertyContainer, "/input/trigger/click", &m_compTrigc);
            vr::VRDriverInput()->CreateBooleanComponent(m_ulPropertyContainer, "/input/trigger/touch", &m_compTrigt);
            vr::VRDriverInput()->CreateScalarComponent(m_ulPropertyContainer, "/input/joystick/x", &m_compJoyx, VRScalarType_Absolute, VRScalarUnits_NormalizedTwoSided);
            vr::VRDriverInput()->CreateScalarComponent(m_ulPropertyContainer, "/input/joystick/y", &m_compJoyy, VRScalarType_Absolute, VRScalarUnits_NormalizedTwoSided);
            vr::VRDriverInput()->CreateBooleanComponent(m_ulPropertyContainer, "/input/joystick/click", &m_compJoyc);
            vr::VRDriverInput()->CreateBooleanComponent(m_ulPropertyContainer, "/input/joystick/touch", &m_compJoyt);
            vr::VRDriverInput()->CreateBooleanComponent(m_ulPropertyContainer, "/input/thumbrest/touch", &m_compThumbt);



            // create our haptic component
            vr::VRDriverInput()->CreateHapticComponent(m_ulPropertyContainer, "/output/haptic", &m_compHaptic);
        }




        return VRInitError_None;
    }
    virtual void Deactivate()
    {
        m_unObjectId = vr::k_unTrackedDeviceIndexInvalid;
    }

    virtual void EnterStandby()
    {
    }

    void* GetComponent(const char* pchComponentNameAndVersion)
    {
        // override this to add a component to a driver
        return NULL;
    }

    virtual void PowerOff()
    {
    }

    /** debug request from a client */
    virtual void DebugRequest(const char* pchRequest, char* pchResponseBuffer, uint32_t unResponseBufferSize)
    {
        if (unResponseBufferSize >= 1)
            pchResponseBuffer[0] = 0;
    }





    inline vr::HmdVector4_t Lerp(vr::HmdVector4_t& v1, vr::HmdVector4_t& v2, double lambda)
    {
        vr::HmdVector4_t res;
        res.v[0] = (float)((1 - lambda) * v1.v[0] + lambda * v2.v[0]);
        res.v[1] = (float)((1 - lambda) * v1.v[1] + lambda * v2.v[1]);
        res.v[2] = (float)((1 - lambda) * v1.v[2] + lambda * v2.v[2]);
        res.v[3] = 1;

        return res;
    }

    inline vr::HmdQuaternionf_t Slerp(vr::HmdQuaternionf_t& q1, vr::HmdQuaternionf_t& q2, double lambda)
    {
        if (q1.w != q2.w || q1.x != q2.x || q1.y != q2.y || q1.z != q2.z) {
            float dotproduct = q1.x * q2.x + q1.y * q2.y + q1.z * q2.z + q1.w * q2.w;
            float theta, st, sut, sout, coeff1, coeff2;

            // algorithm adapted from Shoemake's paper

            theta = (float)acos(dotproduct);
            if (theta < 0.0) theta = -theta;

            st = (float)sin(theta);
            sut = (float)sin(lambda * theta);
            sout = (float)sin((1 - lambda) * theta);
            coeff1 = sout / st;
            coeff2 = sut / st;

            vr::HmdQuaternionf_t res;
            res.w = coeff1 * q1.w + coeff2 * q2.w;
            res.x = coeff1 * q1.x + coeff2 * q2.x;
            res.y = coeff1 * q1.y + coeff2 * q2.y;
            res.z = coeff1 * q1.z + coeff2 * q2.z;

            float norm = res.w * res.w + res.x * res.x + res.y * res.y + res.z * res.z;
            res.w /= norm;
            res.x /= norm;
            res.y /= norm;
            res.z /= norm;

            return res;
        }
        else {
            return q1;
        }
    }






void GetThumbBoneTransform(bool withController,
                           bool isLeftHand,
                           bool touch,
                           vr::VRBoneTransform_t outBoneTransform[]) {
    if (isLeftHand) {
        if (touch) {
            if (withController) {
                outBoneTransform[2] = {{-0.017303f, 0.032567f, 0.025281f, 1.f},
                                       {0.317609f, 0.528344f, 0.213134f, 0.757991f}};
                outBoneTransform[3] = {{0.040406f, 0.000000f, -0.000000f, 1.f},
                                       {0.991742f, 0.085317f, 0.019416f, 0.093765f}};
                outBoneTransform[4] = {{0.032517f, -0.000000f, 0.000000f, 1.f},
                                       {0.959385f, -0.012202f, -0.031055f, 0.280120f}};
            } else {
                outBoneTransform[2] = {{-0.016426f, 0.030866f, 0.025118f, 1.f},
                                       {0.403850f, 0.595704f, 0.082451f, 0.689380f}};
                outBoneTransform[3] = {{0.040406f, 0.000000f, -0.000000f, 1.f},
                                       {0.989655f, -0.090426f, 0.028457f, 0.107691f}};
                outBoneTransform[4] = {{0.032517f, 0.000000f, 0.000000f, 1.f},
                                       {0.988590f, 0.143978f, 0.041520f, 0.015363f}};
            }
        } else {
            outBoneTransform[2] = {{-0.012083f, 0.028070f, 0.025050f, 1},
                                   {0.464112f, 0.567418f, 0.272106f, 0.623374f}};
            outBoneTransform[3] = {{0.040406f, 0.000000f, -0.000000f, 1},
                                   {0.994838f, 0.082939f, 0.019454f, 0.055130f}};
            outBoneTransform[4] = {{0.032517f, 0.000000f, 0.000000f, 1},
                                   {0.974793f, -0.003213f, 0.021867f, -0.222015f}};
        }

        outBoneTransform[5] = {{0.030464f, -0.000000f, -0.000000f, 1},
                               {1.000000f, -0.000000f, 0.000000f, 0.000000f}};
    } else {
        if (touch) {
            if (withController) {
                outBoneTransform[2] = {{0.017303f, 0.032567f, 0.025281f, 1},
                                       {0.528344f, -0.317609f, 0.757991f, -0.213134f}};
                outBoneTransform[3] = {{-0.040406f, -0.000000f, 0.000000f, 1},
                                       {0.991742f, 0.085317f, 0.019416f, 0.093765f}};
                outBoneTransform[4] = {{-0.032517f, 0.000000f, -0.000000f, 1},
                                       {0.959385f, -0.012202f, -0.031055f, 0.280120f}};
            } else {
                outBoneTransform[2] = {{0.016426f, 0.030866f, 0.025118f, 1},
                                       {0.595704f, -0.403850f, 0.689380f, -0.082451f}};
                outBoneTransform[3] = {{-0.040406f, -0.000000f, 0.000000f, 1},
                                       {0.989655f, -0.090426f, 0.028457f, 0.107691f}};
                outBoneTransform[4] = {{-0.032517f, -0.000000f, -0.000000f, 1},
                                       {0.988590f, 0.143978f, 0.041520f, 0.015363f}};
            }
        } else {
            outBoneTransform[2] = {{0.012330f, 0.028661f, 0.025049f, 1},
                                   {0.571059f, -0.451277f, 0.630056f, -0.270685f}};
            outBoneTransform[3] = {{-0.040406f, -0.000000f, 0.000000f, 1},
                                   {0.994565f, 0.078280f, 0.018282f, 0.066177f}};
            outBoneTransform[4] = {{-0.032517f, -0.000000f, -0.000000f, 1},
                                   {0.977658f, -0.003039f, 0.020722f, -0.209156f}};
        }

        outBoneTransform[5] = {{-0.030464f, 0.000000f, 0.000000f, 1},
                               {1.000000f, -0.000000f, 0.000000f, 0.000000f}};
    }
}

void GetTriggerBoneTransform(bool withController,
                             bool isLeftHand,
                             bool touch,
                             float click,
                             vr::VRBoneTransform_t outBoneTransform[]) {
    if (click > 0.001) {
        if (withController) {
            if (isLeftHand) {
                outBoneTransform[6] = {{-0.003925f, 0.027171f, 0.014640f, 1},
                                       {0.666448f, 0.430031f, -0.455947f, 0.403772f}};
                outBoneTransform[7] = {{0.076015f, -0.005124f, 0.000239f, 1},
                                       {-0.956011f, -0.000025f, 0.158355f, -0.246913f}};
                outBoneTransform[8] = {{0.043930f, -0.000000f, -0.000000f, 1},
                                       {-0.944138f, -0.043351f, 0.014947f, -0.326345f}};
                outBoneTransform[9] = {{0.028695f, 0.000000f, 0.000000f, 1},
                                       {-0.912149f, 0.003626f, 0.039888f, -0.407898f}};
                outBoneTransform[10] = {{0.022821f, 0.000000f, -0.000000f, 1},
                                        {1.000000f, -0.000000f, -0.000000f, 0.000000f}};
                outBoneTransform[11] = {{0.002177f, 0.007120f, 0.016319f, 1},
                                        {0.529359f, 0.540512f, -0.463783f, 0.461011f}};
                outBoneTransform[12] = {{0.070953f, 0.000779f, 0.000997f, 1},
                                        {0.847397f, -0.257141f, -0.139135f, 0.443213f}};
                outBoneTransform[13] = {{0.043108f, 0.000000f, 0.000000f, 1},
                                        {0.874907f, 0.009875f, 0.026584f, 0.483460f}};
                outBoneTransform[14] = {{0.033266f, -0.000000f, 0.000000f, 1},
                                        {0.894578f, -0.036774f, -0.050597f, 0.442513f}};
                outBoneTransform[15] = {{0.025892f, -0.000000f, 0.000000f, 1},
                                        {0.999195f, -0.000000f, 0.000000f, 0.040126f}};
                outBoneTransform[16] = {{0.000513f, -0.006545f, 0.016348f, 1},
                                        {0.500244f, 0.530784f, -0.516215f, 0.448939f}};
                outBoneTransform[17] = {{0.065876f, 0.001786f, 0.000693f, 1},
                                        {0.831617f, -0.242931f, -0.139695f, 0.479461f}};
                outBoneTransform[18] = {{0.040697f, 0.000000f, 0.000000f, 1},
                                        {0.769163f, -0.001746f, 0.001363f, 0.639049f}};
                outBoneTransform[19] = {{0.028747f, -0.000000f, -0.000000f, 1},
                                        {0.968615f, -0.064538f, -0.046586f, 0.235477f}};
                outBoneTransform[20] = {{0.022430f, -0.000000f, 0.000000f, 1},
                                        {1.000000f, 0.000000f, -0.000000f, -0.000000f}};
                outBoneTransform[21] = {{-0.002478f, -0.018981f, 0.015214f, 1},
                                        {0.474671f, 0.434670f, -0.653212f, 0.398827f}};
                outBoneTransform[22] = {{0.062878f, 0.002844f, 0.000332f, 1},
                                        {0.798788f, -0.199577f, -0.094418f, 0.559636f}};
                outBoneTransform[23] = {{0.030220f, 0.000002f, -0.000000f, 1},
                                        {0.853087f, 0.001644f, -0.000913f, 0.521765f}};
                outBoneTransform[24] = {{0.018187f, -0.000002f, 0.000000f, 1},
                                        {0.974249f, 0.052491f, 0.003591f, 0.219249f}};
                outBoneTransform[25] = {{0.018018f, 0.000000f, -0.000000f, 1},
                                        {1.000000f, 0.000000f, 0.000000f, 0.000000f}};
                outBoneTransform[26] = {{0.006629f, 0.026690f, 0.061870f, 1},
                                        {0.805084f, -0.018369f, 0.584788f, -0.097597f}};
                outBoneTransform[27] = {{-0.007882f, -0.040478f, 0.039337f, 1},
                                        {-0.322494f, 0.932092f, 0.121861f, 0.111140f}};
                outBoneTransform[28] = {{0.017136f, -0.032633f, 0.080682f, 1},
                                        {-0.169466f, 0.800083f, 0.571006f, 0.071415f}};
                outBoneTransform[29] = {{0.011144f, -0.028727f, 0.108366f, 1},
                                        {-0.076328f, 0.788280f, 0.605097f, 0.081527f}};
                outBoneTransform[30] = {{0.011333f, -0.026044f, 0.128585f, 1},
                                        {-0.144791f, 0.737451f, 0.656958f, -0.060069f}};
            } else {
                outBoneTransform[6] = {{-0.003925f, 0.027171f, 0.014640f, 1},
                                       {0.666448f, 0.430031f, -0.455947f, 0.403772f}};
                outBoneTransform[7] = {{0.076015f, -0.005124f, 0.000239f, 1},
                                       {-0.956011f, -0.000025f, 0.158355f, -0.246913f}};
                outBoneTransform[8] = {{0.043930f, -0.000000f, -0.000000f, 1},
                                       {-0.944138f, -0.043351f, 0.014947f, -0.326345f}};
                outBoneTransform[9] = {{0.028695f, 0.000000f, 0.000000f, 1},
                                       {-0.912149f, 0.003626f, 0.039888f, -0.407898f}};
                outBoneTransform[10] = {{0.022821f, 0.000000f, -0.000000f, 1},
                                        {1.000000f, -0.000000f, -0.000000f, 0.000000f}};
                outBoneTransform[11] = {{0.002177f, 0.007120f, 0.016319f, 1},
                                        {0.529359f, 0.540512f, -0.463783f, 0.461011f}};
                outBoneTransform[12] = {{0.070953f, 0.000779f, 0.000997f, 1},
                                        {0.847397f, -0.257141f, -0.139135f, 0.443213f}};
                outBoneTransform[13] = {{0.043108f, 0.000000f, 0.000000f, 1},
                                        {0.874907f, 0.009875f, 0.026584f, 0.483460f}};
                outBoneTransform[14] = {{0.033266f, -0.000000f, 0.000000f, 1},
                                        {0.894578f, -0.036774f, -0.050597f, 0.442513f}};
                outBoneTransform[15] = {{0.025892f, -0.000000f, 0.000000f, 1},
                                        {0.999195f, -0.000000f, 0.000000f, 0.040126f}};
                outBoneTransform[16] = {{0.000513f, -0.006545f, 0.016348f, 1},
                                        {0.500244f, 0.530784f, -0.516215f, 0.448939f}};
                outBoneTransform[17] = {{0.065876f, 0.001786f, 0.000693f, 1},
                                        {0.831617f, -0.242931f, -0.139695f, 0.479461f}};
                outBoneTransform[18] = {{0.040697f, 0.000000f, 0.000000f, 1},
                                        {0.769163f, -0.001746f, 0.001363f, 0.639049f}};
                outBoneTransform[19] = {{0.028747f, -0.000000f, -0.000000f, 1},
                                        {0.968615f, -0.064538f, -0.046586f, 0.235477f}};
                outBoneTransform[20] = {{0.022430f, -0.000000f, 0.000000f, 1},
                                        {1.000000f, 0.000000f, -0.000000f, -0.000000f}};
                outBoneTransform[21] = {{-0.002478f, -0.018981f, 0.015214f, 1},
                                        {0.474671f, 0.434670f, -0.653212f, 0.398827f}};
                outBoneTransform[22] = {{0.062878f, 0.002844f, 0.000332f, 1},
                                        {0.798788f, -0.199577f, -0.094418f, 0.559636f}};
                outBoneTransform[23] = {{0.030220f, 0.000002f, -0.000000f, 1},
                                        {0.853087f, 0.001644f, -0.000913f, 0.521765f}};
                outBoneTransform[24] = {{0.018187f, -0.000002f, 0.000000f, 1},
                                        {0.974249f, 0.052491f, 0.003591f, 0.219249f}};
                outBoneTransform[25] = {{0.018018f, 0.000000f, -0.000000f, 1},
                                        {1.000000f, 0.000000f, 0.000000f, 0.000000f}};
                outBoneTransform[26] = {{0.006629f, 0.026690f, 0.061870f, 1},
                                        {0.805084f, -0.018369f, 0.584788f, -0.097597f}};
                outBoneTransform[27] = {{-0.007882f, -0.040478f, 0.039337f, 1},
                                        {-0.322494f, 0.932092f, 0.121861f, 0.111140f}};
                outBoneTransform[28] = {{0.017136f, -0.032633f, 0.080682f, 1},
                                        {-0.169466f, 0.800083f, 0.571006f, 0.071415f}};
                outBoneTransform[29] = {{0.011144f, -0.028727f, 0.108366f, 1},
                                        {-0.076328f, 0.788280f, 0.605097f, 0.081527f}};
                outBoneTransform[30] = {{0.011333f, -0.026044f, 0.128585f, 1},
                                        {-0.144791f, 0.737451f, 0.656958f, -0.060069f}};
            }
        } else {
            if (isLeftHand) {
                outBoneTransform[6] = {{0.003802f, 0.021514f, 0.012803f, 1},
                                       {0.617314f, 0.395175f, -0.510874f, 0.449185f}};
                outBoneTransform[7] = {{0.074204f, -0.005002f, 0.000234f, 1},
                                       {0.737291f, -0.032006f, -0.115013f, 0.664944f}};
                outBoneTransform[8] = {{0.043287f, -0.000000f, -0.000000f, 1},
                                       {0.611381f, 0.003287f, 0.003823f, 0.791320f}};
                outBoneTransform[9] = {{0.028275f, 0.000000f, 0.000000f, 1},
                                       {0.745389f, -0.000684f, -0.000945f, 0.666629f}};
                outBoneTransform[10] = {{0.022821f, 0.000000f, -0.000000f, 1},
                                        {1.000000f, 0.000000f, -0.000000f, 0.000000f}};
                outBoneTransform[11] = {{0.004885f, 0.006885f, 0.016480f, 1},
                                        {0.522678f, 0.527374f, -0.469333f, 0.477923f}};
                outBoneTransform[12] = {{0.070953f, 0.000779f, 0.000997f, 1},
                                        {0.826071f, -0.121321f, 0.017267f, 0.550082f}};
                outBoneTransform[13] = {{0.043108f, 0.000000f, 0.000000f, 1},
                                        {0.956676f, 0.013210f, 0.009330f, 0.290704f}};
                outBoneTransform[14] = {{0.033266f, 0.000000f, 0.000000f, 1},
                                        {0.979740f, -0.001605f, -0.019412f, 0.199323f}};
                outBoneTransform[15] = {{0.025892f, -0.000000f, 0.000000f, 1},
                                        {0.999195f, 0.000000f, 0.000000f, 0.040126f}};
                outBoneTransform[16] = {{0.001696f, -0.006648f, 0.016418f, 1},
                                        {0.509620f, 0.540794f, -0.504891f, 0.439220f}};
                outBoneTransform[17] = {{0.065876f, 0.001786f, 0.000693f, 1},
                                        {0.955009f, -0.065344f, -0.063228f, 0.282294f}};
                outBoneTransform[18] = {{0.040577f, 0.000000f, 0.000000f, 1},
                                        {0.953823f, -0.000972f, 0.000697f, 0.300366f}};
                outBoneTransform[19] = {{0.028698f, -0.000000f, -0.000000f, 1},
                                        {0.977627f, -0.001163f, -0.011433f, 0.210033f}};
                outBoneTransform[20] = {{0.022430f, -0.000000f, 0.000000f, 1},
                                        {1.000000f, 0.000000f, 0.000000f, 0.000000f}};
                outBoneTransform[21] = {{-0.001792f, -0.019041f, 0.015254f, 1},
                                        {0.518602f, 0.511152f, -0.596086f, 0.338315f}};
                outBoneTransform[22] = {{0.062878f, 0.002844f, 0.000332f, 1},
                                        {0.978584f, -0.045398f, -0.103083f, 0.172297f}};
                outBoneTransform[23] = {{0.030154f, 0.000000f, 0.000000f, 1},
                                        {0.970479f, -0.000068f, -0.002025f, 0.241175f}};
                outBoneTransform[24] = {{0.018187f, 0.000000f, 0.000000f, 1},
                                        {0.997053f, -0.000687f, -0.052009f, -0.056395f}};
                outBoneTransform[25] = {{0.018018f, 0.000000f, -0.000000f, 1},
                                        {1.000000f, -0.000000f, -0.000000f, -0.000000f}};
                outBoneTransform[26] = {{-0.005193f, 0.054191f, 0.060030f, 1},
                                        {0.747374f, 0.182388f, 0.599615f, 0.220518f}};
                outBoneTransform[27] = {{0.000171f, 0.016473f, 0.096515f, 1},
                                        {-0.006456f, 0.022747f, -0.932927f, -0.359287f}};
                outBoneTransform[28] = {{-0.038019f, -0.074839f, 0.046941f, 1},
                                        {-0.199973f, 0.698334f, -0.635627f, -0.261380f}};
                outBoneTransform[29] = {{-0.036836f, -0.089774f, 0.081969f, 1},
                                        {-0.191006f, 0.756582f, -0.607429f, -0.148761f}};
                outBoneTransform[30] = {{-0.030241f, -0.086049f, 0.119881f, 1},
                                        {-0.019037f, 0.779368f, -0.612017f, -0.132881f}};
            } else {
                outBoneTransform[6] = {{-0.003802f, 0.021514f, 0.012803f, 1},
                                       {0.395174f, -0.617314f, 0.449185f, 0.510874f}};
                outBoneTransform[7] = {{-0.074204f, 0.005002f, -0.000234f, 1},
                                       {0.737291f, -0.032006f, -0.115013f, 0.664944f}};
                outBoneTransform[8] = {{-0.043287f, 0.000000f, 0.000000f, 1},
                                       {0.611381f, 0.003287f, 0.003823f, 0.791320f}};
                outBoneTransform[9] = {{-0.028275f, -0.000000f, -0.000000f, 1},
                                       {0.745389f, -0.000684f, -0.000945f, 0.666629f}};
                outBoneTransform[10] = {{-0.022821f, -0.000000f, 0.000000f, 1},
                                        {1.000000f, 0.000000f, -0.000000f, 0.000000f}};
                outBoneTransform[11] = {{-0.004885f, 0.006885f, 0.016480f, 1},
                                        {0.527233f, -0.522513f, 0.478085f, 0.469510f}};
                outBoneTransform[12] = {{-0.070953f, -0.000779f, -0.000997f, 1},
                                        {0.826317f, -0.120120f, 0.019005f, 0.549918f}};
                outBoneTransform[13] = {{-0.043108f, -0.000000f, -0.000000f, 1},
                                        {0.958363f, 0.013484f, 0.007380f, 0.285138f}};
                outBoneTransform[14] = {{-0.033266f, -0.000000f, -0.000000f, 1},
                                        {0.977901f, -0.001431f, -0.018078f, 0.208279f}};
                outBoneTransform[15] = {{-0.025892f, 0.000000f, -0.000000f, 1},
                                        {0.999195f, 0.000000f, 0.000000f, 0.040126f}};
                outBoneTransform[16] = {{-0.001696f, -0.006648f, 0.016418f, 1},
                                        {0.541481f, -0.508179f, 0.441001f, 0.504054f}};
                outBoneTransform[17] = {{-0.065876f, -0.001786f, -0.000693f, 1},
                                        {0.953780f, -0.064506f, -0.058812f, 0.287548f}};
                outBoneTransform[18] = {{-0.040577f, -0.000000f, -0.000000f, 1},
                                        {0.954761f, -0.000983f, 0.000698f, 0.297372f}};
                outBoneTransform[19] = {{-0.028698f, 0.000000f, 0.000000f, 1},
                                        {0.976924f, -0.001344f, -0.010281f, 0.213335f}};
                outBoneTransform[20] = {{-0.022430f, 0.000000f, -0.000000f, 1},
                                        {1.000000f, 0.000000f, 0.000000f, 0.000000f}};
                outBoneTransform[21] = {{0.001792f, -0.019041f, 0.015254f, 1},
                                        {0.510569f, -0.514906f, 0.341115f, 0.598191f}};
                outBoneTransform[22] = {{-0.062878f, -0.002844f, -0.000332f, 1},
                                        {0.979195f, -0.043879f, -0.095103f, 0.173800f}};
                outBoneTransform[23] = {{-0.030154f, -0.000000f, -0.000000f, 1},
                                        {0.971387f, -0.000102f, -0.002019f, 0.237494f}};
                outBoneTransform[24] = {{-0.018187f, -0.000000f, -0.000000f, 1},
                                        {0.997961f, 0.000800f, -0.051911f, -0.037114f}};
                outBoneTransform[25] = {{-0.018018f, -0.000000f, 0.000000f, 1},
                                        {1.000000f, -0.000000f, -0.000000f, -0.000000f}};
                outBoneTransform[26] = {{0.004392f, 0.055515f, 0.060253f, 1},
                                        {0.745924f, 0.156756f, -0.597950f, -0.247953f}};
                outBoneTransform[27] = {{-0.000171f, 0.016473f, 0.096515f, 1},
                                        {-0.006456f, 0.022747f, 0.932927f, 0.359287f}};
                outBoneTransform[28] = {{0.038119f, -0.074730f, 0.046338f, 1},
                                        {-0.207931f, 0.699835f, 0.632631f, 0.258406f}};
                outBoneTransform[29] = {{0.035492f, -0.089519f, 0.081636f, 1},
                                        {-0.197555f, 0.760574f, 0.601098f, 0.145535f}};
                outBoneTransform[30] = {{0.029073f, -0.085957f, 0.119561f, 1},
                                        {-0.031423f, 0.791013f, 0.597190f, 0.129133f}};
            }
        }
    } else if (touch) {
        if (withController) {
            if (isLeftHand) {
                outBoneTransform[6] = {{-0.003925f, 0.027171f, 0.014640f, 1},
                                       {0.666448f, 0.430031f, -0.455947f, 0.403772f}};
                outBoneTransform[7] = {{0.074204f, -0.005002f, 0.000234f, 1},
                                       {-0.951843f, 0.009717f, 0.158611f, -0.262188f}};
                outBoneTransform[8] = {{0.043930f, -0.000000f, -0.000000f, 1},
                                       {-0.973045f, -0.044676f, 0.010341f, -0.226012f}};
                outBoneTransform[9] = {{0.028695f, 0.000000f, 0.000000f, 1},
                                       {-0.935253f, -0.002881f, 0.023037f, -0.353217f}};
                outBoneTransform[10] = {{0.022821f, 0.000000f, -0.000000f, 1},
                                        {1.000000f, -0.000000f, -0.000000f, 0.000000f}};
                outBoneTransform[11] = {{0.002177f, 0.007120f, 0.016319f, 1},
                                        {0.529359f, 0.540512f, -0.463783f, 0.461011f}};
                outBoneTransform[12] = {{0.070953f, 0.000779f, 0.000997f, 1},
                                        {0.847397f, -0.257141f, -0.139135f, 0.443213f}};
                outBoneTransform[13] = {{0.043108f, 0.000000f, 0.000000f, 1},
                                        {0.874907f, 0.009875f, 0.026584f, 0.483460f}};
                outBoneTransform[14] = {{0.033266f, -0.000000f, 0.000000f, 1},
                                        {0.894578f, -0.036774f, -0.050597f, 0.442513f}};
                outBoneTransform[15] = {{0.025892f, -0.000000f, 0.000000f, 1},
                                        {0.999195f, -0.000000f, 0.000000f, 0.040126f}};
                outBoneTransform[16] = {{0.000513f, -0.006545f, 0.016348f, 1},
                                        {0.500244f, 0.530784f, -0.516215f, 0.448939f}};
                outBoneTransform[17] = {{0.065876f, 0.001786f, 0.000693f, 1},
                                        {0.831617f, -0.242931f, -0.139695f, 0.479461f}};
                outBoneTransform[18] = {{0.040697f, 0.000000f, 0.000000f, 1},
                                        {0.769163f, -0.001746f, 0.001363f, 0.639049f}};
                outBoneTransform[19] = {{0.028747f, -0.000000f, -0.000000f, 1},
                                        {0.968615f, -0.064538f, -0.046586f, 0.235477f}};
                outBoneTransform[20] = {{0.022430f, -0.000000f, 0.000000f, 1},
                                        {1.000000f, 0.000000f, -0.000000f, -0.000000f}};
                outBoneTransform[21] = {{-0.002478f, -0.018981f, 0.015214f, 1},
                                        {0.474671f, 0.434670f, -0.653212f, 0.398827f}};
                outBoneTransform[22] = {{0.062878f, 0.002844f, 0.000332f, 1},
                                        {0.798788f, -0.199577f, -0.094418f, 0.559636f}};
                outBoneTransform[23] = {{0.030220f, 0.000002f, -0.000000f, 1},
                                        {0.853087f, 0.001644f, -0.000913f, 0.521765f}};
                outBoneTransform[24] = {{0.018187f, -0.000002f, 0.000000f, 1},
                                        {0.974249f, 0.052491f, 0.003591f, 0.219249f}};
                outBoneTransform[25] = {{0.018018f, 0.000000f, -0.000000f, 1},
                                        {1.000000f, 0.000000f, 0.000000f, 0.000000f}};
                outBoneTransform[26] = {{0.006629f, 0.026690f, 0.061870f, 1},
                                        {0.805084f, -0.018369f, 0.584788f, -0.097597f}};
                outBoneTransform[27] = {{-0.009005f, -0.041708f, 0.037992f, 1},
                                        {-0.338860f, 0.939952f, -0.007564f, 0.040082f}};
                outBoneTransform[28] = {{0.017136f, -0.032633f, 0.080682f, 1},
                                        {-0.169466f, 0.800083f, 0.571006f, 0.071415f}};
                outBoneTransform[29] = {{0.011144f, -0.028727f, 0.108366f, 1},
                                        {-0.076328f, 0.788280f, 0.605097f, 0.081527f}};
                outBoneTransform[30] = {{0.011333f, -0.026044f, 0.128585f, 1},
                                        {-0.144791f, 0.737451f, 0.656958f, -0.060069f}};
            } else {
                outBoneTransform[6] = {{-0.003925f, 0.027171f, 0.014640f, 1},
                                       {0.666448f, 0.430031f, -0.455947f, 0.403772f}};
                outBoneTransform[7] = {{0.074204f, -0.005002f, 0.000234f, 1},
                                       {-0.951843f, 0.009717f, 0.158611f, -0.262188f}};
                outBoneTransform[8] = {{0.043930f, -0.000000f, -0.000000f, 1},
                                       {-0.973045f, -0.044676f, 0.010341f, -0.226012f}};
                outBoneTransform[9] = {{0.028695f, 0.000000f, 0.000000f, 1},
                                       {-0.935253f, -0.002881f, 0.023037f, -0.353217f}};
                outBoneTransform[10] = {{0.022821f, 0.000000f, -0.000000f, 1},
                                        {1.000000f, -0.000000f, -0.000000f, 0.000000f}};
                outBoneTransform[11] = {{0.002177f, 0.007120f, 0.016319f, 1},
                                        {0.529359f, 0.540512f, -0.463783f, 0.461011f}};
                outBoneTransform[12] = {{0.070953f, 0.000779f, 0.000997f, 1},
                                        {0.847397f, -0.257141f, -0.139135f, 0.443213f}};
                outBoneTransform[13] = {{0.043108f, 0.000000f, 0.000000f, 1},
                                        {0.874907f, 0.009875f, 0.026584f, 0.483460f}};
                outBoneTransform[14] = {{0.033266f, -0.000000f, 0.000000f, 1},
                                        {0.894578f, -0.036774f, -0.050597f, 0.442513f}};
                outBoneTransform[15] = {{0.025892f, -0.000000f, 0.000000f, 1},
                                        {0.999195f, -0.000000f, 0.000000f, 0.040126f}};
                outBoneTransform[16] = {{0.000513f, -0.006545f, 0.016348f, 1},
                                        {0.500244f, 0.530784f, -0.516215f, 0.448939f}};
                outBoneTransform[17] = {{0.065876f, 0.001786f, 0.000693f, 1},
                                        {0.831617f, -0.242931f, -0.139695f, 0.479461f}};
                outBoneTransform[18] = {{0.040697f, 0.000000f, 0.000000f, 1},
                                        {0.769163f, -0.001746f, 0.001363f, 0.639049f}};
                outBoneTransform[19] = {{0.028747f, -0.000000f, -0.000000f, 1},
                                        {0.968615f, -0.064538f, -0.046586f, 0.235477f}};
                outBoneTransform[20] = {{0.022430f, -0.000000f, 0.000000f, 1},
                                        {1.000000f, 0.000000f, -0.000000f, -0.000000f}};
                outBoneTransform[21] = {{-0.002478f, -0.018981f, 0.015214f, 1},
                                        {0.474671f, 0.434670f, -0.653212f, 0.398827f}};
                outBoneTransform[22] = {{0.062878f, 0.002844f, 0.000332f, 1},
                                        {0.798788f, -0.199577f, -0.094418f, 0.559636f}};
                outBoneTransform[23] = {{0.030220f, 0.000002f, -0.000000f, 1},
                                        {0.853087f, 0.001644f, -0.000913f, 0.521765f}};
                outBoneTransform[24] = {{0.018187f, -0.000002f, 0.000000f, 1},
                                        {0.974249f, 0.052491f, 0.003591f, 0.219249f}};
                outBoneTransform[25] = {{0.018018f, 0.000000f, -0.000000f, 1},
                                        {1.000000f, 0.000000f, 0.000000f, 0.000000f}};
                outBoneTransform[26] = {{0.006629f, 0.026690f, 0.061870f, 1},
                                        {0.805084f, -0.018369f, 0.584788f, -0.097597f}};
                outBoneTransform[27] = {{-0.009005f, -0.041708f, 0.037992f, 1},
                                        {-0.338860f, 0.939952f, -0.007564f, 0.040082f}};
                outBoneTransform[28] = {{0.017136f, -0.032633f, 0.080682f, 1},
                                        {-0.169466f, 0.800083f, 0.571006f, 0.071415f}};
                outBoneTransform[29] = {{0.011144f, -0.028727f, 0.108366f, 1},
                                        {-0.076328f, 0.788280f, 0.605097f, 0.081527f}};
                outBoneTransform[30] = {{0.011333f, -0.026044f, 0.128585f, 1},
                                        {-0.144791f, 0.737451f, 0.656958f, -0.060069f}};
            }
        } else {
            if (isLeftHand) {
                outBoneTransform[6] = {{0.002693f, 0.023387f, 0.013573f, 1},
                                       {0.626743f, 0.404630f, -0.499840f, 0.440032f}};
                outBoneTransform[7] = {{0.074204f, -0.005002f, 0.000234f, 1},
                                       {0.869067f, -0.019031f, -0.093524f, 0.485400f}};
                outBoneTransform[8] = {{0.043512f, -0.000000f, -0.000000f, 1},
                                       {0.834068f, 0.020722f, 0.003930f, 0.551259f}};
                outBoneTransform[9] = {{0.028422f, 0.000000f, 0.000000f, 1},
                                       {0.890556f, 0.000289f, -0.009290f, 0.454779f}};
                outBoneTransform[10] = {{0.022821f, 0.000000f, -0.000000f, 1},
                                        {1.000000f, 0.000000f, -0.000000f, 0.000000f}};
                outBoneTransform[11] = {{0.003937f, 0.006967f, 0.016424f, 1},
                                        {0.531603f, 0.532690f, -0.459598f, 0.471602f}};
                outBoneTransform[12] = {{0.070953f, 0.000779f, 0.000997f, 1},
                                        {0.906933f, -0.142169f, -0.015445f, 0.396261f}};
                outBoneTransform[13] = {{0.043108f, 0.000000f, 0.000000f, 1},
                                        {0.975787f, 0.014996f, 0.010867f, 0.217936f}};
                outBoneTransform[14] = {{0.033266f, 0.000000f, 0.000000f, 1},
                                        {0.992777f, -0.002096f, -0.021403f, 0.118029f}};
                outBoneTransform[15] = {{0.025892f, -0.000000f, 0.000000f, 1},
                                        {0.999195f, 0.000000f, 0.000000f, 0.040126f}};
                outBoneTransform[16] = {{0.001282f, -0.006612f, 0.016394f, 1},
                                        {0.513688f, 0.543325f, -0.502550f, 0.434011f}};
                outBoneTransform[17] = {{0.065876f, 0.001786f, 0.000693f, 1},
                                        {0.971280f, -0.068108f, -0.073480f, 0.215818f}};
                outBoneTransform[18] = {{0.040619f, 0.000000f, 0.000000f, 1},
                                        {0.976566f, -0.001379f, 0.000441f, 0.215216f}};
                outBoneTransform[19] = {{0.028715f, -0.000000f, -0.000000f, 1},
                                        {0.987232f, -0.000977f, -0.011919f, 0.158838f}};
                outBoneTransform[20] = {{0.022430f, -0.000000f, 0.000000f, 1},
                                        {1.000000f, 0.000000f, 0.000000f, 0.000000f}};
                outBoneTransform[21] = {{-0.002032f, -0.019020f, 0.015240f, 1},
                                        {0.521784f, 0.511917f, -0.594340f, 0.335325f}};
                outBoneTransform[22] = {{0.062878f, 0.002844f, 0.000332f, 1},
                                        {0.982925f, -0.053050f, -0.108004f, 0.139206f}};
                outBoneTransform[23] = {{0.030177f, 0.000000f, 0.000000f, 1},
                                        {0.979798f, 0.000394f, -0.001374f, 0.199982f}};
                outBoneTransform[24] = {{0.018187f, 0.000000f, 0.000000f, 1},
                                        {0.997410f, -0.000172f, -0.051977f, -0.049724f}};
                outBoneTransform[25] = {{0.018018f, 0.000000f, -0.000000f, 1},
                                        {1.000000f, -0.000000f, -0.000000f, -0.000000f}};
                outBoneTransform[26] = {{-0.004857f, 0.053377f, 0.060017f, 1},
                                        {0.751040f, 0.174397f, 0.601473f, 0.209178f}};
                outBoneTransform[27] = {{-0.013234f, -0.004327f, 0.069740f, 1},
                                        {-0.119277f, 0.262590f, -0.888979f, -0.355718f}};
                outBoneTransform[28] = {{-0.037500f, -0.074514f, 0.046899f, 1},
                                        {-0.204942f, 0.706005f, -0.626220f, -0.259623f}};
                outBoneTransform[29] = {{-0.036251f, -0.089302f, 0.081732f, 1},
                                        {-0.194045f, 0.764033f, -0.596592f, -0.150590f}};
                outBoneTransform[30] = {{-0.029633f, -0.085595f, 0.119439f, 1},
                                        {-0.025015f, 0.787219f, -0.601140f, -0.135243f}};
            } else {
                outBoneTransform[6] = {{-0.002693f, 0.023387f, 0.013573f, 1},
                                       {0.404698f, -0.626951f, 0.439894f, 0.499645f}};
                outBoneTransform[7] = {{-0.074204f, 0.005002f, -0.000234f, 1},
                                       {0.870303f, -0.017421f, -0.092515f, 0.483436f}};
                outBoneTransform[8] = {{-0.043512f, 0.000000f, 0.000000f, 1},
                                       {0.835972f, 0.018944f, 0.003312f, 0.548436f}};
                outBoneTransform[9] = {{-0.028422f, -0.000000f, -0.000000f, 1},
                                       {0.890326f, 0.000173f, -0.008504f, 0.455244f}};
                outBoneTransform[10] = {{-0.022821f, -0.000000f, 0.000000f, 1},
                                        {1.000000f, 0.000000f, -0.000000f, 0.000000f}};
                outBoneTransform[11] = {{-0.003937f, 0.006967f, 0.016424f, 1},
                                        {0.532293f, -0.531137f, 0.472074f, 0.460113f}};
                outBoneTransform[12] = {{-0.070953f, -0.000779f, -0.000997f, 1},
                                        {0.908154f, -0.139967f, -0.013210f, 0.394323f}};
                outBoneTransform[13] = {{-0.043108f, -0.000000f, -0.000000f, 1},
                                        {0.977887f, 0.015350f, 0.008912f, 0.208378f}};
                outBoneTransform[14] = {{-0.033266f, -0.000000f, -0.000000f, 1},
                                        {0.992487f, -0.002006f, -0.020888f, 0.120540f}};
                outBoneTransform[15] = {{-0.025892f, 0.000000f, -0.000000f, 1},
                                        {0.999195f, 0.000000f, 0.000000f, 0.040126f}};
                outBoneTransform[16] = {{-0.001282f, -0.006612f, 0.016394f, 1},
                                        {0.544460f, -0.511334f, 0.436935f, 0.501187f}};
                outBoneTransform[17] = {{-0.065876f, -0.001786f, -0.000693f, 1},
                                        {0.971233f, -0.064561f, -0.071188f, 0.217877f}};
                outBoneTransform[18] = {{-0.040619f, -0.000000f, -0.000000f, 1},
                                        {0.978211f, -0.001419f, 0.000451f, 0.207607f}};
                outBoneTransform[19] = {{-0.028715f, 0.000000f, 0.000000f, 1},
                                        {0.987488f, -0.001166f, -0.010852f, 0.157314f}};
                outBoneTransform[20] = {{-0.022430f, 0.000000f, -0.000000f, 1},
                                        {1.000000f, 0.000000f, 0.000000f, 0.000000f}};
                outBoneTransform[21] = {{0.002032f, -0.019020f, 0.015240f, 1},
                                        {0.513640f, -0.518192f, 0.337332f, 0.594860f}};
                outBoneTransform[22] = {{-0.062878f, -0.002844f, -0.000332f, 1},
                                        {0.983501f, -0.050059f, -0.104491f, 0.138930f}};
                outBoneTransform[23] = {{-0.030177f, -0.000000f, -0.000000f, 1},
                                        {0.981170f, 0.000501f, -0.001363f, 0.193138f}};
                outBoneTransform[24] = {{-0.018187f, -0.000000f, -0.000000f, 1},
                                        {0.997801f, 0.000487f, -0.051933f, -0.041173f}};
                outBoneTransform[25] = {{-0.018018f, -0.000000f, 0.000000f, 1},
                                        {1.000000f, -0.000000f, -0.000000f, -0.000000f}};
                outBoneTransform[26] = {{0.004574f, 0.055518f, 0.060226f, 1},
                                        {0.745334f, 0.161961f, -0.597782f, -0.246784f}};
                outBoneTransform[27] = {{0.013831f, -0.004360f, 0.069547f, 1},
                                        {-0.117443f, 0.257604f, 0.890065f, 0.357255f}};
                outBoneTransform[28] = {{0.038220f, -0.074817f, 0.046428f, 1},
                                        {-0.205767f, 0.697939f, 0.635107f, 0.259191f}};
                outBoneTransform[29] = {{0.035802f, -0.089658f, 0.081733f, 1},
                                        {-0.196007f, 0.758396f, 0.604341f, 0.145564f}};
                outBoneTransform[30] = {{0.029364f, -0.086069f, 0.119701f, 1},
                                        {-0.028444f, 0.787767f, 0.601616f, 0.129123f}};
            }
        }
    } else {
        // no touch
        if (isLeftHand) {
            outBoneTransform[6] = {{0.000632f, 0.026866f, 0.015002f, 1},
                                   {0.644251f, 0.421979f, -0.478202f, 0.422133f}};
            outBoneTransform[7] = {{0.074204f, -0.005002f, 0.000234f, 1},
                                   {0.995332f, 0.007007f, -0.039124f, 0.087949f}};
            outBoneTransform[8] = {{0.043930f, -0.000000f, -0.000000f, 1},
                                   {0.997891f, 0.045808f, 0.002142f, -0.045943f}};
            outBoneTransform[9] = {{0.028695f, 0.000000f, 0.000000f, 1},
                                   {0.999649f, 0.001850f, -0.022782f, -0.013409f}};
            outBoneTransform[10] = {{0.022821f, 0.000000f, -0.000000f, 1},
                                    {1.000000f, 0.000000f, -0.000000f, 0.000000f}};
            outBoneTransform[11] = {{0.002177f, 0.007120f, 0.016319f, 1},
                                    {0.546723f, 0.541277f, -0.442520f, 0.460749f}};
            outBoneTransform[12] = {{0.070953f, 0.000779f, 0.000997f, 1},
                                    {0.980294f, -0.167261f, -0.078959f, 0.069368f}};
            outBoneTransform[13] = {{0.043108f, 0.000000f, 0.000000f, 1},
                                    {0.997947f, 0.018493f, 0.013192f, 0.059886f}};
            outBoneTransform[14] = {{0.033266f, 0.000000f, 0.000000f, 1},
                                    {0.997394f, -0.003328f, -0.028225f, -0.066315f}};
            outBoneTransform[15] = {{0.025892f, -0.000000f, 0.000000f, 1},
                                    {0.999195f, 0.000000f, 0.000000f, 0.040126f}};
            outBoneTransform[16] = {{0.000513f, -0.006545f, 0.016348f, 1},
                                    {0.516692f, 0.550144f, -0.495548f, 0.429888f}};
            outBoneTransform[17] = {{0.065876f, 0.001786f, 0.000693f, 1},
                                    {0.990420f, -0.058696f, -0.101820f, 0.072495f}};
            outBoneTransform[18] = {{0.040697f, 0.000000f, 0.000000f, 1},
                                    {0.999545f, -0.002240f, 0.000004f, 0.030081f}};
            outBoneTransform[19] = {{0.028747f, -0.000000f, -0.000000f, 1},
                                    {0.999102f, -0.000721f, -0.012693f, 0.040420f}};
            outBoneTransform[20] = {{0.022430f, -0.000000f, 0.000000f, 1},
                                    {1.000000f, 0.000000f, 0.000000f, 0.000000f}};
            outBoneTransform[21] = {{-0.002478f, -0.018981f, 0.015214f, 1},
                                    {0.526918f, 0.523940f, -0.584025f, 0.326740f}};
            outBoneTransform[22] = {{0.062878f, 0.002844f, 0.000332f, 1},
                                    {0.986609f, -0.059615f, -0.135163f, 0.069132f}};
            outBoneTransform[23] = {{0.030220f, 0.000000f, 0.000000f, 1},
                                    {0.994317f, 0.001896f, -0.000132f, 0.106446f}};
            outBoneTransform[24] = {{0.018187f, 0.000000f, 0.000000f, 1},
                                    {0.995931f, -0.002010f, -0.052079f, -0.073526f}};
            outBoneTransform[25] = {{0.018018f, 0.000000f, -0.000000f, 1},
                                    {1.000000f, -0.000000f, -0.000000f, -0.000000f}};
            outBoneTransform[26] = {{-0.006059f, 0.056285f, 0.060064f, 1},
                                    {0.737238f, 0.202745f, 0.594267f, 0.249441f}};
            outBoneTransform[27] = {{-0.040416f, -0.043018f, 0.019345f, 1},
                                    {-0.290330f, 0.623527f, -0.663809f, -0.293734f}};
            outBoneTransform[28] = {{-0.039354f, -0.075674f, 0.047048f, 1},
                                    {-0.187047f, 0.678062f, -0.659285f, -0.265683f}};
            outBoneTransform[29] = {{-0.038340f, -0.090987f, 0.082579f, 1},
                                    {-0.183037f, 0.736793f, -0.634757f, -0.143936f}};
            outBoneTransform[30] = {{-0.031806f, -0.087214f, 0.121015f, 1},
                                    {-0.003659f, 0.758407f, -0.639342f, -0.126678f}};
        } else {
            outBoneTransform[6] = {{-0.000632f, 0.026866f, 0.015002f, 1},
                                   {0.421833f, -0.643793f, 0.422458f, 0.478661f}};
            outBoneTransform[7] = {{-0.074204f, 0.005002f, -0.000234f, 1},
                                   {0.994784f, 0.007053f, -0.041286f, 0.093009f}};
            outBoneTransform[8] = {{-0.043930f, 0.000000f, 0.000000f, 1},
                                   {0.998404f, 0.045905f, 0.002780f, -0.032767f}};
            outBoneTransform[9] = {{-0.028695f, -0.000000f, -0.000000f, 1},
                                   {0.999704f, 0.001955f, -0.022774f, -0.008282f}};
            outBoneTransform[10] = {{-0.022821f, -0.000000f, 0.000000f, 1},
                                    {1.000000f, 0.000000f, -0.000000f, 0.000000f}};
            outBoneTransform[11] = {{-0.002177f, 0.007120f, 0.016319f, 1},
                                    {0.541874f, -0.547427f, 0.459996f, 0.441701f}};
            outBoneTransform[12] = {{-0.070953f, -0.000779f, -0.000997f, 1},
                                    {0.979837f, -0.168061f, -0.075910f, 0.076899f}};
            outBoneTransform[13] = {{-0.043108f, -0.000000f, -0.000000f, 1},
                                    {0.997271f, 0.018278f, 0.013375f, 0.070266f}};
            outBoneTransform[14] = {{-0.033266f, -0.000000f, -0.000000f, 1},
                                    {0.998402f, -0.003143f, -0.026423f, -0.049849f}};
            outBoneTransform[15] = {{-0.025892f, 0.000000f, -0.000000f, 1},
                                    {0.999195f, 0.000000f, 0.000000f, 0.040126f}};
            outBoneTransform[16] = {{-0.000513f, -0.006545f, 0.016348f, 1},
                                    {0.548983f, -0.519068f, 0.426914f, 0.496920f}};
            outBoneTransform[17] = {{-0.065876f, -0.001786f, -0.000693f, 1},
                                    {0.989791f, -0.065882f, -0.096417f, 0.081716f}};
            outBoneTransform[18] = {{-0.040697f, -0.000000f, -0.000000f, 1},
                                    {0.999102f, -0.002168f, -0.000020f, 0.042317f}};
            outBoneTransform[19] = {{-0.028747f, 0.000000f, 0.000000f, 1},
                                    {0.998584f, -0.000674f, -0.012714f, 0.051653f}};
            outBoneTransform[20] = {{-0.022430f, 0.000000f, -0.000000f, 1},
                                    {1.000000f, 0.000000f, 0.000000f, 0.000000f}};
            outBoneTransform[21] = {{0.002478f, -0.018981f, 0.015214f, 1},
                                    {0.518597f, -0.527304f, 0.328264f, 0.587580f}};
            outBoneTransform[22] = {{-0.062878f, -0.002844f, -0.000332f, 1},
                                    {0.987294f, -0.063356f, -0.125964f, 0.073274f}};
            outBoneTransform[23] = {{-0.030220f, -0.000000f, -0.000000f, 1},
                                    {0.993413f, 0.001573f, -0.000147f, 0.114578f}};
            outBoneTransform[24] = {{-0.018187f, -0.000000f, -0.000000f, 1},
                                    {0.997047f, -0.000695f, -0.052009f, -0.056495f}};
            outBoneTransform[25] = {{-0.018018f, -0.000000f, 0.000000f, 1},
                                    {1.000000f, -0.000000f, -0.000000f, -0.000000f}};
            outBoneTransform[26] = {{0.005198f, 0.054204f, 0.060030f, 1},
                                    {0.747318f, 0.182508f, -0.599586f, -0.220688f}};
            outBoneTransform[27] = {{0.038779f, -0.042973f, 0.019824f, 1},
                                    {-0.297445f, 0.639373f, 0.648910f, 0.285734f}};
            outBoneTransform[28] = {{0.038027f, -0.074844f, 0.046941f, 1},
                                    {-0.199898f, 0.698218f, 0.635767f, 0.261406f}};
            outBoneTransform[29] = {{0.036845f, -0.089781f, 0.081973f, 1},
                                    {-0.190960f, 0.756469f, 0.607591f, 0.148733f}};
            outBoneTransform[30] = {{0.030251f, -0.086056f, 0.119887f, 1},
                                    {-0.018948f, 0.779249f, 0.612180f, 0.132846f}};
        }
    }
}

void GetGripClickBoneTransform(bool withController,
                               bool isLeftHand,
                               vr::VRBoneTransform_t outBoneTransform[]) {
    if (withController) {
        if (isLeftHand) {
            outBoneTransform[11] = {{0.002177f, 0.007120f, 0.016319f, 1},
                                    {0.529359f, 0.540512f, -0.463783f, 0.461011f}};
            outBoneTransform[12] = {{0.070953f, 0.000779f, 0.000997f, 1},
                                    {-0.831727f, 0.270927f, 0.175647f, -0.451638f}};
            outBoneTransform[13] = {{0.043108f, 0.000000f, 0.000000f, 1},
                                    {-0.854886f, -0.008231f, -0.028107f, -0.517990f}};
            outBoneTransform[14] = {{0.033266f, -0.000000f, 0.000000f, 1},
                                    {-0.825759f, 0.085208f, 0.086456f, -0.550805f}};
            outBoneTransform[15] = {{0.025892f, -0.000000f, 0.000000f, 1},
                                    {0.999195f, -0.000000f, 0.000000f, 0.040126f}};
            outBoneTransform[16] = {{0.000513f, -0.006545f, 0.016348f, 1},
                                    {0.500244f, 0.530784f, -0.516215f, 0.448939f}};
            outBoneTransform[17] = {{0.065876f, 0.001786f, 0.000693f, 1},
                                    {0.831617f, -0.242931f, -0.139695f, 0.479461f}};
            outBoneTransform[18] = {{0.040697f, 0.000000f, 0.000000f, 1},
                                    {0.769163f, -0.001746f, 0.001363f, 0.639049f}};
            outBoneTransform[19] = {{0.028747f, -0.000000f, -0.000000f, 1},
                                    {0.968615f, -0.064537f, -0.046586f, 0.235477f}};
            outBoneTransform[20] = {{0.022430f, -0.000000f, 0.000000f, 1},
                                    {1.000000f, 0.000000f, -0.000000f, -0.000000f}};
            outBoneTransform[21] = {{-0.002478f, -0.018981f, 0.015214f, 1},
                                    {0.474671f, 0.434670f, -0.653212f, 0.398827f}};
            outBoneTransform[22] = {{0.062878f, 0.002844f, 0.000332f, 1},
                                    {0.798788f, -0.199577f, -0.094418f, 0.559636f}};
            outBoneTransform[23] = {{0.030220f, 0.000002f, -0.000000f, 1},
                                    {0.853087f, 0.001644f, -0.000913f, 0.521765f}};
            outBoneTransform[24] = {{0.018187f, -0.000002f, 0.000000f, 1},
                                    {0.974249f, 0.052491f, 0.003591f, 0.219249f}};
            outBoneTransform[25] = {{0.018018f, 0.000000f, -0.000000f, 1},
                                    {1.000000f, 0.000000f, 0.000000f, 0.000000f}};

            outBoneTransform[28] = {{0.016642f, -0.029992f, 0.083200f, 1},
                                    {-0.094577f, 0.694550f, 0.702845f, 0.121100f}};
            outBoneTransform[29] = {{0.011144f, -0.028727f, 0.108366f, 1},
                                    {-0.076328f, 0.788280f, 0.605097f, 0.081527f}};
            outBoneTransform[30] = {{0.011333f, -0.026044f, 0.128585f, 1},
                                    {-0.144791f, 0.737451f, 0.656958f, -0.060069f}};
        } else {
            outBoneTransform[11] = {{0.002177f, 0.007120f, 0.016319f, 1},
                                    {0.529359f, 0.540512f, -0.463783f, 0.461011f}};
            outBoneTransform[12] = {{0.070953f, 0.000779f, 0.000997f, 1},
                                    {-0.831727f, 0.270927f, 0.175647f, -0.451638f}};
            outBoneTransform[13] = {{0.043108f, 0.000000f, 0.000000f, 1},
                                    {-0.854886f, -0.008231f, -0.028107f, -0.517990f}};
            outBoneTransform[14] = {{0.033266f, -0.000000f, 0.000000f, 1},
                                    {-0.825759f, 0.085208f, 0.086456f, -0.550805f}};
            outBoneTransform[15] = {{0.025892f, -0.000000f, 0.000000f, 1},
                                    {0.999195f, -0.000000f, 0.000000f, 0.040126f}};
            outBoneTransform[16] = {{0.000513f, -0.006545f, 0.016348f, 1},
                                    {0.500244f, 0.530784f, -0.516215f, 0.448939f}};
            outBoneTransform[17] = {{0.065876f, 0.001786f, 0.000693f, 1},
                                    {0.831617f, -0.242931f, -0.139695f, 0.479461f}};
            outBoneTransform[18] = {{0.040697f, 0.000000f, 0.000000f, 1},
                                    {0.769163f, -0.001746f, 0.001363f, 0.639049f}};
            outBoneTransform[19] = {{0.028747f, -0.000000f, -0.000000f, 1},
                                    {0.968615f, -0.064537f, -0.046586f, 0.235477f}};
            outBoneTransform[20] = {{0.022430f, -0.000000f, 0.000000f, 1},
                                    {1.000000f, 0.000000f, -0.000000f, -0.000000f}};
            outBoneTransform[21] = {{-0.002478f, -0.018981f, 0.015214f, 1},
                                    {0.474671f, 0.434670f, -0.653212f, 0.398827f}};
            outBoneTransform[22] = {{0.062878f, 0.002844f, 0.000332f, 1},
                                    {0.798788f, -0.199577f, -0.094418f, 0.559636f}};
            outBoneTransform[23] = {{0.030220f, 0.000002f, -0.000000f, 1},
                                    {0.853087f, 0.001644f, -0.000913f, 0.521765f}};
            outBoneTransform[24] = {{0.018187f, -0.000002f, 0.000000f, 1},
                                    {0.974249f, 0.052491f, 0.003591f, 0.219249f}};
            outBoneTransform[25] = {{0.018018f, 0.000000f, -0.000000f, 1},
                                    {1.000000f, 0.000000f, 0.000000f, 0.000000f}};

            outBoneTransform[28] = {{0.016642f, -0.029992f, 0.083200f, 1},
                                    {-0.094577f, 0.694550f, 0.702845f, 0.121100f}};
            outBoneTransform[29] = {{0.011144f, -0.028727f, 0.108366f, 1},
                                    {-0.076328f, 0.788280f, 0.605097f, 0.081527f}};
            outBoneTransform[30] = {{0.011333f, -0.026044f, 0.128585f, 1},
                                    {-0.144791f, 0.737451f, 0.656958f, -0.060069f}};
        }

    } else {
        if (isLeftHand) {
            outBoneTransform[11] = {{0.005787f, 0.006806f, 0.016534f, 1},
                                    {0.514203f, 0.522315f, -0.478348f, 0.483700f}};
            outBoneTransform[12] = {{0.070953f, 0.000779f, 0.000997f, 1},
                                    {0.723653f, -0.097901f, 0.048546f, 0.681458f}};
            outBoneTransform[13] = {{0.043108f, 0.000000f, 0.000000f, 1},
                                    {0.637464f, -0.002366f, -0.002831f, 0.770472f}};
            outBoneTransform[14] = {{0.033266f, 0.000000f, 0.000000f, 1},
                                    {0.658008f, 0.002610f, 0.003196f, 0.753000f}};
            outBoneTransform[15] = {{0.025892f, -0.000000f, 0.000000f, 1},
                                    {0.999195f, 0.000000f, 0.000000f, 0.040126f}};
            outBoneTransform[16] = {{0.004123f, -0.006858f, 0.016563f, 1},
                                    {0.489609f, 0.523374f, -0.520644f, 0.463997f}};
            outBoneTransform[17] = {{0.065876f, 0.001786f, 0.000693f, 1},
                                    {0.759970f, -0.055609f, 0.011571f, 0.647471f}};
            outBoneTransform[18] = {{0.040331f, 0.000000f, 0.000000f, 1},
                                    {0.664315f, 0.001595f, 0.001967f, 0.747449f}};
            outBoneTransform[19] = {{0.028489f, -0.000000f, -0.000000f, 1},
                                    {0.626957f, -0.002784f, -0.003234f, 0.779042f}};
            outBoneTransform[20] = {{0.022430f, -0.000000f, 0.000000f, 1},
                                    {1.000000f, 0.000000f, 0.000000f, 0.000000f}};
            outBoneTransform[21] = {{0.001131f, -0.019295f, 0.015429f, 1},
                                    {0.479766f, 0.477833f, -0.630198f, 0.379934f}};
            outBoneTransform[22] = {{0.062878f, 0.002844f, 0.000332f, 1},
                                    {0.827001f, 0.034282f, 0.003440f, 0.561144f}};
            outBoneTransform[23] = {{0.029874f, 0.000000f, 0.000000f, 1},
                                    {0.702185f, -0.006716f, -0.009289f, 0.711903f}};
            outBoneTransform[24] = {{0.017979f, 0.000000f, 0.000000f, 1},
                                    {0.676853f, 0.007956f, 0.009917f, 0.736009f}};
            outBoneTransform[25] = {{0.018018f, 0.000000f, -0.000000f, 1},
                                    {1.000000f, -0.000000f, -0.000000f, -0.000000f}};

            outBoneTransform[28] = {{0.000448f, 0.001536f, 0.116543f, 1},
                                    {-0.039357f, 0.105143f, -0.928833f, -0.353079f}};
            outBoneTransform[29] = {{0.003949f, -0.014869f, 0.130608f, 1},
                                    {-0.055071f, 0.068695f, -0.944016f, -0.317933f}};
            outBoneTransform[30] = {{0.003263f, -0.034685f, 0.139926f, 1},
                                    {0.019690f, -0.100741f, -0.957331f, -0.270149f}};
        } else {
            outBoneTransform[11] = {{-0.005787f, 0.006806f, 0.016534f, 1},
                                    {0.522315f, -0.514203f, 0.483700f, 0.478348f}};
            outBoneTransform[12] = {{-0.070953f, -0.000779f, -0.000997f, 1},
                                    {0.723653f, -0.097901f, 0.048546f, 0.681458f}};
            outBoneTransform[13] = {{-0.043108f, -0.000000f, -0.000000f, 1},
                                    {0.637464f, -0.002366f, -0.002831f, 0.770472f}};
            outBoneTransform[14] = {{-0.033266f, -0.000000f, -0.000000f, 1},
                                    {0.658008f, 0.002610f, 0.003196f, 0.753000f}};
            outBoneTransform[15] = {{-0.025892f, 0.000000f, -0.000000f, 1},
                                    {0.999195f, 0.000000f, 0.000000f, 0.040126f}};
            outBoneTransform[16] = {{-0.004123f, -0.006858f, 0.016563f, 1},
                                    {0.523374f, -0.489609f, 0.463997f, 0.520644f}};
            outBoneTransform[17] = {{-0.065876f, -0.001786f, -0.000693f, 1},
                                    {0.759970f, -0.055609f, 0.011571f, 0.647471f}};
            outBoneTransform[18] = {{-0.040331f, -0.000000f, -0.000000f, 1},
                                    {0.664315f, 0.001595f, 0.001967f, 0.747449f}};
            outBoneTransform[19] = {{-0.028489f, 0.000000f, 0.000000f, 1},
                                    {0.626957f, -0.002784f, -0.003234f, 0.779042f}};
            outBoneTransform[20] = {{-0.022430f, 0.000000f, -0.000000f, 1},
                                    {1.000000f, 0.000000f, 0.000000f, 0.000000f}};
            outBoneTransform[21] = {{-0.001131f, -0.019295f, 0.015429f, 1},
                                    {0.477833f, -0.479766f, 0.379935f, 0.630198f}};
            outBoneTransform[22] = {{-0.062878f, -0.002844f, -0.000332f, 1},
                                    {0.827001f, 0.034282f, 0.003440f, 0.561144f}};
            outBoneTransform[23] = {{-0.029874f, -0.000000f, -0.000000f, 1},
                                    {0.702185f, -0.006716f, -0.009289f, 0.711903f}};
            outBoneTransform[24] = {{-0.017979f, -0.000000f, -0.000000f, 1},
                                    {0.676853f, 0.007956f, 0.009917f, 0.736009f}};
            outBoneTransform[25] = {{-0.018018f, -0.000000f, 0.000000f, 1},
                                    {1.000000f, -0.000000f, -0.000000f, -0.000000f}};

            outBoneTransform[28] = {{-0.000448f, 0.001536f, 0.116543f, 1},
                                    {-0.039357f, 0.105143f, 0.928833f, 0.353079f}};
            outBoneTransform[29] = {{-0.003949f, -0.014869f, 0.130608f, 1},
                                    {-0.055071f, 0.068695f, 0.944016f, 0.317933f}};
            outBoneTransform[30] = {{-0.003263f, -0.034685f, 0.139926f, 1},
                                    {0.019690f, -0.100741f, 0.957331f, 0.270149f}};
        }
    }
}

void GetBoneTransform(bool withController, vr::VRBoneTransform_t outBoneTransform[]) {
    auto isLeftHand = !isRightHand;

    vr::VRBoneTransform_t boneTransform1[SKELETON_BONE_COUNT];
    vr::VRBoneTransform_t boneTransform2[SKELETON_BONE_COUNT];

    // root and wrist
    outBoneTransform[0] = {{0.000000f, 0.000000f, 0.000000f, 1},
                           {1.000000f, -0.000000f, -0.000000f, 0.000000f}};
    if (isLeftHand) {
        outBoneTransform[1] = {{-0.034038f, 0.036503f, 0.164722f, 1},
                               {-0.055147f, -0.078608f, -0.920279f, 0.379296f}};
    } else {
        outBoneTransform[1] = {{0.034038f, 0.036503f, 0.164722f, 1},
                               {-0.055147f, -0.078608f, 0.920279f, -0.379296f}};
    }

    // thumb
    GetThumbBoneTransform(withController, isLeftHand, m_lastThumbTouch, boneTransform1);
    GetThumbBoneTransform(withController, isLeftHand, m_currentThumbTouch, boneTransform2);
    for (int boneIdx = 2; boneIdx < 6; boneIdx++) {
        outBoneTransform[boneIdx].position = Lerp(boneTransform1[boneIdx].position,
                                                  boneTransform2[boneIdx].position,
                                                  m_thumbTouchAnimationProgress);
        outBoneTransform[boneIdx].orientation = Slerp(boneTransform1[boneIdx].orientation,
                                                      boneTransform2[boneIdx].orientation,
                                                      m_thumbTouchAnimationProgress);
    }

    // trigger (index to pinky)
    if (m_triggerValue > 0.001) {
        GetTriggerBoneTransform(withController, isLeftHand, true, false, boneTransform1);
        GetTriggerBoneTransform(withController, isLeftHand, true, true, boneTransform2);
        for (int boneIdx = 6; boneIdx < SKELETON_BONE_COUNT; boneIdx++) {
            outBoneTransform[boneIdx].position = Lerp(
                boneTransform1[boneIdx].position, boneTransform2[boneIdx].position, m_triggerValue);
            outBoneTransform[boneIdx].orientation = Slerp(boneTransform1[boneIdx].orientation,
                                                          boneTransform2[boneIdx].orientation,
                                                          m_triggerValue);
        }
    } else {
        GetTriggerBoneTransform(
            withController, isLeftHand, m_lastTriggerTouch, false, boneTransform1);
        GetTriggerBoneTransform(
            withController, isLeftHand, m_currentTriggerTouch, false, boneTransform2);
        for (int boneIdx = 6; boneIdx < SKELETON_BONE_COUNT; boneIdx++) {
            outBoneTransform[boneIdx].position = Lerp(boneTransform1[boneIdx].position,
                                                      boneTransform2[boneIdx].position,
                                                      m_indexTouchAnimationProgress);
            outBoneTransform[boneIdx].orientation = Slerp(boneTransform1[boneIdx].orientation,
                                                          boneTransform2[boneIdx].orientation,
                                                          m_indexTouchAnimationProgress);
        }
    }

    // grip (middle to pinky)
    if (m_gripValue > 0.001) {
        GetGripClickBoneTransform(withController, isLeftHand, boneTransform2);
        for (int boneIdx = 11; boneIdx < 26; boneIdx++) {
            outBoneTransform[boneIdx].position = Lerp(
                outBoneTransform[boneIdx].position, boneTransform2[boneIdx].position, m_gripValue);
            outBoneTransform[boneIdx].orientation = Slerp(outBoneTransform[boneIdx].orientation,
                                                          boneTransform2[boneIdx].orientation,
                                                          m_gripValue);
        }
        for (int boneIdx = 28; boneIdx < SKELETON_BONE_COUNT; boneIdx++) {
            outBoneTransform[boneIdx].position = Lerp(
                outBoneTransform[boneIdx].position, boneTransform2[boneIdx].position, m_gripValue);
            outBoneTransform[boneIdx].orientation = Slerp(outBoneTransform[boneIdx].orientation,
                                                          boneTransform2[boneIdx].orientation,
                                                          m_gripValue);
        }
    }
}





    virtual DriverPose_t GetPose()
    {
        m_last_pose = CalculatePose();
        return this->m_last_pose;
    }
    virtual DriverPose_t CalculatePose()
    {
        ovrTrackingState ss;
        if (comm_buffer->config.external_tracking) {
            ss = comm_buffer->tracking_state;
        } else if (mSession) {
            ss = ovr_GetTrackingState(mSession,
                (ovr_GetTimeInSeconds() + (comm_buffer->config.extra_prediction_ms * 0.001)),
                ovrTrue);
        } else {
            // error, we switched from external tracking to in-drver tracking
            ss.StatusFlags = 0;
        }
     
        //m_time_of_last_pose = ovr_GetTimeInSeconds();// ss.HandPoses[isRightHand].TimeInSeconds;
        //float delta_t = (comm_buffer->config.extra_prediction_ms * 0.001f) + (ovr_GetTimeInSeconds() - ss.HandPoses[isRightHand].TimeInSeconds);

        DriverPose_t pose = { 0 };
        if(ss.HandStatusFlags[isRightHand] & ovrStatus_PositionValid){
            pose.result = TrackingResult_Running_OK;
            pose.poseIsValid = true;
        } else if (ss.HandStatusFlags[isRightHand] & ovrStatus_OrientationValid) {
            pose.result = TrackingResult_Fallback_RotationOnly;
            pose.poseIsValid = true;
        } else{
            pose.result = TrackingResult_Running_OutOfRange;
            pose.poseIsValid = false;
        }
        
        pose.deviceIsConnected = (ss.HandStatusFlags[isRightHand] != 0);

        ovrQuatf hand_qoffset = { 0.3420201, 0, 0, 0.9396926 };
        ovrQuatf hand_input = ss.HandPoses[isRightHand].ThePose.Orientation;
        ovrQuatf hand_result = ovrQuatfmul(hand_input, hand_qoffset);
        ovrVector3f hand_voffset = { 0,0,0 };
        if (isRightHand) {
            hand_voffset = rotateVector2(hand_offset, hand_input);
        }
        else {
            ovrVector3f left_hand_offset = hand_offset;
            left_hand_offset.x = -left_hand_offset.x;
            hand_voffset = rotateVector2(left_hand_offset, hand_input);
        }


        //hand_result = ovrQuatfmul(overall_rotation, hand_result);
        pose.qRotation.w = hand_result.w;
        pose.qRotation.x = hand_result.x;
        pose.qRotation.y = hand_result.y;
        pose.qRotation.z = hand_result.z;
        ovrVector3f position;
        if (comm_buffer->config.be_objects) {
            position.x = ss.HandPoses[isRightHand].ThePose.Position.x;
            position.y = ss.HandPoses[isRightHand].ThePose.Position.y;
            position.z = ss.HandPoses[isRightHand].ThePose.Position.z;
        }
        else {
            position.x = ss.HandPoses[isRightHand].ThePose.Position.x + hand_voffset.x + hand_offset2.x;
            position.y = ss.HandPoses[isRightHand].ThePose.Position.y + hand_voffset.y + hand_offset2.y;
            position.z = ss.HandPoses[isRightHand].ThePose.Position.z + hand_voffset.z + hand_offset2.z;
        }
        //position = rotateVector2(position, overall_rotation);
        pose.vecPosition[0] = position.x;// +overall_offset.x;
        pose.vecPosition[1] = position.y + 0.09;// +overall_offset.y;
        pose.vecPosition[2] = position.z;// +overall_offset.z;



        ovrVector3f linAcc = (ss.HandPoses[isRightHand].LinearAcceleration);
        ovrVector3f linVel = (ss.HandPoses[isRightHand].LinearVelocity);
        ovrQuatf hand_nqoffset = { 0.3420201, 0, 0, -0.9396926 };
        /*linAcc = rotateVector2(linAcc, hand_qoffset);
        linVel = rotateVector2(linVel, hand_nqoffset);*/    //do not do this


        pose.vecAcceleration[0] = linAcc.x;
        pose.vecAcceleration[1] = linAcc.y;
        pose.vecAcceleration[2] = linAcc.z;

        pose.qWorldFromDriverRotation = HmdQuaternion_Init(1, 0, 0, 0);
        pose.qDriverFromHeadRotation = HmdQuaternion_Init(1, 0, 0, 0);

        pose.vecVelocity[0] = linVel.x;
        pose.vecVelocity[1] = linVel.y;
        pose.vecVelocity[2] = linVel.z;


        pose.poseTimeOffset = 0;  // let's let Oculus do it

        pose.vecAngularAcceleration[0] = ss.HandPoses[isRightHand].AngularAcceleration.x;
        pose.vecAngularAcceleration[1] = ss.HandPoses[isRightHand].AngularAcceleration.y;
        pose.vecAngularAcceleration[2] = ss.HandPoses[isRightHand].AngularAcceleration.z;

        pose.vecAngularVelocity[0] = ss.HandPoses[isRightHand].AngularVelocity.x;
        pose.vecAngularVelocity[1] = ss.HandPoses[isRightHand].AngularVelocity.y;
        pose.vecAngularVelocity[2] = ss.HandPoses[isRightHand].AngularVelocity.z;

        if (comm_buffer->config.do_world_transformation) {
            pose.qWorldFromDriverRotation = comm_buffer->config.world_orientation_q * pose.qWorldFromDriverRotation;

            vr::HmdVector3d_t rotatedTranslation = quaternionRotateVector(comm_buffer->config.world_orientation_q, pose.vecWorldFromDriverTranslation);
            pose.vecWorldFromDriverTranslation[0] = rotatedTranslation.v[0] + comm_buffer->config.world_translation[0];
            pose.vecWorldFromDriverTranslation[1] = rotatedTranslation.v[1] + comm_buffer->config.world_translation[1];
            pose.vecWorldFromDriverTranslation[2] = rotatedTranslation.v[2] + comm_buffer->config.world_translation[2];
                
        }

        return pose;
    }


    const static int NUM_BONES = 31;

    vr::HmdQuaternionf_t scaler_quat_mult(vr::HmdQuaternionf_t q, float s) {
        vr::HmdQuaternionf_t qr{ q.w * s, q.x * s, q.y * s, q.z * s };
        return qr;
    }
    vr::HmdQuaternionf_t add_quat(vr::HmdQuaternionf_t q1, vr::HmdQuaternionf_t q2) {
        vr::HmdQuaternionf_t qr{ q1.w + q2.w, q1.x + q2.x, q1.y + q2.y, q1.z + q2.z };
        return qr;
    }

    HmdQuaternionf_t Nlerp(HmdQuaternionf_t qa, HmdQuaternionf_t qb, float t2) {
        HmdQuaternionf_t qm;
        float t1 = 1.0f - t2;
        qm = add_quat(scaler_quat_mult(qa, t1), scaler_quat_mult(qb, t2));
        float len = sqrtf(qm.x * qm.x + qm.y * qm.y + qm.z * qm.z + qm.w * qm.w);

        return scaler_quat_mult(qm, 1.0 / len);
    }

    HmdVector4_t v4_scalar_mult(HmdVector4_t v, float s) {
        HmdVector4_t result{ v.v[0] * s, v.v[1] * s, v.v[2] * s, v.v[3] * s };
        return result;
    }

    HmdVector4_t v4_add(HmdVector4_t v1, HmdVector4_t v2) {
        HmdVector4_t r{ v1.v[0] + v2.v[0], v1.v[1] + v2.v[1], v1.v[2] + v2.v[2], v1.v[3] + v2.v[3] };
        return r;
    }

    VRBoneTransform_t blend_bones(VRBoneTransform_t b1, VRBoneTransform_t b2, float f) {
        VRBoneTransform_t br;
        br.position = v4_add(v4_scalar_mult(b1.position, (1.0 - f)), v4_scalar_mult(b2.position, f));
        br.orientation = Nlerp(b1.orientation, b2.orientation, f);
        return br;
    }

    void RunFrame()
    {
        //#if defined( _WINDOWS )
                // Your driver would read whatever hardware state is associated with its input components and pass that
                // in to UpdateBooleanComponent. This could happen in RunFrame or on a thread of your own that's reading USB
                // state. There's no need to update input state unless it changes, but it doesn't do any harm to do so.
        if (!comm_buffer->config.be_objects) {
#if USE_MUTEX
            if (!WaitForSingleObject(comm_mutex, 10)) {
#else
                {
#endif

                    ovrInputState& inputState(comm_buffer->input_state);
                    if (isRightHand) {
                        //ovr_GetInputState(mSession, ovrControllerType::ovrControllerType_RTouch, &inputState);
                        m_currentThumbTouch = inputState.Touches & (ovrTouch_A | ovrTouch_B | ovrTouch_RThumb | ovrTouch_RThumbRest);
                        m_currentTriggerTouch = inputState.Touches & ovrTouch_RIndexTrigger;
                        vr::VRDriverInput()->UpdateBooleanComponent(m_compAc, inputState.Buttons & ovrButton_A, 0);
                        vr::VRDriverInput()->UpdateBooleanComponent(m_compBc, inputState.Buttons & ovrButton_B, 0);
                        vr::VRDriverInput()->UpdateBooleanComponent(m_compJoyc, inputState.Buttons & ovrButton_RThumb, 0);
                        vr::VRDriverInput()->UpdateBooleanComponent(m_compAt, inputState.Touches & ovrTouch_A, 0);
                        vr::VRDriverInput()->UpdateBooleanComponent(m_compBt, inputState.Touches & ovrTouch_B, 0);
                        vr::VRDriverInput()->UpdateBooleanComponent(m_compTrigt, inputState.Touches & ovrTouch_RIndexTrigger, 0);
                        vr::VRDriverInput()->UpdateBooleanComponent(m_compJoyt, inputState.Touches & ovrTouch_RThumb, 0);
                        vr::VRDriverInput()->UpdateBooleanComponent(m_compThumbt, inputState.Touches & ovrTouch_RThumbRest, 0);
                        vr::VRDriverInput()->UpdateBooleanComponent(m_compSyst, inputState.Touches & ovrTouch_RThumbRest, 0);
                    }
                    else {
                        //ovr_GetInputState(mSession, ovrControllerType::ovrControllerType_LTouch, &inputState);
                        m_currentThumbTouch = inputState.Touches & (ovrTouch_X | ovrTouch_Y | ovrTouch_LThumb | ovrTouch_LThumbRest);
                        m_currentTriggerTouch = inputState.Touches & ovrTouch_LIndexTrigger;
                        vr::VRDriverInput()->UpdateBooleanComponent(m_compXc, inputState.Buttons & ovrButton_X, 0);
                        vr::VRDriverInput()->UpdateBooleanComponent(m_compYc, inputState.Buttons & ovrButton_Y, 0);
                        vr::VRDriverInput()->UpdateBooleanComponent(m_compJoyc, inputState.Buttons & ovrButton_LThumb, 0);
                        vr::VRDriverInput()->UpdateBooleanComponent(m_compXt, inputState.Touches & ovrTouch_X, 0);
                        vr::VRDriverInput()->UpdateBooleanComponent(m_compYt, inputState.Touches & ovrTouch_Y, 0);
                        vr::VRDriverInput()->UpdateBooleanComponent(m_compTrigt, inputState.Touches & ovrTouch_LIndexTrigger, 0);
                        vr::VRDriverInput()->UpdateBooleanComponent(m_compJoyt, inputState.Touches & ovrTouch_LThumb, 0);
                        vr::VRDriverInput()->UpdateBooleanComponent(m_compThumbt, inputState.Touches & ovrTouch_LThumbRest, 0);
                        vr::VRDriverInput()->UpdateBooleanComponent(m_compSyst, inputState.Touches & ovrTouch_LThumbRest, 0);

                        vr::VRDriverInput()->UpdateBooleanComponent(m_compSysc, inputState.Buttons & (ovrButton_Home | ovrButton_Enter), 0);
                    }

                    m_triggerValue = inputState.IndexTrigger[isRightHand];
                    m_gripValue = inputState.HandTrigger[isRightHand];










#if DO_SKELETON

                    if (m_lastThumbTouch != m_currentThumbTouch) {
                        m_thumbTouchAnimationProgress += 1.f / ANIMATION_FRAME_COUNT;
                        if (m_thumbTouchAnimationProgress > 1.f) {
                            m_thumbTouchAnimationProgress = 0;
                            m_lastThumbTouch = m_currentThumbTouch;
                        }
                    }
                    else {
                        m_thumbTouchAnimationProgress = 0;
                    }

                    if (m_lastTriggerTouch != m_currentTriggerTouch) {
                        m_indexTouchAnimationProgress += 1.f / ANIMATION_FRAME_COUNT;
                        if (m_indexTouchAnimationProgress > 1.f) {
                            m_indexTouchAnimationProgress = 0;
                            m_lastTriggerTouch = m_currentTriggerTouch;
                        }
                    }
                    else {
                        m_indexTouchAnimationProgress = 0;
                    }











                    vr::VRBoneTransform_t boneTransforms[SKELETON_BONE_COUNT];

                    // Perform whatever logic is necessary to convert your device's input into a
                    // skeletal pose, first to create a pose "With Controller", that is as close to the
                    // pose of the user's real hand as possible
                    GetBoneTransform(true, boneTransforms);

                    // Then update the WithController pose on the component with those transforms
                    vr::VRDriverInput()->UpdateSkeletonComponent(m_compSkel,
                        vr::VRSkeletalMotionRange_WithController,
                        boneTransforms,
                        SKELETON_BONE_COUNT);

                    GetBoneTransform(false, boneTransforms);

                    // Then update the WithoutController pose on the component
                    vr::VRDriverInput()->UpdateSkeletonComponent(m_compSkel,
                        vr::VRSkeletalMotionRange_WithoutController,
                        boneTransforms,
                        SKELETON_BONE_COUNT);
#endif











                    vr::VRDriverInput()->UpdateBooleanComponent(m_compTrigc, inputState.IndexTrigger[isRightHand] > 0.1, 0);
                    vr::VRDriverInput()->UpdateScalarComponent(m_compTrigv, inputState.IndexTrigger[isRightHand], 0);
                    vr::VRDriverInput()->UpdateBooleanComponent(m_compGripb, inputState.HandTrigger[isRightHand] > 0.9, 0);
                    vr::VRDriverInput()->UpdateBooleanComponent(m_compGripc, inputState.HandTrigger[isRightHand] > 0.9, 0);
                    vr::VRDriverInput()->UpdateScalarComponent(m_compGripv, inputState.HandTrigger[isRightHand], 0);
                    vr::VRDriverInput()->UpdateScalarComponent(m_compJoyx, inputState.Thumbstick[isRightHand].x, 0);
                    vr::VRDriverInput()->UpdateScalarComponent(m_compJoyy, inputState.Thumbstick[isRightHand].y, 0);

#if USE_MUTEX
                    ReleaseMutex(comm_mutex);
#endif
                }
            }

        //this block of code lets you manipulate the pre and post rotation offsets of the controllers, the final value used in the constructor was determined by manual calibration using this code
        /*
        if (inputState.Buttons & ovrTouch_RThumb) {
            if (inputState.Thumbstick[isRightHand].x > 0.5) hand_offset.x += 0.001f;
            if (inputState.Thumbstick[isRightHand].x < -0.5) hand_offset.x -= 0.001f;

            if (inputState.Thumbstick[isRightHand].y > 0.5) hand_offset.y += 0.001f;
            if (inputState.Thumbstick[isRightHand].y < -0.5) hand_offset.y -= 0.001f;

            if (inputState.HandTrigger[isRightHand] > 0.5) hand_offset.z -= 0.001f;
            if (inputState.IndexTrigger[isRightHand] > 0.5) hand_offset.z += 0.001f;
        }
        else {
            if (inputState.Thumbstick[isRightHand].x > 0.5) hand_offset2.x += 0.001f;
            if (inputState.Thumbstick[isRightHand].x < -0.5) hand_offset2.x -= 0.001f;

            if (inputState.Thumbstick[isRightHand].y > 0.5) hand_offset2.y += 0.001f;
            if (inputState.Thumbstick[isRightHand].y < -0.5) hand_offset2.y -= 0.001f;

            if (inputState.HandTrigger[isRightHand] > 0.5) hand_offset2.z -= 0.001f;
            if (inputState.IndexTrigger[isRightHand] > 0.5) hand_offset2.z += 0.001f;
        }

        std::fstream tmp_out;
        tmp_out.open((isRightHand?"c:/test/ovr_offset_r.txt": "c:/test/ovr_offset_l.txt"));
        tmp_out << "x = " << hand_offset.x << " y = " << hand_offset.y << " z = " << hand_offset.z << " x2 =" << hand_offset2.x << " y2 = " << hand_offset2.y << " z = " << hand_offset2.z << std::endl;
        tmp_out.close();
        tmp_out.open((isRightHand ? "ovr_offset_r.txt" : "ovr_offset_l.txt"));
        tmp_out << "x = " << hand_offset.x << " y = " << hand_offset.y << " z = " << hand_offset.z << " x2 =" << hand_offset2.x << " y2 = " << hand_offset2.y << " z = " << hand_offset2.z << std::endl;
        tmp_out.close();
        std::cout << "x = " << hand_offset.x << " y = " << hand_offset.y << " z = " << hand_offset.z << " x2 =" << hand_offset2.x << " y2 = " << hand_offset2.y << " z = " << hand_offset2.z << std::endl;
        std::cerr << "x = " << hand_offset.x << " y = " << hand_offset.y << " z = " << hand_offset.z << " x2 =" << hand_offset2.x << " y2 = " << hand_offset2.y << " z = " << hand_offset2.z << std::endl;
        */

        m_last_pose = this->CalculatePose();
        vr::VRServerDriverHost()->TrackedDevicePoseUpdated(m_unObjectId, m_last_pose, sizeof(DriverPose_t));


        }

    void ProcessEvent(const vr::VREvent_t & vrEvent)
    {

        //if(vrEvent.trackedDeviceIndex == m_unObjectId)
        switch (vrEvent.eventType)
        {
        case vr::VREvent_Input_HapticVibration:
        {
            if (vrEvent.data.hapticVibration.componentHandle == m_compHaptic)
            {


#if USE_MUTEX                
                if (!WaitForSingleObject(comm_mutex, 10)) {
#endif
                    comm_buffer->vib_duration_s[isRightHand] = vrEvent.data.hapticVibration.fDurationSeconds;
                    comm_buffer->vib_amplitude[isRightHand] = vrEvent.data.hapticVibration.fAmplitude;
                    comm_buffer->vib_frequency[isRightHand] = vrEvent.data.hapticVibration.fFrequency;
                    comm_buffer->vib_valid[isRightHand] = true;
#if USE_MUTEX
                    ReleaseMutex(comm_mutex);
                }
#endif


            }
        }
        break;
        }
    }


    std::string GetSerialNumber() const { return m_sSerialNumber; }

    vr::TrackedDeviceIndex_t m_unObjectId;
private:
    vr::PropertyContainerHandle_t m_ulPropertyContainer;

    vr::VRInputComponentHandle_t m_compAc;
    vr::VRInputComponentHandle_t m_compBc;
    vr::VRInputComponentHandle_t m_compXc;
    vr::VRInputComponentHandle_t m_compYc;
    vr::VRInputComponentHandle_t m_compAt;
    vr::VRInputComponentHandle_t m_compBt;
    vr::VRInputComponentHandle_t m_compXt;
    vr::VRInputComponentHandle_t m_compYt;
    vr::VRInputComponentHandle_t m_compTrigv;
    vr::VRInputComponentHandle_t m_compTrigc;
    vr::VRInputComponentHandle_t m_compTrigt;
    vr::VRInputComponentHandle_t m_compGripv;
    vr::VRInputComponentHandle_t m_compGripb;
    vr::VRInputComponentHandle_t m_compGripc;
    vr::VRInputComponentHandle_t m_compGript;
    vr::VRInputComponentHandle_t m_compJoyx;
    vr::VRInputComponentHandle_t m_compJoyy;
    vr::VRInputComponentHandle_t m_compJoyc;
    vr::VRInputComponentHandle_t m_compJoyt;
    vr::VRInputComponentHandle_t m_compSysc;
    vr::VRInputComponentHandle_t m_compSyst;
    vr::VRInputComponentHandle_t m_compThumbt;
    vr::VRInputComponentHandle_t m_compSkel;

    vr::VRInputComponentHandle_t m_compHaptic;
    std::string m_sSerialNumber;
    std::string m_sModelNumber;
    ovrSession mSession;
    bool isRightHand;
    ovrVector3f hand_offset;
    ovrVector3f hand_offset2;


    DriverPose_t m_last_pose;
    float m_time_of_last_pose;






    static const int SKELETON_BONE_COUNT = 31;
    static const int ANIMATION_FRAME_COUNT = 15;

    // These variables are used for controller hand animation
    // todo: move to rust
    float m_thumbTouchAnimationProgress = 0;
    float m_indexTouchAnimationProgress = 0;
    bool m_currentThumbTouch = false;
    bool m_lastThumbTouch = false;
    bool m_currentTriggerTouch = false;
    bool m_lastTriggerTouch = false;
    float m_triggerValue = 0;
    float m_gripValue = 0;

    /*std::chrono::time_point<std::chrono::steady_clock>  haptic_end;
    float  haptic_strength;
    float  haptic_frequency; */
    //uint8_t hap_buf[24];// = { 255,255,0,0,255,255,0,0,0,0,255,255,0,0,255,255,0,0,0,0,255,255,0,0,255,255,0,0,0,0,255,255,255,255,0,0,0,0,255,255,255,255,0,0,0,0,255,255,255,255,0,0,0,0 };
    //ovrHapticsBuffer vibuffer;
   /* ovrVector3f overall_offset{ 0 };
    ovrQuatf overall_rotation{ 0 };         */
    };
