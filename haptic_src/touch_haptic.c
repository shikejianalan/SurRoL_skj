/*****************************************************************************

Copyright (c) 2004 SensAble Technologies, Inc. All rights reserved.

OpenHaptics(TM) toolkit. The material embodied in this software and use of
this software is subject to the terms and conditions of the clickthrough
Development License Agreement.

For questions, comments or bug reports, go to forums at: 
    http://dsc.sensable.com

Module Name:

  Calibration.c

Description: 

  This example demonstrates how to handle haptic device calibration using the
  functions available in the HD API.

*******************************************************************************/
#ifdef  _WIN64
#pragma warning (disable:4996)
#endif

#include <stdio.h>
#include <assert.h>

#if defined(WIN32)
# include <windows.h>
# include <test.h>
#else
# include "touch_haptic.h"
# include <unistd.h>
# define Sleep(x) usleep((x) * 1000)
#endif

#include <HD/hd.h>
#include <HDU/hduError.h>
#include <HDU/hduVector.h>

HDCallbackCode HDCALLBACK DeviceAllInfoCallback_right(void *pUserData);
HDCallbackCode HDCALLBACK DeviceAllInfoCallback_left(void *pUserData);
HDCallbackCode HDCALLBACK AnchoredPointCallback(void *pUserData);
HDCallbackCode HDCALLBACK GetEndPosition(void *pUserData);
HDCallbackCode HDCALLBACK RenderForceCallback(void *pUserData);

HDenum GetCalibrationStatus_right();
HDenum GetCalibrationStatus_left();
void getDeviceAction_right(float* retrived_info, int n1);
void getDeviceAction_left(float* retrived_info2, int n2);
void sendForceFeedback_right(float* position_info, int n3);
int renderForce_right(float* position_info2, int n4);

hduVector3Dd anchorPos;

struct S_Haptic_info
{
    /* data */
    hduVector3Dd position;
    hduVector3Dd angle;
    HDint button;
};

typedef struct
{
    hduVector3Dd force;
    hduVector3Dd position;
    HDfloat velocity[3];
    HDint isRenderForce;
} RenderForce;

/*******************************************************************************
 Main function.
*******************************************************************************/
HHD hHD_right;
int calibrationStyle_right;

HHD hHD_left;
int calibrationStyle_left;

int initTouch_right()
{
    HDErrorInfo error;
    int supportedCalibrationStyles;

    hHD_right = hdInitDevice("Right");
    if (HD_DEVICE_ERROR(error = hdGetError())) 
    {
        hduPrintError(stderr, &error, "Failed to initialize haptic device");
        fprintf(stderr, "\nPress any key to quit.\n");
        getch();
        return -1;
    }

    printf("Haptic Calibration\n");
    printf("Found haptic device: %s.\n\n", hdGetString(HD_DEVICE_MODEL_TYPE));

    /* Choose a calibration style.  Some devices may support multiple types of 
       calibration.  In that case, prefer auto calibration over inkwell 
       calibration, and prefer inkwell calibration over reset encoders. */
    hdGetIntegerv(HD_CALIBRATION_STYLE, &supportedCalibrationStyles);
    if (supportedCalibrationStyles & HD_CALIBRATION_ENCODER_RESET)
    {
        calibrationStyle_right = HD_CALIBRATION_ENCODER_RESET;
    }
    if (supportedCalibrationStyles & HD_CALIBRATION_INKWELL)
    {
        calibrationStyle_right = HD_CALIBRATION_INKWELL;
    }
    if (supportedCalibrationStyles & HD_CALIBRATION_AUTO)
    {
        calibrationStyle_right = HD_CALIBRATION_AUTO;
    }

    /* Some haptic devices only support manual encoder calibration via a
       hardware reset. This requires that the endpoint be placed at a known
       physical location when the reset is commanded. For the PHANTOM haptic
       devices, this means positioning the device so that all links are
       orthogonal. Also, this reset is typically performed before the servoloop
       is running, and only technically needs to be performed once after each
       time the device is plugged in. */
    if (calibrationStyle_right == HD_CALIBRATION_ENCODER_RESET)
    {
        printf("Please prepare for manual calibration by\n");
        printf("placing the device at its reset position.\n\n");
        printf("Press any key to continue...\n");

        getch();

        hdUpdateCalibration(calibrationStyle_right);
        if (hdCheckCalibration() == HD_CALIBRATION_OK)
        {
            printf("Calibration complete.\n\n");
        }
        if (HD_DEVICE_ERROR(error = hdGetError()))
        {
            hduPrintError(stderr, &error, "Reset encoders reset failed.");
            return -1;           
        }
    }

    // Enable the force output
    hdEnable(HD_FORCE_OUTPUT);
    printf("Force output enabled");

    // hdStartScheduler();
    // if (HD_DEVICE_ERROR(error = hdGetError()))
    // {
    //     hduPrintError(stderr, &error, "Failed to start the scheduler");
    //     return -1;           
    // }

    /* Some haptic devices are calibrated when the gimbal is placed into
       the device inkwell and updateCalibration is called.  This form of
       calibration is always performed after the servoloop has started 
       running. */
    if (calibrationStyle_right  == HD_CALIBRATION_INKWELL)
    {
        if (GetCalibrationStatus_right() == HD_CALIBRATION_NEEDS_MANUAL_INPUT)
        {
            printf("Please place the device into the inkwell ");
            printf("for calibration.\n\n");
        }
    }

    return 0;
}

int initTouch_left()
{
    HDErrorInfo error;
    int supportedCalibrationStyles;

    hHD_left = hdInitDevice("Left");
    if (HD_DEVICE_ERROR(error = hdGetError())) 
    {
        hduPrintError(stderr, &error, "Failed to initialize haptic device");
        fprintf(stderr, "\nPress any key to quit.\n");
        getch();
        return -1;
    }

    printf("Haptic Calibration\n");
    printf("Found haptic device: %s.\n\n", hdGetString(HD_DEVICE_MODEL_TYPE));

    /* Choose a calibration style.  Some devices may support multiple types of 
       calibration.  In that case, prefer auto calibration over inkwell 
       calibration, and prefer inkwell calibration over reset encoders. */
    hdGetIntegerv(HD_CALIBRATION_STYLE, &supportedCalibrationStyles);
    if (supportedCalibrationStyles & HD_CALIBRATION_ENCODER_RESET)
    {
        calibrationStyle_left = HD_CALIBRATION_ENCODER_RESET;
    }
    if (supportedCalibrationStyles & HD_CALIBRATION_INKWELL)
    {
        calibrationStyle_left = HD_CALIBRATION_INKWELL;
    }
    if (supportedCalibrationStyles & HD_CALIBRATION_AUTO)
    {
        calibrationStyle_left = HD_CALIBRATION_AUTO;
    }

    /* Some haptic devices only support manual encoder calibration via a
       hardware reset. This requires that the endpoint be placed at a known
       physical location when the reset is commanded. For the PHANTOM haptic
       devices, this means positioning the device so that all links are
       orthogonal. Also, this reset is typically performed before the servoloop
       is running, and only technically needs to be performed once after each
       time the device is plugged in. */
    if (calibrationStyle_left == HD_CALIBRATION_ENCODER_RESET)
    {
        printf("Please prepare for manual calibration by\n");
        printf("placing the device at its reset position.\n\n");
        printf("Press any key to continue...\n");

        getch();

        hdUpdateCalibration(calibrationStyle_left);
        if (hdCheckCalibration() == HD_CALIBRATION_OK)
        {
            printf("Calibration complete.\n\n");
        }
        if (HD_DEVICE_ERROR(error = hdGetError()))
        {
            hduPrintError(stderr, &error, "Reset encoders reset failed.");
            return -1;           
        }
    }

    // hdStartScheduler();
    // if (HD_DEVICE_ERROR(error = hdGetError()))
    // {
    //     hduPrintError(stderr, &error, "Failed to start the scheduler");
    //     return -1;           
    // }

    /* Some haptic devices are calibrated when the gimbal is placed into
       the device inkwell and updateCalibration is called.  This form of
       calibration is always performed after the servoloop has started 
       running. */
    if (calibrationStyle_left  == HD_CALIBRATION_INKWELL)
    {
        if (GetCalibrationStatus_left() == HD_CALIBRATION_NEEDS_MANUAL_INPUT)
        {
            printf("Please place the device into the inkwell ");
            printf("for calibration.\n\n");
        }
    }

    return 0;
}

void startScheduler()
{
    HDErrorInfo error;
    hdStartScheduler();
    if (HD_DEVICE_ERROR(error = hdGetError()))
    {
        hduPrintError(stderr, &error, "Failed to start the scheduler");
        fprintf(stderr, "\nPress any key to quit.\n");
        getch();
        return -1;           
    }
}

void stopScheduler()
{
    hdStopScheduler();
}

void closeTouch_left()
{
    // hdStopScheduler();
    hdDisableDevice(hHD_left);
    return;
}

void closeTouch_right()
{
    // hdStopScheduler();
    hdDisableDevice(hHD_right);
    return;
}

/******************************************************************************
 Begin Scheduler callbacks
 */

HDCallbackCode HDCALLBACK RenderForceCallback(void *pUserData)
{
    RenderForce *pforce = (RenderForce *) pUserData;
    pforce->isRenderForce = 0;
    pforce->force[0] = 0;
    pforce->force[1] = 0;
    pforce->force[2] = 0;
    HDErrorInfo error;

    static const double kStiffiness = 0.1;
    static const double cDamper = -0.002;

    HDint nCurrentButtons, nLastButtons;
    static HDboolean bRenderForce = HD_FALSE;
    hduVector3Dd position;
    HDfloat velocity[3];
    hduVector3Dd force = { 0, 0, 0 };
    hduVector3Dd vel_vec = { 0, 0, 0 };
    hduVector3Dd distance = { 0, 0, 0 };

    hdBeginFrame(hHD_right);

    hdGetDoublev(HD_CURRENT_POSITION, position);
    pforce->position[0] = position[0];
    pforce->position[1] = position[1];
    pforce->position[2] = position[2];
    hdGetFloatv(HD_CURRENT_VELOCITY,velocity);
    pforce->velocity[0] = velocity[0];
    pforce->velocity[1] = velocity[1];
    pforce->velocity[2] = velocity[2];
    vel_vec[0] = velocity[0];
    vel_vec[1] = velocity[1];
    vel_vec[2] = velocity[2];

    hdGetIntegerv(HD_CURRENT_BUTTONS, &nCurrentButtons);
    hdGetIntegerv(HD_LAST_BUTTONS, &nLastButtons);

    // hduVecSubtract(distance, anchorPos, position);
    // HDdouble distance_norm = hduVecMagnitude(distance);

    if ((nCurrentButtons & HD_DEVICE_BUTTON_1) != 0 &&
        (nLastButtons & HD_DEVICE_BUTTON_1) == 0) 
    {
        /* Detected button down */
        bRenderForce = HD_TRUE;
        pforce->isRenderForce = 1;
    }
    else if ((nCurrentButtons & HD_DEVICE_BUTTON_1) == 0 &&
             (nLastButtons & HD_DEVICE_BUTTON_1) != 0)

    {
        /* Detected button up */
        bRenderForce = HD_FALSE;
        pforce->isRenderForce = 0;
        /* Send zero force to the device, or else it will just continue
           rendering the last force sent */
        hdSetDoublev(HD_CURRENT_FORCE, force);
    }

    if (bRenderForce)
    {
        /* Compute spring force as F = k * (anchor - pos), which will attract
           the device position towards the anchor position */
        hduVecSubtract(force, anchorPos, position);
        hduVecScaleInPlace(force, kStiffiness);
        hduVecScaleInPlace(vel_vec, cDamper);
        printf("%.3f %.3f %.3f\n", vel_vec[0], vel_vec[1], vel_vec[2]);
        hduVecAdd(force, force, vel_vec);
        // printf("%.3f %.3f %.3f\n", vel_vec[0], vel_vec[1], vel_vec[2]);
        hdSetDoublev(HD_CURRENT_FORCE, force);
        pforce->force[0] = force[0];
        pforce->force[1] = force[1];
        pforce->force[2] = force[2];
        pforce->isRenderForce = 1;
    }

    hdEndFrame(hHD_right);
    return HD_CALLBACK_DONE;
}

HDCallbackCode HDCALLBACK CalibrationStatusCallback_left(void *pUserData)
{
    HDenum *pStatus = (HDenum *) pUserData;

    hdBeginFrame(hHD_left);
    *pStatus = hdCheckCalibration();
    hdEndFrame(hHD_left);

    return HD_CALLBACK_DONE;
}

HDCallbackCode HDCALLBACK CalibrationStatusCallback_right(void *pUserData)
{
    HDenum *pStatus = (HDenum *) pUserData;

    hdBeginFrame(hHD_right);
    *pStatus = hdCheckCalibration();
    hdEndFrame(hHD_right);

    return HD_CALLBACK_DONE;
}


HDCallbackCode HDCALLBACK DeviceAllInfoCallback_right(void *pUserData)
{

    struct S_Haptic_info *pinfo = (struct S_Haptic_info *) pUserData;

    hduVector3Dd pre_posistion;
    hduVector3Dd pre_angle;

    hdBeginFrame(hHD_right);

    hdGetDoublev(HD_LAST_POSITION, &pre_posistion);
    hdGetDoublev(HD_LAST_GIMBAL_ANGLES, &pre_angle);

    hdGetIntegerv(HD_CURRENT_BUTTONS, &(pinfo->button));
    hdGetDoublev(HD_CURRENT_POSITION, &(pinfo->position));
    hdGetDoublev(HD_CURRENT_GIMBAL_ANGLES, &(pinfo->angle));

    hdEndFrame(hHD_right);

    pinfo->position[0] = pinfo->position[0] - pre_posistion[0];
    pinfo->position[1] = pinfo->position[1] - pre_posistion[1];
    pinfo->position[2] = pinfo->position[2] - pre_posistion[2];

    pinfo->angle[0] = pinfo->angle[0] - pre_angle[0];
    pinfo->angle[1] = pinfo->angle[1] - pre_angle[1];
    pinfo->angle[2] = pinfo->angle[2] - pre_angle[2];

    // printf("Device left position: %.3f %.3f %.3f\n", 
    //     pre_posistion[0], pre_posistion[1], pre_posistion[2]);
    // printf("Device left angle: %.3f %.3f %.3f\n", 
    //     pinfo->angle[0], pinfo->angle[1], pinfo->angle[2]);
    // printf("Device left button: %d\n", pinfo->button);

    return HD_CALLBACK_DONE;
}

HDCallbackCode HDCALLBACK DeviceAllInfoCallback_left(void *pUserData)
{

    struct S_Haptic_info *pinfo = (struct S_Haptic_info *) pUserData;

    hduVector3Dd pre_posistion;
    hduVector3Dd pre_angle;

    hdBeginFrame(hHD_left);

    hdGetDoublev(HD_LAST_POSITION, &pre_posistion);
    hdGetDoublev(HD_LAST_GIMBAL_ANGLES, &pre_angle);

    hdGetIntegerv(HD_CURRENT_BUTTONS, &(pinfo->button));
    hdGetDoublev(HD_CURRENT_POSITION, &(pinfo->position));
    hdGetDoublev(HD_CURRENT_GIMBAL_ANGLES, &(pinfo->angle));

    hdEndFrame(hHD_left);

    pinfo->position[0] = pinfo->position[0] - pre_posistion[0];
    pinfo->position[1] = pinfo->position[1] - pre_posistion[1];
    pinfo->position[2] = pinfo->position[2] - pre_posistion[2];

    pinfo->angle[0] = pinfo->angle[0] - pre_angle[0];
    pinfo->angle[1] = pinfo->angle[1] - pre_angle[1];
    pinfo->angle[2] = pinfo->angle[2] - pre_angle[2];

    // printf("Device left position: %.3f %.3f %.3f\n", 
    //     pinfo->position[0], pinfo->position[1], pinfo->position[2]);
    // printf("Device left angle: %.3f %.3f %.3f\n", 
    //     pinfo->angle[0], pinfo->angle[1], pinfo->angle[2]);
    // printf("Device left button: %d\n", pinfo->button);

    return HD_CALLBACK_DONE;
}

HDenum GetCalibrationStatus_left()
{
    HDenum status;
    hdScheduleSynchronous(CalibrationStatusCallback_left, &status,
                          HD_DEFAULT_SCHEDULER_PRIORITY);
    return status;
}

HDenum GetCalibrationStatus_right()
{
    HDenum status;
    hdScheduleSynchronous(CalibrationStatusCallback_right, &status,
                          HD_DEFAULT_SCHEDULER_PRIORITY);
    return status;
}

int count_frame_right = 0;
int count_frame_left = 0;

void getDeviceAction_right(float* retrived_info, int n1)
{

    struct S_Haptic_info myinfo;

    hdScheduleSynchronous(DeviceAllInfoCallback_right, &myinfo,
    HD_DEFAULT_SCHEDULER_PRIORITY);
    if (count_frame_right<=1)
    {
        retrived_info[0] = 0;
        retrived_info[1] = 0;
        retrived_info[2] = 0;
        retrived_info[3] = 0;
        retrived_info[4] = myinfo.button;   
    }
    else{
        retrived_info[0] = myinfo.position[0];
        retrived_info[1] = myinfo.position[1];
        retrived_info[2] = myinfo.position[2];
        retrived_info[3] = myinfo.angle[2];
        retrived_info[4] = myinfo.button;        
    }
    count_frame_right= count_frame_right + 1;    

    // printf("Device right position: %.3f %.3f %.3f\n", 
    //     myinfo.position[0], myinfo.position[1], myinfo.position[2]);
    // printf("Device right angle: %.3f %.3f %.3f\n", 
    //     myinfo.angle[0], myinfo.angle[1], myinfo.angle[2]);
    // printf("Device right button: %d\n", myinfo.button);
}

void getDeviceAction_left(float* retrived_info2, int n2)
{
    struct S_Haptic_info myinfo;

    hdScheduleSynchronous(DeviceAllInfoCallback_left, &myinfo,
    HD_DEFAULT_SCHEDULER_PRIORITY);
    if (count_frame_left<=1)
    {
        retrived_info2[0] = 0;
        retrived_info2[1] = 0;
        retrived_info2[2] = 0;
        retrived_info2[3] = 0;
        retrived_info2[4] = myinfo.button;   
    }
    else{
        retrived_info2[0] = myinfo.position[0];
        retrived_info2[1] = myinfo.position[1];
        retrived_info2[2] = myinfo.position[2];
        retrived_info2[3] = myinfo.angle[2];
        retrived_info2[4] = myinfo.button;        
    }
    count_frame_left= count_frame_left + 1;    

    // printf("Device left position: %.3f %.3f %.3f\n", 
    //     myinfo.position[0], myinfo.position[1], myinfo.position[2]);
    // printf("Device left angle: %.3f %.3f %.3f\n", 
    //     myinfo.angle[0], myinfo.angle[1], myinfo.angle[2]);
    // printf("Device left button: %d\n", myinfo.button);
}

HDCallbackCode HDCALLBACK AnchoredPointCallback(void *pUserData)
{
    // hduVector3Dd *anchor = (hduVector3Dd *) pUserData;
    // hduVector3Dd anchor = anchorPos;
    static HDboolean bRenderForce = HD_FALSE;
    HDErrorInfo error;

    HDint nCurrentButtons, nLastButtons;
    hduVector3Dd position;
    hduVector3Dd force = { 0, 0, 0 };
    hduVector3Dd distance = { 0, 0, 0 };

    hdBeginFrame(hHD_right);
    
    hdGetDoublev(HD_CURRENT_POSITION, position);
    hdGetIntegerv(HD_CURRENT_BUTTONS, &nCurrentButtons);
    hdGetIntegerv(HD_LAST_BUTTONS, &nLastButtons);

    hduVecSubtract(distance, anchorPos, position);
    HDdouble distance_norm = hduVecMagnitude(distance);

    if ((nCurrentButtons & HD_DEVICE_BUTTON_1) != 0 &&
        (nLastButtons & HD_DEVICE_BUTTON_1) == 0) 
    {
        /* Detected button down */
        // memcpy(anchor, position, sizeof(hduVector3Dd));
        if (distance_norm > 20)
        {
            bRenderForce = HD_TRUE;
        }
        else
        {
            bRenderForce = HD_FALSE;
        }
        // bRenderForce = HD_TRUE;
    }
    else if ((nCurrentButtons & HD_DEVICE_BUTTON_1) == 0 &&
             (nLastButtons & HD_DEVICE_BUTTON_1) != 0)

    {
        /* Detected button up */
        bRenderForce = HD_FALSE;

        /* Send zero force to the device, or else it will just continue
           rendering the last force sent */
        hdSetDoublev(HD_CURRENT_FORCE, force);
    }

    if (bRenderForce)
    {
        /* Compute spring force as F = k * (anchor - pos), which will attract
           the device position towards the anchor position */
        hduVecSubtract(force, anchorPos, position);
        // printf("%.3f %.3f %.3f\n", position[0], position[1], position[2]);
        hduVecScaleInPlace(force, 0.15);
                
        hdSetDoublev(HD_CURRENT_FORCE, force);
    }

    hdEndFrame(hHD_right);

    // printf("%s\n", bRenderForce);
    // printf("%.3f %.3f %.3f\n", force[0], force[1], force[2]);

    return HD_CALLBACK_DONE;

}

HDCallbackCode HDCALLBACK GetEndPosition(void *pUserData)
{
    struct S_Haptic_info *pinfo = (struct S_Haptic_info *) pUserData;

    hdBeginFrame(hHD_right);

    hdGetDoublev(HD_CURRENT_POSITION, &(pinfo->position));

    hdEndFrame(hHD_right);

    return HD_CALLBACK_DONE;
}

int renderForce_right(float* render_info, int n4)
{
    RenderForce force_info;
    
    anchorPos[0] = render_info[0];
    anchorPos[1] = render_info[1];
    anchorPos[2] = render_info[2];

    hdScheduleSynchronous(RenderForceCallback, &force_info,
    HD_DEFAULT_SCHEDULER_PRIORITY);

    render_info[3] = force_info.force[0];
    render_info[4] = force_info.force[1];
    render_info[5] = force_info.force[2];
    render_info[6] = force_info.position[0];
    render_info[7] = force_info.position[1];
    render_info[8] = force_info.position[2];
    render_info[9] = force_info.velocity[0];
    render_info[10] = force_info.velocity[1];
    render_info[11] = force_info.velocity[2];
    // printf("%.3f %.3f %.3f\n", force_info.force[0], force_info.force[1], force_info.force[2]);
    // printf("%.3f %.3f %.3f\n", force_info.position[0], force_info.position[1], force_info.position[2]);

    if (force_info.isRenderForce == 1)
    {
        return 1;
    }
    else
    {
        return 0;
    }

}

int distanceCheck()
{
    struct S_Haptic_info posinfo;

    hdScheduleSynchronous(GetEndPosition, &posinfo,
    HD_DEFAULT_SCHEDULER_PRIORITY);

    hduVector3Dd currentPosition = { 0, 0, 0 };
    hduVector3Dd distance = { 0, 0, 0 };
    
    currentPosition[0] = posinfo.position[0];
    currentPosition[1] = posinfo.position[1];
    currentPosition[2] = posinfo.position[2];
    
    hduVecSubtract(distance, anchorPos, currentPosition);
    HDdouble distance_norm = hduVecMagnitude(distance);

    if (distance_norm < 20) 
    {
        return 1;
    }
    else
    {
        return 0;
    }

    
    // printf("%.3f %.3f %.3f\n", distance[0], distance[1], distance[2]);

}

void sendForceFeedback_right(float* position_info, int n3)
{
    
    anchorPos[0] = position_info[0];
    anchorPos[1] = position_info[1];
    anchorPos[2] = position_info[2];

    // printf("%.3f %.3f %.3f\n", pos_info[0], pos_info[1], pos_info[2]);

    hdScheduleAsynchronous(AnchoredPointCallback, 0,
    HD_MAX_SCHEDULER_PRIORITY);

    // struct S_Haptic_info pos_info;

    // hdScheduleSynchronous(GetEndPosition, &pos_info,
    // HD_MAX_SCHEDULER_PRIORITY);

    // hduVector3Dd initial_point = {10.0, 30.0, 10.0};
    // hduVector3Dd force = { 0, 0, 0 };
    // hduVecSubtract(force, initial_point, pos_info.position);
    // hduVecScaleInPlace(force, 0.1);
    // hdSetDoublev(HD_CURRENT_FORCE, force);
    // hdScheduleSynchronous(AnchoredSpringForceCallback, 0, HD_MAX_SCHEDULER_PRIORITY);
    
    // hduVector3Dd pos_info;
    // hdGetDoublev(HD_CURRENT_POSITION, pos_info);

    // printf("%.3f %.3f %.3f\n", force[0], force[1], force[2]);
    // printf("Device righttt position: %.3f %.3f %.3f\n", 
    //     pos_info.position[0], pos_info.position[1], pos_info.position[2]);

    // HDdouble gSpringStiffness = 0.1;

    // hdSetDoublev(HD_CURRENT_FORCE, force)

}

/*
 End Scheduler callbacks
 *****************************************************************************/

/*****************************************************************************/

