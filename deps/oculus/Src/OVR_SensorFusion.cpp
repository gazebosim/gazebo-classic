/************************************************************************************

Filename    :   OVR_SensorFusion.cpp
Content     :   Methods that determine head orientation from sensor data over time
Created     :   October 9, 2012
Authors     :   Michael Antonov, Steve LaValle, Max Katsev

Copyright   :   Copyright 2012 Oculus VR, Inc. All Rights reserved.

Use of this software is subject to the terms of the Oculus license
agreement provided at the time of installation or download, or which
otherwise accompanies this software in either electronic or hard copy form.

*************************************************************************************/

#include "OVR_SensorFusion.h"
#include "Kernel/OVR_Log.h"
#include "Kernel/OVR_System.h"
#include "OVR_JSON.h"
#include "OVR_Profile.h"

#define MAX_DEVICE_PROFILE_MAJOR_VERSION 1

namespace OVR {

//-------------------------------------------------------------------------------------
// ***** Sensor Fusion

SensorFusion::SensorFusion(SensorDevice* sensor)
  : Stage(0), RunningTime(0), DeltaT(0.001f), 
    Handler(getThis()), pDelegate(0),
    Gain(0.05f), EnableGravity(true), 
    EnablePrediction(true), PredictionDT(0.03f), PredictionTimeIncrement(0.001f),
    FRawMag(10), FAngV(20), 
    GyroOffset(), TiltAngleFilter(1000),
    EnableYawCorrection(false), MagCalibrated(false), MagNumReferences(0), MagRefIdx(-1), MagRefScore(0),
    MotionTrackingEnabled(true)
{
   if (sensor)
       AttachToSensor(sensor);
   MagCalibrationMatrix.SetIdentity();
}

SensorFusion::~SensorFusion()
{
}


bool SensorFusion::AttachToSensor(SensorDevice* sensor)
{
    // clear the cached device information
    CachedSensorInfo.SerialNumber[0] = 0;   
    CachedSensorInfo.VendorId = 0;
    CachedSensorInfo.ProductId = 0;

    if (sensor != NULL)
    {
        // Cache the sensor device so we can access this information during
        // mag saving and loading (avoid holding a reference to sensor to prevent 
        // deadlock on shutdown)
        sensor->GetDeviceInfo(&CachedSensorInfo);   // save the device information
        MessageHandler* pCurrentHandler = sensor->GetMessageHandler();

        if (pCurrentHandler == &Handler)
        {
            Reset();
            return true;
        }

        if (pCurrentHandler != NULL)
        {
            OVR_DEBUG_LOG(
                ("SensorFusion::AttachToSensor failed - sensor %p already has handler", sensor));
            return false;
        }

        // Automatically load the default mag calibration for this sensor
        LoadMagCalibration();        
    }

    if (Handler.IsHandlerInstalled())
    {
        Handler.RemoveHandlerFromDevices();
    }

    if (sensor != NULL)
    {
        sensor->SetMessageHandler(&Handler);
    }

    Reset();
    return true;
}


    // Resets the current orientation
void SensorFusion::Reset()
{
    Lock::Locker lockScope(Handler.GetHandlerLock());
    Q                     = Quatf();
    QUncorrected          = Quatf();
    Stage                 = 0;
    RunningTime           = 0;
    MagNumReferences      = 0;
    MagRefIdx             = -1;
    GyroOffset            = Vector3f();
}

// Compute a rotation required to transform "estimated" into "measured"
// Returns an approximation of the goal rotation in the Simultaneous Orthogonal Rotations Angle representation
// (vector direction is the axis of rotation, norm is the angle)
Vector3f SensorFusion_ComputeCorrection(Vector3f measured, Vector3f estimated)
{
    measured.Normalize();
    estimated.Normalize();
    Vector3f correction = measured.Cross(estimated);
    float cosError = measured.Dot(estimated);
    // from the def. of cross product, correction.Length() = sin(error)
    // therefore sin(error) * sqrt(2 / (1 + cos(error))) = 2 * sin(error / 2) ~= error in [-pi, pi]
    // Mathf::Tolerance is used to avoid div by 0 if cos(error) = -1
    return correction * sqrt(2 / (1 + cosError + Mathf::Tolerance));
}

void SensorFusion::handleMessage(const MessageBodyFrame& msg)
{
    if (msg.Type != Message_BodyFrame || !IsMotionTrackingEnabled())
        return;

    // Put the sensor readings into convenient local variables
    Vector3f gyro  = msg.RotationRate; 
    Vector3f accel = msg.Acceleration;
    Vector3f mag   = msg.MagneticField;

    // Insert current sensor data into filter history
    FRawMag.AddElement(mag);
    FAngV.AddElement(gyro);

    // Apply the calibration parameters to raw mag
    Vector3f calMag = MagCalibrated ? GetCalibratedMagValue(FRawMag.Mean()) : FRawMag.Mean();

    // Set variables accessible through the class API
    DeltaT = msg.TimeDelta;
    AngV   = gyro;
    A      = accel;
    RawMag = mag;  
    CalMag = calMag;

    // Keep track of time
    Stage++;
    RunningTime += DeltaT;

    // Small preprocessing
    Quatf Qinv = Q.Inverted();
    Vector3f up = Qinv.Rotate(Vector3f(0, 1, 0));

    Vector3f gyroCorrected = gyro;

    // Apply integral term
    // All the corrections are stored in the Simultaneous Orthogonal Rotations Angle representation,
    // which allows to combine and scale them by just addition and multiplication
    if (EnableGravity || EnableYawCorrection)
        gyroCorrected -= GyroOffset;

    if (EnableGravity)
    {
        const float spikeThreshold = 0.01f;
        const float gravityThreshold = 0.1f;
        float proportionalGain     = 5 * Gain; // Gain parameter should be removed in a future release
        float integralGain         = 0.0125f;

        Vector3f tiltCorrection = SensorFusion_ComputeCorrection(accel, up);

        if (Stage > 5)
        {
            // Spike detection
            float tiltAngle = up.Angle(accel);
            TiltAngleFilter.AddElement(tiltAngle);
            if (tiltAngle > TiltAngleFilter.Mean() + spikeThreshold)
                proportionalGain = integralGain = 0;
            // Acceleration detection
            const float gravity = 9.8f;
            if (fabs(accel.Length() / gravity - 1) > gravityThreshold)
                integralGain = 0;
        }
        else // Apply full correction at the startup
        {
            proportionalGain = 1 / DeltaT;
            integralGain = 0;
        }

        gyroCorrected += (tiltCorrection * proportionalGain);
        GyroOffset -= (tiltCorrection * integralGain * DeltaT);
    }

    if (EnableYawCorrection && MagCalibrated && RunningTime > 2.0f)
    {
        const float maxMagRefDist = 0.1f;
        const float maxTiltError = 0.05f;
        float proportionalGain   = 0.01f;
        float integralGain       = 0.0005f;

        // Update the reference point if needed
        if (MagRefIdx < 0 || calMag.Distance(MagRefsInBodyFrame[MagRefIdx]) > maxMagRefDist)
        {
            // Delete a bad point
            if (MagRefIdx >= 0 && MagRefScore < 0)
            {
                MagNumReferences--;
                MagRefsInBodyFrame[MagRefIdx] = MagRefsInBodyFrame[MagNumReferences];
                MagRefsInWorldFrame[MagRefIdx] = MagRefsInWorldFrame[MagNumReferences];
            }
            // Find a new one
            MagRefIdx = -1;
            MagRefScore = 1000;
            float bestDist = maxMagRefDist;
            for (int i = 0; i < MagNumReferences; i++)
            {
                float dist = calMag.Distance(MagRefsInBodyFrame[i]);
                if (bestDist > dist)
                {
                    bestDist = dist;
                    MagRefIdx = i;
                }
            }
            // Create one if needed
            if (MagRefIdx < 0 && MagNumReferences < MagMaxReferences)
            {
                MagRefIdx = MagNumReferences;
                MagRefsInBodyFrame[MagRefIdx] = calMag;
                MagRefsInWorldFrame[MagRefIdx] = Q.Rotate(calMag).Normalized();
                MagNumReferences++;
            }
        }

        if (MagRefIdx >= 0)
        {
            Vector3f magEstimated = Qinv.Rotate(MagRefsInWorldFrame[MagRefIdx]);
            Vector3f magMeasured  = calMag.Normalized();

            // Correction is computed in the horizontal plane (in the world frame)
            Vector3f yawCorrection = SensorFusion_ComputeCorrection(magMeasured.ProjectToPlane(up), 
                                                                    magEstimated.ProjectToPlane(up));

            if (fabs(up.Dot(magEstimated - magMeasured)) < maxTiltError)
            {
                MagRefScore += 2;
            }
            else // If the vertical angle is wrong, decrease the score
            {
                MagRefScore -= 1;
                proportionalGain = integralGain = 0;
            }
            gyroCorrected += (yawCorrection * proportionalGain);
            GyroOffset -= (yawCorrection * integralGain * DeltaT);
        }
    }

    // Update the orientation quaternion based on the corrected angular velocity vector
    Q = Q * Quatf(gyroCorrected, gyroCorrected.Length() * DeltaT);

    // The quaternion magnitude may slowly drift due to numerical error,
    // so it is periodically normalized.
    if (Stage % 500 == 0)
        Q.Normalize();
}

//  A predictive filter based on extrapolating the smoothed, current angular velocity
Quatf SensorFusion::GetPredictedOrientation(float pdt)
{		
    Lock::Locker lockScope(Handler.GetHandlerLock());
    Quatf        qP = Q;
    
    if (EnablePrediction)
    {
        // This method assumes a constant angular velocity
        Vector3f angVelF  = FAngV.SavitzkyGolaySmooth8();
        float    angVelFL = angVelF.Length();

        // Force back to raw measurement
        angVelF  = AngV;
        angVelFL = AngV.Length();

        // Dynamic prediction interval: Based on angular velocity to reduce vibration
        const float minPdt   = 0.001f;
        const float slopePdt = 0.1f;
        float       newpdt   = pdt;
        float       tpdt     = minPdt + slopePdt * angVelFL;
        if (tpdt < pdt)
            newpdt = tpdt;
        //LogText("PredictonDTs: %d\n",(int)(newpdt / PredictionTimeIncrement + 0.5f));

        if (angVelFL > 0.001f)
        {
            Vector3f    rotAxisP      = angVelF / angVelFL;  
            float       halfRotAngleP = angVelFL * newpdt * 0.5f;
            float       sinaHRAP      = sin(halfRotAngleP);
            Quatf       deltaQP(rotAxisP.x*sinaHRAP, rotAxisP.y*sinaHRAP,
                                rotAxisP.z*sinaHRAP, cos(halfRotAngleP));
            qP = Q * deltaQP;
        }
    }
    return qP;
}    


Vector3f SensorFusion::GetCalibratedMagValue(const Vector3f& rawMag) const
{
    OVR_ASSERT(HasMagCalibration());
    return MagCalibrationMatrix.Transform(rawMag);
    }

SensorFusion::BodyFrameHandler::~BodyFrameHandler()
{
    RemoveHandlerFromDevices();
}

void SensorFusion::BodyFrameHandler::OnMessage(const Message& msg)
{
    if (msg.Type == Message_BodyFrame)
        pFusion->handleMessage(static_cast<const MessageBodyFrame&>(msg));
    if (pFusion->pDelegate)
        pFusion->pDelegate->OnMessage(msg);
}

bool SensorFusion::BodyFrameHandler::SupportsMessageType(MessageType type) const
{
    return (type == Message_BodyFrame);
}

// Writes the current calibration for a particular device to a device profile file
// sensor - the sensor that was calibrated
// cal_name - an optional name for the calibration or default if cal_name == NULL
bool SensorFusion::SaveMagCalibration(const char* calibrationName) const
{
    if (CachedSensorInfo.SerialNumber[0] == 0 || !HasMagCalibration())
        return false;
    
    // A named calibration may be specified for calibration in different
    // environments, otherwise the default calibration is used
    if (calibrationName == NULL)
        calibrationName = "default";

    // Generate a mag calibration event
    JSON* calibration = JSON::CreateObject();
    // (hardcoded for now) the measurement and representation method 
    calibration->AddStringItem("Version", "2.0");   
    calibration->AddStringItem("Name", "default");

    // time stamp the calibration
    char time_str[64];
   
#ifdef OVR_OS_WIN32
    struct tm caltime;
    localtime_s(&caltime, &MagCalibrationTime);
    strftime(time_str, 64, "%Y-%m-%d %H:%M:%S", &caltime);
#else
    struct tm* caltime;
    caltime = localtime(&MagCalibrationTime);
    strftime(time_str, 64, "%Y-%m-%d %H:%M:%S", caltime);
#endif
   
    calibration->AddStringItem("Time", time_str);

    // write the full calibration matrix
    char matrix[256];
    Matrix4f calmat = GetMagCalibration();
    calmat.ToString(matrix, 256);
    calibration->AddStringItem("CalibrationMatrix", matrix);
    // save just the offset, for backwards compatibility
    // this can be removed when we don't want to support 0.2.4 anymore
    Vector3f center(calmat.M[0][3], calmat.M[1][3], calmat.M[2][3]);
    Matrix4f tmp = calmat; tmp.M[0][3] = tmp.M[1][3] = tmp.M[2][3] = 0; tmp.M[3][3] = 1;
    center = tmp.Inverted().Transform(center);
    Matrix4f oldcalmat; oldcalmat.M[0][3] = center.x; oldcalmat.M[1][3] = center.y; oldcalmat.M[2][3] = center.z; 
    oldcalmat.ToString(matrix, 256);
    calibration->AddStringItem("Calibration", matrix);
    

    String path = GetBaseOVRPath(true);
    path += "/Devices.json";

    // Look for a prexisting device file to edit
    Ptr<JSON> root = *JSON::Load(path);
    if (root)
    {   // Quick sanity check of the file type and format before we parse it
        JSON* version = root->GetFirstItem();
        if (version && version->Name == "Oculus Device Profile Version")
        {   
            int major = atoi(version->Value.ToCStr());
            if (major > MAX_DEVICE_PROFILE_MAJOR_VERSION)
            {
                // don't use the file on unsupported major version number
                root->Release();
                root = NULL;
            }
        }
        else
        {
            root->Release();
            root = NULL;
        }
    }

    JSON* device = NULL;
    if (root)
    {
        device = root->GetFirstItem();   // skip the header
        device = root->GetNextItem(device);
        while (device)
        {   // Search for a previous calibration with the same name for this device
            // and remove it before adding the new one
            if (device->Name == "Device")
            {   
                JSON* item = device->GetItemByName("Serial");
                if (item && item->Value == CachedSensorInfo.SerialNumber)
                {   // found an entry for this device
                    item = device->GetNextItem(item);
                    while (item)
                    {
                        if (item->Name == "MagCalibration")
                        {   
                            JSON* name = item->GetItemByName("Name");
                            if (name && name->Value == calibrationName)
                            {   // found a calibration of the same name
                                item->RemoveNode();
                                item->Release();
                                break;
                            } 
                        }
                        item = device->GetNextItem(item);
                    }

                    // update the auto-mag flag
                    item = device->GetItemByName("EnableYawCorrection");
                    if (item)
                        item->dValue = (double)EnableYawCorrection;
                    else
                        device->AddBoolItem("EnableYawCorrection", EnableYawCorrection);

                    break;
                }
            }

            device = root->GetNextItem(device);
        }
    }
    else
    {   // Create a new device root
        root = *JSON::CreateObject();
        root->AddStringItem("Oculus Device Profile Version", "1.0");
    }

    if (device == NULL)
    {
        device = JSON::CreateObject();
        device->AddStringItem("Product", CachedSensorInfo.ProductName);
        device->AddNumberItem("ProductID", CachedSensorInfo.ProductId);
        device->AddStringItem("Serial", CachedSensorInfo.SerialNumber);
        device->AddBoolItem("EnableYawCorrection", EnableYawCorrection);

        root->AddItem("Device", device);
    }

    // Create and the add the new calibration event to the device
    device->AddItem("MagCalibration", calibration);

    return root->Save(path);
}

// Loads a saved calibration for the specified device from the device profile file
// sensor - the sensor that the calibration was saved for
// cal_name - an optional name for the calibration or the default if cal_name == NULL
bool SensorFusion::LoadMagCalibration(const char* calibrationName)
{
    if (CachedSensorInfo.SerialNumber[0] == 0)
        return false;

    // A named calibration may be specified for calibration in different
    // environments, otherwise the default calibration is used
    if (calibrationName == NULL)
        calibrationName = "default";

    String path = GetBaseOVRPath(true);
    path += "/Devices.json";

    // Load the device profiles
    Ptr<JSON> root = *JSON::Load(path);
    if (root == NULL)
        return false;

    // Quick sanity check of the file type and format before we parse it
    JSON* version = root->GetFirstItem();
    if (version && version->Name == "Oculus Device Profile Version")
    {   
        int major = atoi(version->Value.ToCStr());
        if (major > MAX_DEVICE_PROFILE_MAJOR_VERSION)
            return false;   // don't parse the file on unsupported major version number
    }
    else
    {
        return false;
    }

    bool autoEnableCorrection = false;    

    JSON* device = root->GetNextItem(version);
    while (device)
    {   // Search for a previous calibration with the same name for this device
        // and remove it before adding the new one
        if (device->Name == "Device")
        {   
            JSON* item = device->GetItemByName("Serial");
            if (item && item->Value == CachedSensorInfo.SerialNumber)
            {   // found an entry for this device

                JSON* autoyaw = device->GetItemByName("EnableYawCorrection");
                if (autoyaw)
                    autoEnableCorrection = (autoyaw->dValue != 0);

                int maxCalibrationVersion = 0;
                item = device->GetNextItem(item);
                while (item)
                {
                    if (item->Name == "MagCalibration")
                    {   
                        JSON* calibration = item;
                        JSON* name = calibration->GetItemByName("Name");
                        if (name && name->Value == calibrationName)
                        {   // found a calibration with this name
                            
                            int major = 0;
                            JSON* version = calibration->GetItemByName("Version");
                            if (version)
                                major = atoi(version->Value.ToCStr());

                            if (major > maxCalibrationVersion && major <= 2)
                            {
                                time_t now;
                                time(&now);

                                // parse the calibration time
                                time_t calibration_time = now;
                                JSON* caltime = calibration->GetItemByName("Time");
                                if (caltime)
                                {
                                    const char* caltime_str = caltime->Value.ToCStr();

                                    tm ct;
                                    memset(&ct, 0, sizeof(tm));
                            
#ifdef OVR_OS_WIN32
                                    struct tm nowtime;
                                    localtime_s(&nowtime, &now);
                                    ct.tm_isdst = nowtime.tm_isdst;
                                    sscanf_s(caltime_str, "%d-%d-%d %d:%d:%d", 
                                        &ct.tm_year, &ct.tm_mon, &ct.tm_mday,
                                        &ct.tm_hour, &ct.tm_min, &ct.tm_sec);
#else
                                    struct tm* nowtime = localtime(&now);
                                    ct.tm_isdst = nowtime->tm_isdst;
                                    sscanf(caltime_str, "%d-%d-%d %d:%d:%d", 
                                        &ct.tm_year, &ct.tm_mon, &ct.tm_mday,
                                        &ct.tm_hour, &ct.tm_min, &ct.tm_sec);
#endif
                                    ct.tm_year -= 1900;
                                    ct.tm_mon--;
                                    calibration_time = mktime(&ct);
                                }
                                                        
                                // parse the calibration matrix
                                JSON* cal = calibration->GetItemByName("CalibrationMatrix");
                                if (cal == NULL)
                                    cal = calibration->GetItemByName("Calibration");
                               
                                if (cal)
                                {
                                    Matrix4f calmat = Matrix4f::FromString(cal->Value.ToCStr());
                                    SetMagCalibration(calmat);
                                    MagCalibrationTime  = calibration_time;
                                    EnableYawCorrection = autoEnableCorrection;

                                    maxCalibrationVersion = major;
                                }
                            }
                        } 
                    }
                    item = device->GetNextItem(item);
                }

                return (maxCalibrationVersion > 0);
            }
        }

        device = root->GetNextItem(device);
    }
    
    return false;
}



} // namespace OVR

