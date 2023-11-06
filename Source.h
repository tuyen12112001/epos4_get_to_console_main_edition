#include "Definitions.h"

#define _USE_MATH_DEFINES
#include <math.h>
#include <chrono>
#include <thread>
#include <iostream>

class DeviceController
{
public:
    DeviceController();
    ~DeviceController();

    void tuyendeptrai();
    void configureDevice();
    void selectOperationMode();
    void closeDevice();
    BOOL ShowErrorInformation(DWORD ulErrorCode);
    void printMotorParameters(int NominalCurrent,
        int MaxOutputCurrent,
        int ThermalTimeConstant,
        int NbOfPolePairs);

    float convertPositionToAngle(long position);

    long convertAngleToPosition(float angle);

    void printCurrentPosition();
    void printMotorAnglesAndDifference();
    
private:
    // Private member variables
    HANDLE keyHandleA;
    HANDLE keyHandleD;
    BOOL success_key;
    BOOL setProtA;
    BOOL setProtD;
    BOOL success_prot;
    char* deviceName;
    char* protocolStackName;
    char* interfaceName;
    char* portNameA;
    char* portNameD;
    bool findDevSet;
    DWORD errorCode;
    DWORD Baudrate;  // [bit/s]
    DWORD Timeout;       // [ms]
    WORD MotorType;        // EC motor block commutated
    DWORD NominalCurrent;       // Maximal continuous current
    DWORD MaxOutputCurrent;     // Maximal peak current
    WORD ThermalTimeConstant;   // Thermal time constant windint
    BYTE NbOfPolePairs;         // Number of pole pairs
    WORD sensorType;        // incremental encoder 1 without index (2-channel)
    int motorNo;            // Set motor as 633399 or 658677
    WORD nodeId;
    //WORD nodeIdD;

    BOOL Absolute;      // TRUE: Absolute　FALSE: Relative
    BOOL Immediately;   // TRUE: Start immediately　FALSE: Wait until the lasti positioning is completed


    WORD AnalogValue = 0;
    long VoltageValueA = 0;// For analog read of motor A
    long VoltageValueD = 0;// For analog read of motor D
    long OutVoltageA = 0;        // For analog out of motor A
    long OutVoltageD = 0;        // For analog out of motor D
    long PositionIsA = 0;       // Actual angle[qc] of motor A
    long PositionIsD = 0;       // Actual angle[qc] of motor D
    long AngleOriginA_qc = 0;      // Angle origin of motor A
    long AngleOriginA_deg = 0;      // Angle origin of motor A (degree)
    long AngleOriginD_qc = 0;      // Angle origin of motor D
    long AngleOriginD_deg = 0;      // Angle origin of motor D (degree)
    float angleA = 0;           // Actual angle (0-360 [degree]) of motor A
    float angleD = 0;           // Actual angle (0-360 [degree]) of motor D
    int rotA = 0;               // Number of times motor A has rotated
    int rotD = 0;               // Number of times motor D has rotated
    float deltaTheta_qc = 0;     // Angle difference [qc]: A-D
    float deltaTheta = 0;       // Angle difference [degree]

    // Private member functions
    
    void handleProfilePositionMode();
    void handleProfileVelocityMode();
    //void handleInterpolatedPositionMode();
    //void handleHomingMode();
    //void handlePositionMode();
    //void handleVelocityMode();
    void handleCurrentMode();
    //void handleMasterEncoderMode();
    //void handleStepDirectionMode();
};
