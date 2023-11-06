#include "Source.h"

// Default constructor for the DeviceController class
DeviceController::DeviceController()
{
    keyHandleA = 0;
    keyHandleD = 0;
    success_key = 0;
    setProtA = 0;
    setProtD = 0;
    success_prot = 0;
    errorCode = 0;
    Baudrate = 1000000;  // [bit/s]
    Timeout = 500;       // [ms]
    MotorType = 11;        // EC motor block commutated
    NominalCurrent = 0;       // Maximal continuous current
    MaxOutputCurrent = 0;     // Maximal peak current
    ThermalTimeConstant = 0;   // Thermal time constant windint
    NbOfPolePairs = 0;         // Number of pole pairs
    sensorType = 2;        // incremental encoder 1 without index (2-channel)
    motorNo = 0;            // Set motor as 633399 or 658677
    nodeId = 1;
    //nodeIdD = 2;


}

DeviceController::~DeviceController() {}

void DeviceController::tuyendeptrai()
{
    std::cout << " Tuyen dep trai vl " << std::endl ;
}


void DeviceController::configureDevice()
{
    // Configure the communication settings

    char deviceName[] = "EPOS4";
    char protocolStackName[] = "MAXON SERIAL V2";
    char interfaceName[] = "USB";
    char portNameA[] = "USB1";
    char portNameD[] = "USB0";
    findDevSet = false;

    std::cout << "Which motor you use?\n1: 633399, 2: 658677\n";
    std::cin >> motorNo;

    if (motorNo == 1 || motorNo == 2) {
        if (motorNo == 1) {
            NominalCurrent = 5140;
            MaxOutputCurrent = 5000;
            ThermalTimeConstant = 414;
            NbOfPolePairs = 7;
        }
        else if (motorNo == 2) {
            NominalCurrent = 2610;
            MaxOutputCurrent = 2500;
            ThermalTimeConstant = 414;
            NbOfPolePairs = 7;
        }

        std::cout << "Choose the function to open the device:\n";
        std::cout << "1. VCS_OpenDevice\n";
        std::cout << "2. VCS_OpenDeviceDlg\n";
        std::cout << "Enter your choice (1 or 2): ";
        int functionChoice;
        std::cin >> functionChoice;

        if (functionChoice == 1)
        {
            keyHandleA = VCS_OpenDevice(deviceName, protocolStackName, interfaceName, portNameA, &errorCode);
            keyHandleD = VCS_OpenDevice(deviceName, protocolStackName, interfaceName, portNameD, &errorCode);
        }
        else if (functionChoice == 2)
        {
            keyHandleA = VCS_OpenDeviceDlg(&errorCode);
            keyHandleD = VCS_OpenDeviceDlg(&errorCode);
        }
        else
        {
            std::cerr << "Invalid choice, exiting..." << std::endl;
            exit(1);
        }

        success_key = (keyHandleA != NULL) && (keyHandleD != NULL);

        if (success_key) {
            setProtA = VCS_SetProtocolStackSettings(keyHandleA, Baudrate, Timeout, &errorCode);
            setProtD = VCS_SetProtocolStackSettings(keyHandleD, Baudrate, Timeout, &errorCode);
            success_prot = setProtA * setProtD;

            if (success_prot) {
                VCS_ClearFault(keyHandleA, nodeId, &errorCode);
                VCS_ClearFault(keyHandleD, nodeId, &errorCode);

                VCS_SetMotorType(keyHandleA, nodeId, MotorType, &errorCode);
                VCS_SetMotorType(keyHandleD, nodeId, MotorType, &errorCode);

                VCS_SetEcMotorParameter(keyHandleA, nodeId, NominalCurrent,
                    MaxOutputCurrent, ThermalTimeConstant,
                    NbOfPolePairs, &errorCode);
                VCS_SetEcMotorParameter(keyHandleD, nodeId, NominalCurrent,
                    MaxOutputCurrent, ThermalTimeConstant,
                    NbOfPolePairs, &errorCode);

                VCS_SetSensorType(keyHandleA, nodeId, sensorType, &errorCode);
                VCS_SetSensorType(keyHandleD, nodeId, sensorType, &errorCode);

                std::cout << "Successfully opened both motors.\n";
                printMotorParameters(NominalCurrent, MaxOutputCurrent, ThermalTimeConstant, NbOfPolePairs);

                VCS_GetPositionIs(keyHandleA, nodeId, &PositionIsA, &errorCode);  // Current angle of motor A
                VCS_GetPositionIs(keyHandleD, nodeId, &PositionIsD, &errorCode);  // Current angle of motor D
                AngleOriginA_qc = PositionIsA;
                AngleOriginD_qc = PositionIsD;
                std::cout << "Let the current angleA " << AngleOriginA_qc << " [qc] be 0 deg.\n";
                std::cout << "Let the current angleD " << AngleOriginD_qc << " [qc] be 0 deg.\n";

            }           
        }
        else if (keyHandleA != NULL) {
            std::cout << "Successfully opened motor A.\n";
            printMotorParameters(NominalCurrent,MaxOutputCurrent,ThermalTimeConstant,NbOfPolePairs);
        }
        else if (keyHandleD != NULL) {
            std::cout << "Successfully opened motor D.\n";
            printMotorParameters(NominalCurrent, MaxOutputCurrent, ThermalTimeConstant, NbOfPolePairs);
        }
    }
    else {
        std::cout << "Could not open both motors.\n";
        if (keyHandleA != NULL) {
            std::cout << "Motor A is open.\n";
            printMotorParameters(NominalCurrent,
                MaxOutputCurrent,
                ThermalTimeConstant,
                NbOfPolePairs);
        }
        if (keyHandleD != NULL) {
            std::cout << "Motor D is open.\n";
            printMotorParameters(NominalCurrent,
                MaxOutputCurrent,
                ThermalTimeConstant,
                NbOfPolePairs);
        }
    }
}

void DeviceController::printMotorParameters(int NominalCurrent,
    int MaxOutputCurrent,
    int ThermalTimeConstant,
    int NbOfPolePairs)
{
    std::cout << "Motor Parameters:\n";
    std::cout << "Nominal Current: " << NominalCurrent << "\n";
    std::cout << "Max Output Current: " << MaxOutputCurrent << "\n";
    std::cout << "Thermal Time Constant: " << ThermalTimeConstant << "\n";
    std::cout << "Number of Pole Pairs: " << NbOfPolePairs << "\n";
}

    void DeviceController::selectOperationMode()
{
    if (keyHandleA != NULL && keyHandleD != NULL)
    {
        int operationMode;

        while (true)
        {
            std::cout << "\n";
            std::cout << "Select operation mode (1: PPM, 3: PVM, 6: HM, 7: IPM, -1: PM, -2: VM, -3: CM, -5: MEM, -6: SDM): ";
            std::cin >> operationMode;

            BOOL successA = VCS_SetOperationMode(keyHandleA, nodeId, operationMode, &errorCode);
            BOOL successD = VCS_SetOperationMode(keyHandleD, nodeId, operationMode, &errorCode);

            if (successA)
            {
                std::cout << "Successfully set operation mode for motor A. " << std::endl;
            }
            if (successD)
            {
                std::cout << "Successfully set operation mode for motor D. " << std::endl;
            }

            if (successA && successD)
            {
                system("CLS");
                std::cout << "Operation mode set successfully for both motors!" << std::endl;
                std::cout << "Begin Mode setting" << std::endl;
                
                switch (operationMode)
                {
                case 1: // PPM
                    handleProfilePositionMode();
                    break;
                case 3: // PVM
                    handleProfileVelocityMode();
                    break;
                case 6: // HM
                    //handleHomingMode();
                    break;
                case 7: // IPM
                    //handleInterpolatedPositionMode();
                    break;
                case -1: // PM
                    //handlePositionMode();
                    break;
                case -2: // VM
                    //handleVelocityMode();
                    break;
                case -3: // CM
                    handleCurrentMode();
                    break;
                case -5: // MEM
                    //handleMasterEncoderMode();
                    break;
                case -6: // SDM
                    //handleStepDirectionMode();
                    break;
                }
                

                __int8 currentModeA;
                __int8 currentModeD;

                successA = VCS_GetOperationMode(keyHandleA, nodeId, &currentModeA, &errorCode);
                successD = VCS_GetOperationMode(keyHandleD, nodeId, &currentModeD, &errorCode);

                if (successA && successD)
                {
                    std::cout << "Current operation mode for motor A: " << (int)currentModeA << std::endl;
                    std::cout << "Current operation mode for motor D: " << (int)currentModeD << std::endl;
                }
                else
                {
                    if (!successA) {
                        std::cerr << "Failed to set operation mode for motor A, error code: " << errorCode << std::endl;
                        ShowErrorInformation(errorCode);
                    }
                    if (!successD) {
                        std::cerr << "Failed to set operation mode for motor D, error code: " << errorCode << std::endl;
                        ShowErrorInformation(errorCode);
                    }
                }
            }
            else
            {
                if (!successA) {
                    std::cerr << "Failed to set operation mode for motor A, error code: " << errorCode << std::endl;
                    ShowErrorInformation(errorCode);
                }
                if (!successD) {
                    std::cerr << "Failed to set operation mode for motor D, error code: " << errorCode << std::endl;
                    ShowErrorInformation(errorCode);
                }
            }

            std::cout << "\n";
            std::cout << "Select an option:\n";
            std::cout << "1. Try again\n";
            std::cout << "2. Close device\n";
            std::cout << "Enter your choice (1 or 2): ";
            int choice;
            std::cin >> choice;

            switch (choice) {
            case 1:
                continue;
            case 2:
                closeDevice();
                exit(0);
            default:
                std::cerr << "Invalid choice, please try again.\n";
                break;
            }
        }
    }
}

    void DeviceController::closeDevice()
    {
        if (keyHandleA != NULL && keyHandleD != NULL)
        {
            DWORD errorCode = 0;

            // Ask the user for the choice to close the device
            int closeChoice;
            std::cout << "\n";
            std::cout << "Choose the function to close the device:\n";
            std::cout << "1. VCS_CloseDevice (close only the current device)\n";
            std::cout << "2. VCS_CloseAllDevices (close all opened devices)\n";
            std::cout << "Enter your choice (1 or 2): ";

            while (!(std::cin >> closeChoice) || (closeChoice != 1 && closeChoice != 2))
            {
                std::cin.clear();
                std::cin.ignore(1000, '\n');
                std::cerr << "Invalid input. Please enter 1 or 2: ";
            }

            BOOL closeSuccessA = false;
            BOOL closeSuccessD = false;

            if (closeChoice == 1)
            {
                VCS_ResetDevice(keyHandleA, nodeId, &errorCode);
                VCS_ResetDevice(keyHandleD, nodeId, &errorCode);
                VCS_SetDisableState(keyHandleA, nodeId, &errorCode);
                VCS_SetDisableState(keyHandleD, nodeId, &errorCode);
                closeSuccessA = VCS_CloseDevice(keyHandleA, &errorCode);
                closeSuccessD = VCS_CloseDevice(keyHandleD, &errorCode);
            }
            else if (closeChoice == 2)
            {
                VCS_ResetDevice(keyHandleA, nodeId, &errorCode);
                VCS_ResetDevice(keyHandleD, nodeId, &errorCode);
                VCS_SetDisableState(keyHandleA, nodeId, &errorCode);
                VCS_SetDisableState(keyHandleD, nodeId, &errorCode);
                closeSuccessA = VCS_CloseAllDevices(&errorCode);
                closeSuccessD = VCS_CloseAllDevices(&errorCode);
            }

            if (closeSuccessA && closeSuccessD)
            {
                std::cout << "The device(s) closed successfully!" << std::endl;
            }
            else
            {
                std::cerr << "Failed to close device(s), error code: " << errorCode << std::endl;
                ShowErrorInformation(errorCode);
            }

            // Clear device handle
            keyHandleA = NULL;
            keyHandleD = NULL;
        }
        else
        {
            std::cout << "Device is not open, nothing to close." << std::endl;
        }
    }

// ----------------------------------------------------------------------------------------------------------------------------------------------------------
//                                             Basic Motion Control Functions
// ----------------------------------------------------------------------------------------------------------------------------------------------------------

// Handles the Profile Position Mode by activating it, setting position profile, enabling/disabling position window, and moving to a target position
    float DeviceController::convertPositionToAngle(long position)
    {
        return static_cast<double>((position) / 16384.0) * 360.0;
    }

    long DeviceController::convertAngleToPosition(float angle)
    {
        return static_cast<long>((angle / 360.0) * 16384.0);
    }

    void DeviceController::printCurrentPosition()
    {
        long currentPositionA = 0;
        long currentPositionD = 0;
        BOOL successA = VCS_GetPositionIs(keyHandleA, nodeId, &currentPositionA, &errorCode);
        BOOL successD = VCS_GetPositionIs(keyHandleD, nodeId, &currentPositionD, &errorCode);

        if (successA && successD)
        {
            float currentAngleA = convertPositionToAngle(currentPositionA);
            float currentAngleD = convertPositionToAngle(currentPositionD);
            std::cout << "Current position of Motor A: " << currentAngleA << " degrees" << std::endl;
            std::cout << "Current position of Motor B: " << currentAngleD << " degrees" << std::endl;
        }
        else
        {
            std::cerr << "Failed to get current position, error code: " << errorCode << std::endl;
            ShowErrorInformation(errorCode);
        }
    }


    /// <summary>
    /// Profile position mode
    /// </summary>
    void DeviceController::handleProfilePositionMode()
    {

        DWORD errorCode = 0;

        // Activate Profile Position Mode for both motors
        BOOL successA = VCS_SetOperationMode(keyHandleA, nodeId, 1, &errorCode);
        BOOL successD = VCS_SetOperationMode(keyHandleD, nodeId, 1, &errorCode);

        if (successA && successD) {
            std::cout << "\n";
            std::cout << "Profile Position Mode activated for both motors." << std::endl;
        }
        else {
            std::cerr << "Failed to activate Profile Position Mode, error code: " << errorCode << std::endl;
            ShowErrorInformation(errorCode);
            return;
        }

        // Set the position profile for both motors
        DWORD profileVelocity = 2500;
        DWORD profileAcceleration = 200;
        DWORD profileDeceleration = 200;

        successA = VCS_SetPositionProfile(keyHandleA, nodeId, profileVelocity, profileAcceleration, profileDeceleration, &errorCode);
        successD = VCS_SetPositionProfile(keyHandleD, nodeId, profileVelocity, profileAcceleration, profileDeceleration, &errorCode);

        if (successA && successD) {
            std::cout << "Position profile set for both motors." << std::endl;
        }
        else {
            std::cerr << "Failed to set position profile for one or both motors, error code: " << errorCode << std::endl;
            ShowErrorInformation(errorCode);
            return;
        }

        // Get target angle from user in degrees
        float targetAngle = 0;
        std::cout << "Enter the target angle in degrees: ";
        std::cin >> targetAngle;

        // Convert target angle to motor position value
        long targetPosition = convertAngleToPosition(targetAngle);

        BOOL absolute = TRUE;
        BOOL immediately = TRUE;

        successA = VCS_MoveToPosition(keyHandleA, nodeId, targetPosition, absolute, immediately, &errorCode);
        successD = VCS_MoveToPosition(keyHandleD, nodeId, targetPosition, absolute, immediately, &errorCode);

        if (successA && successD) {
            std::cout << "Moving both motors to target position.\n" << std::endl;
        }
        else {
            std::cerr << "Failed to move to target position, error code: " << errorCode << std::endl;
            ShowErrorInformation(errorCode);
        }



        // Wait for the motors to reach the target position
        DWORD timeout = 10000; // Timeout in milliseconds, adjust as needed
        successA = VCS_WaitForTargetReached(keyHandleA, nodeId, timeout, &errorCode);
        successD = VCS_WaitForTargetReached(keyHandleD, nodeId, timeout, &errorCode);

        if (successA && successD) {
            std::cout << "Target position reached by both motors." << std::endl;
        }
        else {
            std::cerr << "Failed to reach target position by one or both motors, error code: " << errorCode << std::endl;
            ShowErrorInformation(errorCode);
        }

        printCurrentPosition();
    }

    /// <summary>
    /// Profile velocity mode
    /// </summary>
    void DeviceController::handleProfileVelocityMode()
    {
        DWORD errorCode = 0;

        // Activate Profile Velocity Mode for both motors
        BOOL successA = VCS_SetOperationMode(keyHandleA, nodeId, 3, &errorCode);
        BOOL successD = VCS_SetOperationMode(keyHandleD, nodeId, 3, &errorCode);

        if (successA && successD) {
            std::cout << "\n";
            std::cout << "Profile Velocity Mode activated for both motors." << std::endl;
        }
        else {
            std::cerr << "Failed to activate Profile Velocity Mode, error code: " << errorCode << std::endl;
            ShowErrorInformation(errorCode);
            return;
        }

        // Enable the device for both motors
        BOOL enabledA = VCS_SetEnableState(keyHandleA, nodeId, &errorCode);
        BOOL enabledD = VCS_SetEnableState(keyHandleD, nodeId, &errorCode);

        if (enabledA && enabledD) {
            std::cout << "Both devices Enabled" << std::endl;
        }
        else {
            std::cerr << "Failed to Enable Devices, error code: " << errorCode << std::endl;
            ShowErrorInformation(errorCode);
            return;
        }

        // Set the velocity profile
        DWORD profileAcceleration = 200;
        DWORD profileDeceleration = 200;

        successA = VCS_SetVelocityProfile(keyHandleA, nodeId, profileAcceleration, profileDeceleration, &errorCode);
        successD = VCS_SetVelocityProfile(keyHandleD, nodeId, profileAcceleration, profileDeceleration, &errorCode);

        if (successA && successD) {
            std::cout << "Velocity profile set for both motors." << std::endl;
        }
        else {
            std::cerr << "Failed to set position profile for one or both motors, error code: " << errorCode << std::endl;
            ShowErrorInformation(errorCode);
            return;
        }

        // Get the velocity profile
        successA = VCS_GetVelocityProfile(keyHandleA, nodeId, NULL, NULL, &errorCode);
        successD = VCS_GetVelocityProfile(keyHandleD, nodeId, NULL, NULL, &errorCode);

        if (successA && successD) {
            std::cout << "Velocity profile retrieved.\n\n" << std::endl;
        }
        else {
            std::cerr << "Failed to retrieve velocity profile, error code: " << errorCode << std::endl;
            ShowErrorInformation(errorCode);
            return;
        }


        // Get target velocity from user in rpm
        float targetVelocity = 0;
        std::cout << "Enter the target velocity in rpm: ";
        while (!(std::cin >> targetVelocity)) {
            std::cin.clear();
            std::cin.ignore(1000, '\n');
            std::cerr << "Invalid input. Please enter a valid velocity: ";
        }

        // Convert target velocity to motor velocity value
        long targetMotorVelocity = static_cast<long>((targetVelocity / 60) * 4096);

        // Move both motors with target velocity
        successA = VCS_MoveWithVelocity(keyHandleA, nodeId, targetMotorVelocity, &errorCode);
        successD = VCS_MoveWithVelocity(keyHandleD, nodeId, targetMotorVelocity, &errorCode);

        if (successA && successD) {
            std::cout << "Moving both motors with target velocity.\n" << std::endl;

            // Wait for the motors to move for a sufficient amount of time
            DWORD waitTime = 5000; // Wait for 5 seconds, adjust as needed
            Sleep(waitTime);

            // Stop the motors
            successA = VCS_HaltVelocityMovement(keyHandleA, nodeId, &errorCode);
            successD = VCS_HaltVelocityMovement(keyHandleD, nodeId, &errorCode);

            if (successA && successD) {
                std::cout << "Movement stopped for both motors." << std::endl;
            }
            else {
                std::cerr << "Failed to stop movement for one or both motors, error code: " << errorCode << std::endl;
                ShowErrorInformation(errorCode);
                return;
            }
        }
        else {
            std::cerr << "Failed to move one or both motors with target velocity, error code: " << errorCode << std::endl;
            ShowErrorInformation(errorCode);
            return;
        }
    }


    /// <summary>
    /// Cyclic synchronous current mode
    /// </summary>
    void DeviceController::handleCurrentMode()
    {
        DWORD errorCode = 0;
        WORD AnalogValueA = 0;
        WORD AnalogValueD = 0;
        long VoltageValueA = 0;
        long VoltageValueD = 0;
        long StateValueA = 0;
        long StateValueD = 0;

        // Get the current angle of both motors
        VCS_GetPositionIs(keyHandleA, nodeId, &PositionIsA, &errorCode);  // Current angle of motor A
        VCS_GetPositionIs(keyHandleD, nodeId, &PositionIsD, &errorCode);  // Current angle of motor D
        AngleOriginA_qc = PositionIsA;
        AngleOriginA_deg=convertPositionToAngle(AngleOriginA_qc);
        AngleOriginD_qc = PositionIsD;
        AngleOriginD_deg = convertPositionToAngle(AngleOriginD_qc);
        std::cout << "Let the current angleA " << AngleOriginA_qc << " [qc] or " << AngleOriginA_deg <<"deg.\n";
        std::cout << "Let the current angleD " << AngleOriginD_qc << " [qc] or " << AngleOriginD_deg << "deg.\n";
        deltaTheta_qc = 0;
        deltaTheta = 0;

        // Activate Cyclic synchronous current mode for both motors
        BOOL successA = VCS_SetOperationMode(keyHandleA, nodeId, -3, &errorCode);
        BOOL successD = VCS_SetOperationMode(keyHandleD, nodeId, -3, &errorCode);

        if (successA && successD) {
            std::cout << "\n";
            std::cout << "Cyclic synchronous current mode activated for both motors." << std::endl;
        }
        else {
            std::cerr << "Failed to activate Cyclic synchronous current mode, error code: " << errorCode << std::endl;
            ShowErrorInformation(errorCode);
            return;
        }

        // Use analog input terminal 1, 100 mA, offset 0 mA
        successA = VCS_ActivateAnalogCurrentSetpoint(keyHandleA, nodeId, 1, (float)100.0, (short)0, &errorCode);
        successD = VCS_ActivateAnalogCurrentSetpoint(keyHandleD, nodeId, 1, (float)100.0, (short)0, &errorCode);

        if (successA && successD) {
            std::cout << "Analog current setpoint activated for both motors." << std::endl;
        }
        else {
            std::cerr << "Failed to activate analog current setpoint, error code: " << errorCode << std::endl;
            ShowErrorInformation(errorCode);
            return;
        }

        // Enable analog current setpoint for both motors
        /*
        successA = VCS_EnableAnalogCurrentSetpoint(keyHandleA, nodeId, &errorCode);
        if (!successA) {
            std::cerr << "Failed to enable analog current setpoint for Motor A, error code: " << errorCode << std::endl;
            ShowErrorInformation(errorCode);
            return;
        }
        else {
            std::cout << "Analog current setpoint enabled for motor A." << std::endl;

        }
        */
        successD = VCS_EnableAnalogCurrentSetpoint(keyHandleD, nodeId, &errorCode);
        if (!successD) {
            std::cerr << "Failed to enable analog current setpoint for Motor D, error code: " << errorCode << std::endl;
            ShowErrorInformation(errorCode);
            return;
        }
        else {
            std::cout << "Analog current setpoint enabled for motor D." << std::endl;
            
        }
        
        /*
        if (successA && successD) {
            std::cout << "Analog current setpoint enabled for both motors." << std::endl;
        }
        */
        while (1) {
            long VoltageValueA = 0;
            long VoltageValueD = 0;
            //VCS_GetAnalogInputVoltage(keyHandleA, nodeId, 2, &VoltageValueA, &errorCode);
            VCS_GetAnalogInputVoltage(keyHandleD, nodeId, 2, &VoltageValueD, &errorCode);
            
            std::cout << "currentvoltage of Motor A= " << VoltageValueA << std::endl;
            std::cout << "currentvoltage of Motor D= " << VoltageValueD << std::endl;
            
            if (VoltageValueD > 1000) {
                // Print motor angles and difference
                printMotorAnglesAndDifference();

                /*
                // Analog out and analog in
                long OutVoltage = 10000 / 360 * ((long)angleA) - 5000;
                OutVoltage = 10000 / 360 * ((long)angleD) - 5000;
                VCS_SetAnalogOutputVoltage(keyHandleA, nodeIdA, 1, OutVoltage, &errorCode);
                VCS_SetAnalogOutputVoltage(keyHandleD, nodeIdD, 1, OutVoltage, &errorCode);
                long AnalogValue = 0;
                VCS_GetAnalogInput(keyHandleD, nodeIdD, 1, &AnalogValue, &errorCode);
                VCS_GetAnalogInputVoltage(keyHandleD, nodeIdD, 1, &VoltageValue, &errorCode);
                printf_s("Analog Input is %d\n", AnalogValue);
                printf_s("Analog Input Voltage is %ld [mV]\n", VoltageValue);
                */

                Sleep(1000); // Wait for a second before the next iteration
            }
            else {
                // safety button
                VCS_DeactivateAnalogCurrentSetpoint(keyHandleD, nodeId, 1, &errorCode);
                VCS_DisableAnalogCurrentSetpoint(keyHandleD, nodeId, &errorCode);
                std::cout << "Operation is stopped because the switch is pressed.\n";
                break;
            }
        }
    }

    void DeviceController::printMotorAnglesAndDifference()
    {
        long currentPositionA = 0;
        long currentPositionD = 0;
        DWORD errorCode = 0;
        BOOL successA = VCS_GetPositionIs(keyHandleA, nodeId, &currentPositionA, &errorCode);
        BOOL successD = VCS_GetPositionIs(keyHandleD, nodeId, &currentPositionD, &errorCode);

        if (successA && successD)
        {
            float currentAngleA = convertPositionToAngle(currentPositionA);
            float currentAngleD = convertPositionToAngle(currentPositionD);
            std::cout << "Current angle of Motor A: " << currentAngleA << " degrees" << std::endl;
            std::cout << "Current angle of Motor D: " << currentAngleD << " degrees" << std::endl;

            float angleDifference = currentAngleA - currentAngleD;
            // Adjust the difference to be within -180 to 180 degrees
            while (angleDifference > 180) angleDifference -= 360;
            while (angleDifference < -180) angleDifference += 360;

            std::cout << "Angle difference between Motor A and Motor B: " << angleDifference << " degrees" << std::endl;
        }
        else
        {
            std::cerr << "Failed to get current position, error code: " << errorCode << std::endl;
            ShowErrorInformation(errorCode);
        }
    }


    

    BOOL DeviceController::ShowErrorInformation(DWORD ulErrorCode)
    {
        char* pErrorInfo;
        std::string Description;

        if ((pErrorInfo = (char*)malloc(100)) == NULL)
        {
            std::cout << "Not enough memory to allocate buffer for error information string\n" << "System Error";
            return FALSE;
        }

        if (VCS_GetErrorInfo(ulErrorCode, pErrorInfo, 100))
        {
            Description = pErrorInfo;
            std::cout << Description << std::endl;
            free(pErrorInfo);
        }
        else
        {
            free(pErrorInfo);
            std::cout << "Error information can't be read!" << std::endl;
            return FALSE;
        }

        return TRUE;
    }