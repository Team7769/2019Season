package frc.robot.Configuration;

public final class Constants {

    //Spark Device ID's
    public static final int kLFDriveId = 5;
    public static final int kLMDriveId = 5;
    public static final int kLRDriveId = 6;
    public static final int kRFDriveId = 2;
    public static final int kRMDriveId = 6;
    public static final int kRRDriveId = 3;
    public static final int kLArmId = 6;
    public static final int kRArmId = 7;
    public static final int kLElevatorId = 8;
    public static final int kRElevatorId = 9;
    public static final int kTCollectorId = 10;
    public static final int kBCollectorId = 11;
    public static final int kCollectorSolenoidSlot = 0;
    public static final int kHab3SolenoidSlot = 1;

    //Controller Usb Slots
    public static final int kDriverControllerSlot = 0;
    public static final int kOperatorControllerSlot = 0;

    //Elevator Setpoints
    public static final double kElevatorLowHatch = 0;
    public static final double kElevatorLowCargo = 1;
    public static final double kElevatorMidHatch = 2;
    public static final double kElevatorMidCargo = 3;
    public static final double kElevatorTopCargo = 4;

    //Arm Setpoints
    public static final double kArmLowHatch = 0;
    public static final double kArmLowCargo = 1;
    public static final double kArmMidHatch = 2;
    public static final double kArmMidCargo = 3;
    public static final double kArmTopCargo = 4;

    //Collector speed
    public static final double kCollectorIntakeSpeed = 0.5;
    public static final double kCollectorReleaseSpeed = -0.5;

    //PID Coefficients
    public static final double kDistancePIDP = 0.01;
    public static final double kDistancePIDI = 0.0001;
    public static final double kDistancePIDD = 0.75;
    public static final double kVelocityPIDP = 0.00025;
    public static final double kVelocityPIDI = 0.0;
    public static final double kVelocityPIDD = 0.0;
    public static final double kVelocityPIDFF = 0.0;

    public static final double kDriveRotationP = 0.014;
    public static final double kDriveRotationI = 0.00000001;
    public static final double kDriveRotationD = 0.0;
    public static final double kDriveRotationTolerance = 0.125; //1 Degree
    
    //Elevator
    public static final double kElevatorP = 0.00025;
    public static final double kElevatorI = 0.0;
    public static final double kElevatorD = 0.0;
    public static final double kElevatorFF = 0.0;

    //Arm
    public static final double kArmP = 0.00025;
    public static final double kArmI = 0.0;
    public static final double kArmD = 0.0;
    public static final double kArmFF = 0.0;

    public static final double kDriveWheelDiameter = 6.0;
    public static final double kDriveVelocityConversion = Math.PI / 107.1; //Rev/Min -> Ft/sec
    public static final double kDrivePositionConversion = 6 * Math.PI / 10.71; 

    public static final boolean kIsTestRobot = true;
        
}