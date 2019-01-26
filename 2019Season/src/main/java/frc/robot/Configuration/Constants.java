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

    public static final boolean kIsTestRobot = true;
        
}