package frc.robot.Utilities;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.Configuration.*;

public class RobotMap {

    //6 Sparks
    //6 TalonSRXs (2 Arm) (2 Ele) (2 Collector)
    private CANSparkMax _lfDrive;
    private CANSparkMax _lmDrive;
    private CANSparkMax _lrDrive;
    private CANSparkMax _rfDrive;
    private CANSparkMax _rmDrive;
    private CANSparkMax _rrDrive;
    private TalonSRX _leftArm;
    private TalonSRX _rightArm;
    private TalonSRX _leftElevator;
    private TalonSRX _rightElevator;
    private TalonSRX _topCollector;
    private TalonSRX _bottomCollector;

    public RobotMap(){
        _lfDrive = new CANSparkMax(Constants.kLFDriveId, MotorType.kBrushless);
        
        _lmDrive = new CANSparkMax(Constants.kLMDriveId, MotorType.kBrushless);
        _lmDrive.follow(_lfDrive);

        _lrDrive = new CANSparkMax(Constants.kLRDriveId, MotorType.kBrushless);
        _lrDrive.follow(_lfDrive);

        _rfDrive = new CANSparkMax(Constants.kRFDriveId, MotorType.kBrushless);

        _rmDrive = new CANSparkMax(Constants.kRMDriveId, MotorType.kBrushless);
        _rmDrive.follow(_rfDrive);

        _rrDrive = new CANSparkMax(Constants.kRRDriveId, MotorType.kBrushless);
        _rrDrive.follow(_rfDrive);

        if (!Constants.kIsTestRobot){
            _leftArm = new TalonSRX(Constants.kLArmId);
            _rightArm = new TalonSRX(Constants.kRArmId);
            _leftElevator = new TalonSRX(Constants.kLElevatorId);
            _rightElevator = new TalonSRX(Constants.kRElevatorId);
            _topCollector = new TalonSRX(Constants.kTCollectorId);
            _bottomCollector = new TalonSRX(Constants.kBCollectorId);
        }
        
    }
    public CANSparkMax getLeftDriveSpark(){
        return _lfDrive;
    }
    public CANSparkMax getRightDriveSpark(){
        return _rfDrive;
    }
    public TalonSRX getLeftArmTalon(){
        return _leftArm;
    }
    public TalonSRX getRightArmTalon(){
        return _rightArm;
    }
    public TalonSRX getLeftElevatorTalon(){
        return _leftElevator;
    }
    public TalonSRX getRightElevatorTalon(){
        return _rightElevator;
    }
    public TalonSRX getTopCollectorTalon(){
        return _topCollector;
    }
    public TalonSRX getBottomCollectorTalon(){
        return _bottomCollector;
    }
}