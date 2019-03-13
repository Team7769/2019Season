package frc.robot.Utilities;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.SPI.Port;
import frc.robot.Configuration.*;

public class RobotMap {

    //6 Sparks
    //5 TalonSRXs (2 Arm) (1 Ele) (2 Collector)
    private CANSparkMax _lfDrive;
    private CANSparkMax _lmDrive;
    private CANSparkMax _lrDrive;
    private CANSparkMax _rfDrive;
    private CANSparkMax _rmDrive;
    private CANSparkMax _rrDrive;
    private TalonSRX _leftArm;
    private TalonSRX _rightArm;
    private TalonSRX _elevator;
    private TalonSRX _topCollector;
    private TalonSRX _bottomCollector;
    private Solenoid _collectorSolenoid;
    private DoubleSolenoid _hab3Solenoid;
    private AHRS _gyro;

    public RobotMap(){
        try {
            _lfDrive = new CANSparkMax(Constants.kLFDriveId, MotorType.kBrushless);
            
            _lrDrive = new CANSparkMax(Constants.kLRDriveId, MotorType.kBrushless);
            _lrDrive.follow(_lfDrive);

            _rfDrive = new CANSparkMax(Constants.kRFDriveId, MotorType.kBrushless);
            _rrDrive = new CANSparkMax(Constants.kRRDriveId, MotorType.kBrushless);
            
            _elevator = new TalonSRX(Constants.kElevatorId);
            _gyro = new AHRS(Port.kMXP);
            _rrDrive.follow(_rfDrive);
        } catch (Exception ex){
            ex.printStackTrace();
        }
        if (!Constants.kIsTestRobot){
            _lmDrive = new CANSparkMax(Constants.kLMDriveId, MotorType.kBrushless);
            _lmDrive.follow(_lfDrive);
            
            _rmDrive = new CANSparkMax(Constants.kRMDriveId, MotorType.kBrushless);
            _rmDrive.follow(_rfDrive);

            _leftArm = new TalonSRX(Constants.kLArmId);
            _rightArm = new TalonSRX(Constants.kRArmId);
            _topCollector = new TalonSRX(Constants.kTCollectorId);
            _bottomCollector = new TalonSRX(Constants.kBCollectorId);
            _collectorSolenoid = new Solenoid(Constants.kCollectorSolenoidSlot);
            _hab3Solenoid = new DoubleSolenoid(Constants.kHab3SolenoidSlotA, Constants.kHab3SolenoidSlotB);
        }
        
        System.out.println("Created Robot Map");
        
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
    public TalonSRX getElevatorTalon(){
        return _elevator;
    }
    public TalonSRX getTopCollectorTalon(){
        return _topCollector;
    }
    public TalonSRX getBottomCollectorTalon(){
        return _bottomCollector;
    }
    public Solenoid getCollectorSolenoid(){
        return _collectorSolenoid;
    }
    public DoubleSolenoid getHab3Solenoid(){
        return _hab3Solenoid;
    }
    public AHRS getGyro(){
        return _gyro;
    }
}