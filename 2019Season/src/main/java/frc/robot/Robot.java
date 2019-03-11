/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Configuration.Constants;
import frc.robot.Subsystems.Arm;
import frc.robot.Subsystems.Collector;
import frc.robot.Subsystems.DriveTrain;
import frc.robot.Subsystems.Elevator;
import frc.robot.Subsystems.Hab3;
import frc.robot.Subsystems.Subsystem;
import frc.robot.Utilities.RobotControllerMap;
import frc.robot.Utilities.RobotMap;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  /**
   * Version: 1.0
   */
  private RobotControllerMap _robotControllers;
  private RobotMap _robotMap;
  private DriveTrain _driveTrain;
  private Elevator _elevator;
  private Arm _arm;
  private Hab3 _hab3;
  private Collector _collector;
  private XboxController _driverController;
  private XboxController _operatorController;
  private Compressor _compressor;

  private ArrayList<Subsystem> _subsystems;
  private int _timer;
  private int _autonomousCase;
  private int _delayTimer;
  private String _autonomousMode;

  @Override
  public void robotInit() {
    _robotControllers = new RobotControllerMap(Constants.kDriverControllerSlot, Constants.kOperatorControllerSlot);
    _subsystems = new ArrayList<Subsystem>();
    try {
      _robotMap = new RobotMap();
      _driveTrain = new DriveTrain(_robotMap.getLeftDriveSpark(), _robotMap.getRightDriveSpark(), _robotMap.getGyro());
      _driveTrain.resetEncoders();
      _subsystems.add(_driveTrain);
    } catch (Exception ex){
      ex.printStackTrace();
    }
    _driverController = _robotControllers.getDriverController();
    _operatorController = _robotControllers.getOperatorController();
    _timer = 0;
    _autonomousCase = 0;
    _delayTimer = 0;
    _autonomousMode = "0";

    if (!Constants.kIsTestRobot){
      _hab3 = new Hab3(_robotMap.getHab3Solenoid());
      _arm = new Arm(_robotMap.getLeftArmTalon(), _robotMap.getRightArmTalon());
      _elevator = new Elevator(_robotMap.getLeftElevatorTalon(), _robotMap.getRightElevatorTalon());
      _collector = new Collector(_robotMap.getTopCollectorTalon(), _robotMap.getBottomCollectorTalon(), 
                                 _robotMap.getCollectorSolenoid());
      _compressor = new Compressor();

      _compressor.start();
      _subsystems.add(_collector);
      _subsystems.add(_arm);
      _subsystems.add(_elevator);
      _subsystems.add(_collector);
      _subsystems.add(_hab3);
    }
    _driveTrain.setPath("LeftFirstShipHatchClose", false);
    //_driveTrain.setPath("CrossLinePath", true);
    
  }

  @Override
  public void autonomousInit() {
    _autonomousCase = 0;
    _subsystems.forEach((s) -> s.ResetSensors());
    _driveTrain.resetEncoders();
    
    System.out.println(Timer.getFPGATimestamp() +  ": Starting Autonomous Mode: " + _autonomousMode);
  }

  @Override
  public void autonomousPeriodic() {
    int currentCase = _autonomousCase;
    switch (_autonomousMode) {
      case "0":
        break;
      case "1":
        //basicSmartMotionAuto();
        break;
      case "2":
        singleHatchCloseShip();
        break;
      case "9":
        testRotationAuto();
        break;
      case "10":
        basicDriveStraightAuto();
        break;
      case "11":
        basicPathAuto();
        break;
      default:
        break;
    }
    if (currentCase != _autonomousCase){
      System.out.println(Timer.getFPGATimestamp() +  ": Moving to Case: " + _autonomousCase);
    }
      
    _timer++;
    _delayTimer++;
  }
  public void basicPathAuto(){
    switch (_autonomousCase){
      case 0:
        _driveTrain.startPath();
        _autonomousCase++;
        break;
      case 1:
        if (_driveTrain.isFinishedFollowingPath()){
          _autonomousCase++;
          _driveTrain.stop();
          break;
        }
      case 2:
        _driveTrain.stop();
        break;
    }
  }
  public void testRotationAuto(){
    switch (_autonomousCase){
      case 0:
        _driveTrain.turnToAngle(90);
        _autonomousCase++;
        break;
      case 1:
        if (_driveTrain.isAngleOnTarget()){
          _autonomousCase++;
        }
        break;
      case 2:
        _driveTrain.stop();
        break;
    }
  }
  public void basicDriveStraightAuto(){
    switch (_autonomousCase){
      case 0:
        _driveTrain.driveDistance(100, 0);
        _autonomousCase++;
        break;
      case 1:
        if (_driveTrain.isDistanceOnTarget()){
          _driveTrain.stop();
          _autonomousCase++;
        }
        break;
      default:
        _driveTrain.stop();
        break;
    }
  }
  public void singleHatchCloseShip(){
    switch (_autonomousCase){
      case 0:
        //_driveTrain.setSmartMotionParameters(60, 36, 87);
        _driveTrain.driveDistance(87, 0);
        _autonomousCase++;
        break;
      case 1:
        //_driveTrain.driveDistanceSmartMotion();
        if (_driveTrain.isDistanceOnTarget()){
          _driveTrain.resetEncoders();
          _driveTrain.stop();
          _autonomousCase++;
        }
        break;
      case 2:
        _driveTrain.turnToAngle(90);
        _autonomousCase++;
        break;
      case 3:
        if (_driveTrain.isAngleOnTarget()){
          _driveTrain.resetEncoders();
          _driveTrain.stop();
          _autonomousCase++;
        }
        break;
      case 4:
        //_driveTrain.setSmartMotionParameters(60, 36, 45);
        _driveTrain.driveDistance(45, 90);
        _autonomousCase++;
        break;
      case 5:
        //_driveTrain.driveDistanceSmartMotion();
        if (_driveTrain.isDistanceOnTarget()){
          _driveTrain.resetEncoders();
          _driveTrain.stop();
          _autonomousCase++;
        }
        break;
      case 6:
        _driveTrain.turnToAngle(0);
        _autonomousCase++;
        break;
      case 7:
        if (_driveTrain.isAngleOnTarget()){
          _driveTrain.stop();
          _autonomousCase++;
        }
        break;
      case 8:
      _driveTrain.resetEncoders();
        //_driveTrain.setSmartMotionParameters(60, 36, 127);
        _driveTrain.driveDistance(127, 0);
        _autonomousCase++;
        break;
      case 9:
        //_driveTrain.driveDistanceSmartMotion();
        if (_driveTrain.isDistanceOnTarget()){
          _driveTrain.stop();
          _autonomousCase++;
        }
        break;
      default:
        _driveTrain.resetEncoders();
        _driveTrain.stop();
        break;
    }
  }

  @Override
  public void teleopInit() {
  }

  @Override
  public void teleopPeriodic() {
    teleopDrive();

    if (!Constants.kIsTestRobot){
      teleopElevator();
      teleopCollector();
      if (_driverController.getStartButton() && _driverController.getBackButton()){
        executeHab3();
      }
    }
  }
  public void executeHab3(){
    //_hab3.GO();
  }
  public void teleopDrive(){
    _driveTrain.curvatureDrive(_driverController.getY(Hand.kLeft), _driverController.getX(Hand.kRight), getQuickTurn());
  }
  public boolean getQuickTurn() {
    return _driverController.getBumper(Hand.kRight);
  }
  public void teleopElevator(){
    if (Math.abs(_operatorController.getY(Hand.kLeft)) > .05
        || Math.abs(_operatorController.getX(Hand.kRight)) > .05)
    {
      manualArm(_operatorController.getX(Hand.kRight));
      manualElevator(_operatorController.getY(Hand.kLeft));
    } else if (_operatorController.getAButton()){
      _elevator.setPositionLowHatch();
      _arm.setPositionLowHatch();
    } else if (_operatorController.getBButton()){
      _elevator.setPositionMidHatch();
      _arm.setPositionMidHatch();
    } else if (_operatorController.getXButton()){
      _elevator.setPositionLowCargo();
      _arm.setPositionLowCargo();
    } else if (_operatorController.getYButton()){
      _elevator.setPositionMidCargo();
      _arm.setPositionMidCargo();
    } else if (_operatorController.getBumper(Hand.kRight)){
      _elevator.setPositionTopCargo();
      _arm.setPositionTopCargo();
    }
  }
  public void manualElevator(double value){
    _elevator.setSpeed(value);
  }
  public void manualArm(double value){
    _arm.setSpeed(value);
  }
  public void teleopCollector(){
    if (Math.abs(_operatorController.getTriggerAxis(Hand.kLeft)) > .05){
      _collector.setSpeed(_operatorController.getTriggerAxis(Hand.kLeft));
    } else if (Math.abs(_operatorController.getTriggerAxis(Hand.kRight)) > .05)
    {
      _collector.setSpeed(_operatorController.getTriggerAxis(Hand.kRight));
    } else {
      _collector.setSpeed(0.0);
    }

    if (_driverController.getAButton()){
      _collector.grabHatch();
    } else if (_driverController.getBButton()){
      _collector.releaseHatch();
    }
  }

  @Override
  public void testInit() {
  }

  @Override
  public void testPeriodic() {
  }

  @Override
  public void robotPeriodic(){
    _autonomousMode = SmartDashboard.getString("autoMode", "0");
    SmartDashboard.putString("autoMode", _autonomousMode);
    _subsystems.forEach((s) -> s.WriteToDashboard());
  }

  @Override 
  public void disabledPeriodic() {
    if (!Constants.kIsTestRobot){
      _elevator.ResetSensors();
      _arm.ResetSensors();
      _driveTrain.resetEncoders();
    }
  }

}
