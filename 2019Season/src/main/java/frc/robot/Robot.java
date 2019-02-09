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
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
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

  @Override
  public void robotInit() {
    _robotControllers = new RobotControllerMap(Constants.kDriverControllerSlot, Constants.kOperatorControllerSlot);
    _subsystems = new ArrayList<Subsystem>();
    try {
      _robotMap = new RobotMap();
      _driveTrain = new DriveTrain(_robotMap.getLeftDriveSpark(), _robotMap.getRightDriveSpark());
      _subsystems.add(_driveTrain);
    } catch (Exception ex){
      ex.printStackTrace();
    }
    _driverController = _robotControllers.getDriverController();
    _operatorController = _robotControllers.getOperatorController();
    _timer = 0;
    _autonomousCase = 0;
    _delayTimer = 0;

    if (!Constants.kIsTestRobot){
      _hab3 = new Hab3();
      _arm = new Arm(_robotMap.getLeftArmTalon(), _robotMap.getRightArmTalon());
      _elevator = new Elevator(_robotMap.getLeftElevatorTalon(), _robotMap.getRightElevatorTalon());
      _collector = new Collector(_robotMap.getTopCollectorTalon(), _robotMap.getBottomCollectorTalon(), 
                                 _robotMap.getLeftCollectorSolenoid(), _robotMap.getRightCollectorSolenoid());
      _compressor = new Compressor();

      _compressor.start();
      _subsystems.add(_collector);
      _subsystems.add(_arm);
      _subsystems.add(_elevator);
      _subsystems.add(_collector);
    }
    
  }

  @Override
  public void autonomousInit() {
    _autonomousCase = 0;
    _subsystems.forEach((s) -> s.ResetSensors());
  }

  @Override
  public void autonomousPeriodic() {
    switch (3) {
      case 0:
        basicDriveAuto();
        break;
      case 1:
        basicProfileAuto();
        break;
      case 2:
        basicSmartMotionAuto();
        break;
      case 3:
        intermediateSmartMotionAuto();
        break;
      case 4:
        testAuto();
        break;
      default:
        break;
    }
    
    System.out.println("Case: " + _autonomousCase);
        
    _timer++;
    _delayTimer++;
  }
  public void testAuto() {
    _driveTrain.arcadeDrive(1.0, 0);
  }
  public void basicDriveAuto(){
    switch (_autonomousCase){
      case 0:
        _driveTrain.setTargetDistance(100);
        _autonomousCase++;
        break;
      case 1:
        _driveTrain.driveDistance();
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
  public void basicProfileAuto(){
    switch (_autonomousCase){
      case 0:
        _driveTrain.setTargetProfile(100);
        _autonomousCase++;
        break;
      case 1:
        _driveTrain.followProfile();
        if (_driveTrain.isProfileOnTarget()){
          _driveTrain.stop();
          _autonomousCase++;
        }
        break;
      default:
        _driveTrain.stop();
        break;
    }
  }
  public void basicSmartMotionAuto(){
    switch (_autonomousCase){
      case 0:
        _driveTrain.setSmartMotionParameters(84, 60, 100);
        _autonomousCase++;
        break;
      case 1:
        _driveTrain.driveDistanceSmartMotion();
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
  public void intermediateSmartMotionAuto(){
    switch (_autonomousCase){
      case 0:
        _driveTrain.ResetSensors();
        _driveTrain.setSmartMotionParameters(84, 84, 100);
        _delayTimer = 0;
        _autonomousCase++;
        break;
      case 1:
        _driveTrain.driveDistanceSmartMotion();
        if (_driveTrain.isDistanceOnTarget() && _delayTimer > 5){
          _driveTrain.stop();
          _autonomousCase++;
        }
        break;
      case 2:
        _delayTimer = 0;
        _driveTrain.ResetSensors();
        _driveTrain.setSmartMotionParameters(84, 84, -100);
        _autonomousCase++;
        break;
      case 3:
        _driveTrain.driveDistanceSmartMotion();
        if (_driveTrain.isDistanceOnTarget() && _delayTimer > 5){
          _driveTrain.stop();
          _autonomousCase++;
        }
        break;
      default:
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
    }
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
    _subsystems.forEach((s) -> s.WriteToDashboard());
  }

}
