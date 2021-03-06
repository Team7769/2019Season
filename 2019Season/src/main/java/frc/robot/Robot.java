/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.util.ArrayList;

import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.Spark;
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
   * Version: 1.2
   * Team 7769
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
  private Spark _blinkin;
  private UsbCamera _camera;

  private ArrayList<Subsystem> _subsystems;
  private int _timer;
  private int _autonomousCase;
  private int _delayTimer;
  private int _diagnosticsTimer;
  private String _autonomousMode;
  private String _autonomousStartPosition;
  private String _autonomousStartZone;
  private boolean _autonomousPartTwo;
  private boolean _sandstormOverride;
  private boolean _manualElevator;
  private double _ledTest;
  private boolean _hatchExtended;
  private double _slowSpeed;

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
    _autonomousStartZone = Constants.kAutonomousZoneHab1;
    _autonomousStartPosition = Constants.kAutonomousPositionLeft;
    _autonomousPartTwo = false;
    _sandstormOverride = true; //Not running autonomous
    _manualElevator = false;
    _diagnosticsTimer = 0;
    _ledTest = Constants.kBlinkinSolidRed;
    _hatchExtended = false;
    _slowSpeed = .55;
    
    _elevator = new Elevator(_robotMap.getElevatorTalon());
    _subsystems.add(_elevator);

    if (!Constants.kIsTestRobot){
      _hab3 = new Hab3(_robotMap.getHab3SolenoidLeft(), _robotMap.getHab3SolenoidRight());
      _arm = new Arm(_robotMap.getLeftArmTalon(), _robotMap.getRightArmTalon());
      _collector = new Collector(_robotMap.getTopCollectorTalon(), _robotMap.getBottomCollectorTalon(), 
                                 _robotMap.getCollectorSolenoid());
      _compressor = new Compressor();
      _blinkin = _robotMap.getBlinkin();

      _camera = CameraServer.getInstance().startAutomaticCapture();

      _compressor.start();
      _subsystems.add(_collector);
      _subsystems.add(_arm);
      _subsystems.add(_collector);
      _subsystems.add(_hab3);
    }
    _driveTrain.setPath("LeftFirstShipHatchClose", false);
    //_driveTrain.setPath("CrossLinePath", false);
    
  }

  @Override
  public void autonomousInit() {
    _blinkin.set(Constants.kBlinkinHeartbeatBlue);
    _autonomousCase = 0;
    _subsystems.forEach((s) -> s.ResetSensors());
    _driveTrain.resetEncoders();
    _sandstormOverride = false;
    
    System.out.println(Timer.getFPGATimestamp() +  ": Starting Autonomous Mode: " + _autonomousMode);
  }

  @Override
  public void autonomousPeriodic() {
    int currentCase = _autonomousCase;
    if (!_sandstormOverride){
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
    } else {
      teleopDrive();
      teleopElevator();
      if (!Constants.kIsTestRobot){
        teleopCollector();
      }
    }
    if (_driverController.getXButton()){
      _autonomousCase = 7769;
      _driveTrain.stop();
      _sandstormOverride = true;
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
        }
        break;
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
        if (_autonomousStartPosition == Constants.kAutonomousPositionRight){
          _driveTrain.turnToAngle(-90);
        } else {
          _driveTrain.turnToAngle(90);
        }
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
        //System.out.println("Hab 3");
        _blinkin.set(Constants.kBlinkinFireLarge);
        executeHab3();
      }
      if (_driverController.getYButton() && _driverController.getBButton()){
        //System.out.println("Hab 3 Retract");
        retractHab3();
      }
    }
  }
  /**
   * Starts Hab 3 mechanism. This is the point of no return.
   */
  public void executeHab3(){
    _hab3.GO();
  }
  public void retractHab3(){
    _hab3.Retract();
  }
  /**
   * Teleoperated drive control method. Place in periodic routines with driver control allowed. Drive style is Curvature.
   */
  public void teleopDrive(){
    double throttleDemand = _driverController.getY(Hand.kLeft);
    double turnDemand = _driverController.getX(Hand.kRight);
    boolean slowMode = _driverController.getBumper(Hand.kRight);

    if (Math.abs(throttleDemand) >= 0.05 || Math.abs(turnDemand) >= 0.05){
      //_driveTrain.curvatureDrive(_driverController.getY(Hand.kLeft), _driverController.getX(Hand.kRight), getQuickTurn());
      if (slowMode){
        _driveTrain.arcadeDrive(throttleDemand * _slowSpeed, turnDemand * _slowSpeed);
      } else {        
        _driveTrain.arcadeDrive(throttleDemand, turnDemand);
      }
    } else {
      _driveTrain.stop();
    }
  }
  public Boolean getQuickTurn() {
    return Math.abs(_driverController.getY(Hand.kLeft)) < 0.05; 
  }
  /**
   * Method for handling operator control for the elevator/arm mechanism. Place in periodic routines.
   */
  public void teleopElevator(){
    if (Math.abs(_operatorController.getY(Hand.kLeft)) > .05)
    {
      _manualElevator = true;
    } 
    
    armControl();
  }
  
  /**
   * Submethod for controlling arm and elevator behavior. The two mechanisms are intended to work in tandem, via programmed setpoints. Manual control is activated
   * by moving either operator joystick, halting autonomous arm/elevator movement.
   */
  public void armControl(){
    double joystick = _operatorController.getX(Hand.kRight);
    /**
     * Operator:
     * Left Bumper: Ground Cargo Pickup
     * Right Bumper: Cargo Ship Deposit
     * A Button: Neutral Position (Packaged)
     * B Button: Low Hatch Level
     * X Button: Mid Rocket Cargo Angle
     * Y Button: Low Rocket Cargo Angle
     * 
     * Driver:
     * 
     * B Button: Set Reversed Positions
     * A Button: Set Forward Positions
     */
    if (_operatorController.getAButton()){
      _manualElevator = false;
      _elevator.setPositionNeutral();
      _arm.setPositionNeutral();
    } else if (_operatorController.getBButton()){
      _manualElevator = false;
      _elevator.setPositionLowHatch();
      _arm.setPositionLowHatch();
    } else if (_operatorController.getYButton()){
      _manualElevator = false;
      _arm.setPositionLowCargo();
      _elevator.setPositionLowCargo();
    } else if (_operatorController.getXButton()){
      _manualElevator = false;
      _arm.setPositionMidHatch();
      _elevator.setPositionMidHatch();
    } else if (_operatorController.getStartButton()){
      _manualElevator = false;
      _arm.setPositionTopCargo();
      _elevator.setPositionTopCargo();
    }else if (Math.abs(_operatorController.getTriggerAxis(Hand.kLeft)) > 0.05){
      _manualElevator = false;
      _arm.setPositionMidCargo();
      _elevator.setPositionMidCargo();
    } else if (_operatorController.getBumper(Hand.kRight)){
      _manualElevator = false;
      _arm.setPositionCargoShipCargo();
      _elevator.setPositionCargoShipCargo();
    } else if (_operatorController.getBumper(Hand.kLeft)){
      _manualElevator = false;
      _elevator.setPositionNeutral();
      _arm.setPositionGround();
    } else if (Math.abs(joystick) > 0.1){
      _manualElevator = true;
    }
    //if (_driverController.getBButton()){
    //  _arm.setReverse(true);
    //} else if (_driverController.getAButton()){
    //  _arm.setReverse(false);
    //}
    if (_manualElevator){
      manualArm(joystick);
      manualElevator(_operatorController.getY(Hand.kLeft));
    }
    SmartDashboard.putBoolean("armStateManual", _manualElevator);
  }
  public void manualElevator(double value){
    _elevator.setSpeed(value);
  }
  public void manualArm(double value){
    _arm.setSpeed(value);
  }
  public void teleopCollector(){
    if (Math.abs(_driverController.getTriggerAxis(Hand.kRight)) > .05){
      _collector.releaseCargo();
    } else if (Math.abs(_operatorController.getTriggerAxis(Hand.kRight)) > .05)
    {
      _collector.intake();
    } else {
      _collector.setSpeed(0.1);
    }
    if (_driverController.getBumper(Hand.kLeft)){
      _blinkin.set(Constants.kBlinkinSolidRed);
      _collector.grabHatch();
    } else if (Math.abs(_driverController.getTriggerAxis(Hand.kLeft)) > .05)
    {
      _blinkin.set(Constants.kBlinkinSolidGreen);
      _collector.releaseHatch();
    }
  }

  @Override
  public void testInit() {
  }

  @Override
  public void testPeriodic() {
    teleopDrive();
    if (!Constants.kIsTestRobot){
      teleopElevator();
      teleopCollector();
      if (_driverController.getStartButton() && _driverController.getBackButton() && _operatorController.getStartButton() && _operatorController.getBackButton()){
        //System.out.println("Hab 3");
        _blinkin.set(Constants.kBlinkinColorWavesOcean);
        executeHab3();
      }
      if (_driverController.getYButton() && _driverController.getBButton()){
        //System.out.println("Hab 3 Retract");
        retractHab3();
      }

      if (_driverController.getXButton()){
        _elevator.ResetSensors();
        _arm.ResetSensors();
        _driveTrain.resetEncoders();
      }
    }
  }

  @Override
  public void robotPeriodic(){
    _ledTest = SmartDashboard.getNumber("ledTest", _ledTest);
    SmartDashboard.putNumber("ledTest", _ledTest);
    _autonomousMode = SmartDashboard.getString("autoMode", "0");
    SmartDashboard.putString("autoMode", _autonomousMode);
    _subsystems.forEach((s) -> s.WriteToDashboard());

    if (_diagnosticsTimer >= 50){
      _robotMap.PrintDiagnostics();
      _diagnosticsTimer = 0;
    }
    _diagnosticsTimer++;
  }

  @Override 
  public void disabledPeriodic() {
    if (!Constants.kIsTestRobot){
      //_elevator.ResetSensors();
      //_arm.ResetSensors();
      _driveTrain.resetEncoders();
      _hab3.Stop();
    }
    _autonomousStartZone = SmartDashboard.getString("startZone", Constants.kAutonomousZoneHab1);
    _autonomousStartPosition = SmartDashboard.getString("startPosition", Constants.kAutonomousPositionLeft);
    _autonomousPartTwo = SmartDashboard.getBoolean("partTwo", false);

  }

}
