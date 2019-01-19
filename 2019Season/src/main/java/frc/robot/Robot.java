/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.io.File;
import java.util.ArrayList;
import java.util.Map;

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

  private ArrayList<Subsystem> _subsystems;

  @Override
  public void robotInit() {
    Map<String, String> env = System.getenv();
    String path = env.get("HOME") + "/RobotConfig.csv";
    Constants.loadConstantsFromFilePath();

    _robotControllers = new RobotControllerMap(Constants.kDriverControllerSlot, Constants.kOperatorControllerSlot);
    _robotMap = new RobotMap();

    _driveTrain = new DriveTrain(_robotMap.getLeftDriveSpark(), _robotMap.getRightDriveSpark());
    _driverController = _robotControllers.getDriverController();
    _operatorController = _robotControllers.getOperatorController();
    _subsystems.add(_driveTrain);

    if (!Constants.kIsTestRobot){
      _hab3 = new Hab3();
      _arm = new Arm(_robotMap.getLeftArmTalon(), _robotMap.getRightArmTalon());
      _elevator = new Elevator(_robotMap.getLeftElevatorTalon(), _robotMap.getRightElevatorTalon());
      _collector = new Collector(_robotMap.getTopCollectorTalon(), _robotMap.getBottomCollectorTalon());
      _subsystems.add(_collector);
      _subsystems.add(_arm);
      _subsystems.add(_elevator);
      _subsystems.add(_collector);
    }
    
  }

  @Override
  public void autonomousInit() {
    _subsystems.forEach((s) -> s.ResetSensors());
  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
  }

  @Override
  public void teleopPeriodic() {
    teleopDrive();

    if (!Constants.kIsTestRobot){
      teleopElevator();
      teleopArm();
      teleopCollector();
    }
  }
  public void teleopDrive(){
    _driveTrain.curvatureDrive(_driverController.getY(Hand.kLeft), _driverController.getX(Hand.kRight), getQuickTurn());
  }
  public boolean getQuickTurn() {
    return _driverController.getAButton();
  }
  public void teleopElevator(){
    if (Math.abs(_operatorController.getY(Hand.kLeft)) > .05)
    {
      _elevator.setSpeed(_operatorController.getY(Hand.kLeft));
    }
  }
  public void teleopArm(){
    if (Math.abs(_operatorController.getX(Hand.kRight)) > .05)
    {
      _arm.setSpeed(_operatorController.getX(Hand.kRight));
    }
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
