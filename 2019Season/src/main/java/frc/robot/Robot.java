/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import frc.robot.Configuration.Constants;
import frc.robot.Subsystems.DriveTrain;
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
  private XboxController _driverController;
  private XboxController _operatorController;

  @Override
  public void robotInit() {
    _robotControllers = new RobotControllerMap(Constants.kDriverControllerSlot, Constants.kOperatorControllerSlot);
    _robotMap = new RobotMap();

    _driveTrain = new DriveTrain(_robotMap.getLeftDriveSpark(), _robotMap.getRightDriveSpark());
    _driverController = _robotControllers.getDriverController();
    _operatorController = _robotControllers.getOperatorController();
  }

  @Override
  public void autonomousInit() {
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
  }
  public void teleopDrive(){
    boolean isQuickTurn = _driverController.getY(Hand.kLeft) > 0.05 && _driverController.getX(Hand.kRight) > 0.5;
    
    _driveTrain.curvatureDrive(_driverController.getY(Hand.kLeft), _driverController.getX(Hand.kRight), isQuickTurn);
  }

  @Override
  public void testInit() {
  }

  @Override
  public void testPeriodic() {
  }

}
