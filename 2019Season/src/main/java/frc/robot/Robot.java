/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Utilities.RobotControllerMap;

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
  private XboxController _driverController;
  private XboxController _operatorController;

  private static final int kDriverUsbSlot = 0;
  private static final int kOperatorUsbSlot = 1;

  @Override
  public void robotInit() {
    _robotControllers = new RobotControllerMap(kDriverUsbSlot, kOperatorUsbSlot);
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
    
  }

  @Override
  public void testInit() {
  }

  @Override
  public void testPeriodic() {
  }

}
