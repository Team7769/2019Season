package frc.robot.Utilities;

import edu.wpi.first.wpilibj.Joystick;

public class RobotControllerMap {
    private Joystick _driverController;
    private Joystick _operatorController;

    public RobotControllerMap(int driverUsbSlot, int operatorUsbSlot) {
        _driverController = new Joystick(driverUsbSlot);
        _operatorController = new Joystick(operatorUsbSlot);
    }
    public Joystick getDriverJoystick(){
        return _driverController;
    }
    public Joystick getOperatorJoystick(){
        return _operatorController;
    }
}