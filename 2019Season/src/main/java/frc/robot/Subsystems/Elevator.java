package frc.robot.Subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Elevator implements Subsystem {

    private TalonSRX _leftTalon;
    private TalonSRX _rightTalon;

    public Elevator(TalonSRX leftTalon, TalonSRX rightTalon){
        _leftTalon = leftTalon;

        _rightTalon = rightTalon;
        _rightTalon.setInverted(true);
    }
    public void setSpeed(double speed) {
        _leftTalon.set(ControlMode.PercentOutput, speed);
        _rightTalon.set(ControlMode.PercentOutput, speed);
    }

    @Override
    public void WriteToDashboard() {
        SmartDashboard.putNumber("leftElevatorSpeed", _leftTalon.getSelectedSensorVelocity());
        SmartDashboard.putNumber("leftElevatorPosition", _leftTalon.getSelectedSensorPosition());
        SmartDashboard.putNumber("rightElevatorSpeed", _rightTalon.getSelectedSensorVelocity());
        SmartDashboard.putNumber("rightElevatorPosition", _rightTalon.getSelectedSensorPosition());
    }

}