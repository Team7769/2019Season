package frc.robot.Subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Arm implements Subsystem{
    private TalonSRX _leftTalon;
    private TalonSRX _rightTalon;

    public Arm(TalonSRX leftMotor, TalonSRX rightMotor){
        _leftTalon = leftMotor;

        _rightTalon = rightMotor;
        _rightTalon.setInverted(true);
    }

    public void setSpeed(double speed) {
        _leftTalon.set(ControlMode.PercentOutput, speed);
        _rightTalon.set(ControlMode.PercentOutput, speed);
    }

    public void WriteToDashboard() {
        SmartDashboard.putNumber("leftArmSpeed", _leftTalon.getSelectedSensorVelocity());
        SmartDashboard.putNumber("leftArmPosition", _leftTalon.getSelectedSensorPosition());
        SmartDashboard.putNumber("rightArmSpeed", _rightTalon.getSelectedSensorVelocity());
        SmartDashboard.putNumber("rightArmPosition", _rightTalon.getSelectedSensorPosition());
    }

	public void ResetSensors() {
        _leftTalon.setSelectedSensorPosition(0);
        _rightTalon.setSelectedSensorPosition(0);
	}

}