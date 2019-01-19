package frc.robot.Subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

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

    }

}