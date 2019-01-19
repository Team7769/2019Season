package frc.robot.Subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Collector implements Subsystem {

    private TalonSRX _topCollector;
    private TalonSRX _bottomCollector;

    public Collector(TalonSRX topCollector, TalonSRX bottomCollector) {
        _topCollector = topCollector;
        _bottomCollector = bottomCollector;
    }
    public void setSpeed(double speed){
        _topCollector.set(ControlMode.PercentOutput, speed);
        _bottomCollector.set(ControlMode.PercentOutput, speed);
    }

    @Override
    public void WriteToDashboard() {
        SmartDashboard.putNumber("topCollectorSpeed", _topCollector.getSelectedSensorVelocity());
        SmartDashboard.putNumber("topCollectorPosition", _topCollector.getSelectedSensorPosition());
        SmartDashboard.putNumber("bottomCollectorSpeed", _bottomCollector.getSelectedSensorVelocity());
        SmartDashboard.putNumber("bottomCollectorPosition", _bottomCollector.getSelectedSensorPosition());
    }

}