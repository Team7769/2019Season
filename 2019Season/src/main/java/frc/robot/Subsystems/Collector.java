package frc.robot.Subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Configuration.Constants;

public class Collector implements Subsystem {

    private TalonSRX _topCollector;
    private TalonSRX _bottomCollector;
    private Solenoid _solenoid;

    public Collector(TalonSRX topCollector, TalonSRX bottomCollector, Solenoid solenoid) {
        _topCollector = topCollector;
        _bottomCollector = bottomCollector;
        _solenoid = solenoid;
    }
    public Collector(TalonSRX topCollector, TalonSRX bottomCollector) {
        _topCollector = topCollector;
        _bottomCollector = bottomCollector;
    }
    public void setSpeed(double speed){
        //double dampen = 0.35;
        //speed = speed * dampen;
        _topCollector.set(ControlMode.PercentOutput, speed);
        _bottomCollector.set(ControlMode.PercentOutput, speed);
    }
    public void intake(){
        setSpeed(Constants.kCollectorIntakeSpeed);
    }
    public void releaseCargo(){
        setSpeed(Constants.kCollectorReleaseSpeed);
    }
    public void grabHatch(){
        _solenoid.set(true);
    }
    public void releaseHatch() {
        _solenoid.set(false);
    }

    @Override
    public void WriteToDashboard() {
        SmartDashboard.putNumber("topCollectorSpeed", _topCollector.getSelectedSensorVelocity());
        SmartDashboard.putNumber("topCollectorPosition", _topCollector.getSelectedSensorPosition());
        SmartDashboard.putNumber("bottomCollectorSpeed", _bottomCollector.getSelectedSensorVelocity());
        SmartDashboard.putNumber("bottomCollectorPosition", _bottomCollector.getSelectedSensorPosition());
        SmartDashboard.putBoolean("hatchExtended", _solenoid.get());
    }
	
	public void ResetSensors() {
		//Do nothing
	}

}