package frc.robot.Subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Hab3 implements Subsystem {

    private DoubleSolenoid _solenoid;

    public Hab3(DoubleSolenoid solenoid){
        _solenoid = solenoid;
    }
    public void GO(){
        _solenoid.set(Value.kForward);
    }
    public void Retract() {
        _solenoid.set(Value.kOff);
    }

    public void WriteToDashboard() {
        SmartDashboard.putBoolean("hab3Triggered", _solenoid.get() == Value.kForward);
        //Write some data
    }

	@Override
	public void ResetSensors() {
		
	}
}