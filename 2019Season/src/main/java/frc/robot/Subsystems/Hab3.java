package frc.robot.Subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Hab3 implements Subsystem {

    private Solenoid _solenoid;

    public Hab3(Solenoid solenoid){
        _solenoid = solenoid;
    }
    public void GO(){
        _solenoid.set(true);
    }
    public void Retract() {
        _solenoid.set(false);
    }

    public void WriteToDashboard() {
        SmartDashboard.putBoolean("hab3Triggered", _solenoid.get());
        //Write some data
    }

	@Override
	public void ResetSensors() {
		
	}
}