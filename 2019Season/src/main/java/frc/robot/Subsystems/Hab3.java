package frc.robot.Subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Hab3 implements Subsystem {

    private DoubleSolenoid _leftSolenoid;
    private DoubleSolenoid _rightSolenoid;

    public Hab3(DoubleSolenoid leftSolenoid, DoubleSolenoid rightSolenoid){
        _leftSolenoid = leftSolenoid;
        _rightSolenoid = rightSolenoid;
    }
    public void GO(){
        _leftSolenoid.set(Value.kReverse);
        _rightSolenoid.set(Value.kReverse);
    }
    public void Retract() {
        _leftSolenoid.set(Value.kForward);
        _rightSolenoid.set(Value.kForward);
    }
    public void Stop() {
        _leftSolenoid.set(Value.kOff);
        _rightSolenoid.set(Value.kOff);
    }

    public void WriteToDashboard() {
        SmartDashboard.putBoolean("hab3Triggered", _leftSolenoid.get() == Value.kReverse);
        SmartDashboard.putString("hab3Value", _leftSolenoid.get().toString());
        //Write some data
    }

	@Override
	public void ResetSensors() {
		
	}
}