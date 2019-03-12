package frc.robot.Subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Configuration.Constants;

public class Arm implements Subsystem{
    private TalonSRX _leftTalon;
    private TalonSRX _rightTalon;
    private double _setpoint;
    private boolean _isReverse;
    private String _setpointName;


    public Arm(TalonSRX leftMotor, TalonSRX rightMotor){
        _leftTalon = leftMotor;
        _leftTalon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);

        _rightTalon = rightMotor;
        _rightTalon.setInverted(true);
        _rightTalon.follow(_leftTalon);

        _setpoint = Constants.kArmNeutral;
        _isReverse = false;
        _setpointName = "Neutral";

        setPositionPIDValues();
    }
    public void setPositionPIDValues(){
        _leftTalon.config_kP(0, Constants.kArmP);
        _leftTalon.config_kI(0, Constants.kArmI);
        _leftTalon.config_kD(0, Constants.kArmD);
        _leftTalon.config_kF(0, Constants.kArmFF);        
    }
    public void setSpeed(double speed) {
        _leftTalon.set(ControlMode.PercentOutput, speed);
        _rightTalon.set(ControlMode.PercentOutput, speed);
    }
    public void setPosition(double position) {
        //_leftTalon.set(ControlMode.Position, position);
        //_rightTalon.set(ControlMode.Position, position);
        _setpoint = position;
    }
    public void setReverse(boolean reverse){
        _isReverse = reverse;
    }
    public void setPositionNeutral(){
        setPosition(Constants.kArmNeutral);
        _setpointName = "Neutral";
    }
    public void setPositionLowHatch(){
        setPosition(Constants.kArmLowHatch);
        _setpointName = "Low Hatch";
    }
    public void setPositionLowCargo(){
        setPosition(Constants.kArmLowCargo);
        _setpointName = "Low Cargo";
    }
    public void setPositionMidHatch(){
        setPosition(Constants.kArmMidHatch);
        _setpointName = "Mid Hatch";
    }
    public void setPositionMidCargo(){
        setPosition(Constants.kArmMidCargo);
        _setpointName = "Mid Cargo";
    }
    public void setPositionTopCargo(){
        setPosition(Constants.kArmTopCargo);
        _setpointName = "Top Cargo";
    }

    public void WriteToDashboard() {
        SmartDashboard.putNumber("armSpeed", _leftTalon.getSelectedSensorVelocity());
        SmartDashboard.putNumber("armPosition", _leftTalon.getSelectedSensorPosition());
        SmartDashboard.putNumber("rightArmSpeed", _rightTalon.getSelectedSensorVelocity());
        SmartDashboard.putNumber("rightArmPosition", _rightTalon.getSelectedSensorPosition());

        SmartDashboard.putString("armSetpointName", _setpointName);
        SmartDashboard.putString("armOrientation", _isReverse ? "Reverse" : "Forward");
    }

	public void ResetSensors() {
        _leftTalon.setSelectedSensorPosition(0);
        _rightTalon.setSelectedSensorPosition(0);
	}

}