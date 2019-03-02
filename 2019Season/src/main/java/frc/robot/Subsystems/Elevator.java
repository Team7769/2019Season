package frc.robot.Subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Configuration.Constants;

public class Elevator implements Subsystem {

    private TalonSRX _leftTalon;
    private TalonSRX _rightTalon;
    private double _setpoint;
    private String _setpointName;
    
    public Elevator(TalonSRX leftTalon, TalonSRX rightTalon){
        _leftTalon = leftTalon;
        _leftTalon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
        _leftTalon.setSelectedSensorPosition(0);

        _rightTalon = rightTalon;
        _rightTalon.setInverted(true);
        _rightTalon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
        _rightTalon.setSelectedSensorPosition(0);

        _setpointName = "Neutral";
        _setpoint = Constants.kElevatorNeutral;

        setPositionPIDValues();
    }
    public void setPositionPIDValues(){
        _leftTalon.config_kP(0, Constants.kElevatorP);
        _leftTalon.config_kI(0, Constants.kElevatorI);
        _leftTalon.config_kD(0, Constants.kElevatorD);
        _leftTalon.config_kF(0, Constants.kElevatorFF);
        
        _rightTalon.config_kP(0, Constants.kElevatorP);
        _rightTalon.config_kI(0, Constants.kElevatorI);
        _rightTalon.config_kD(0, Constants.kElevatorD);
        _rightTalon.config_kF(0, Constants.kElevatorFF);
        
    }
    public void setSpeed(double speed) {
        _leftTalon.set(ControlMode.PercentOutput, speed);
        _rightTalon.set(ControlMode.PercentOutput, speed);
    }
    public void setPosition(double position){
        //_leftTalon.set(ControlMode.Position, position);
        //_rightTalon.set(ControlMode.Position, position);
        _setpoint = position;
    }
    
    public void setPositionNeutral(){
        setPosition(Constants.kElevatorNeutral);
        _setpointName = "Neutral";
    }
    public void setPositionLowHatch(){
        setPosition(Constants.kElevatorLowHatch);
        _setpointName = "Low Hatch";
    }
    public void setPositionLowCargo(){
        setPosition(Constants.kElevatorLowCargo);
        _setpointName = "Low Cargo";
    }
    public void setPositionMidHatch(){
        setPosition(Constants.kElevatorMidHatch);
        _setpointName = "Mid Hatch";
    }
    public void setPositionMidCargo(){
        setPosition(Constants.kElevatorMidCargo);
        _setpointName = "Mid Cargo";
    }
    public void setPositionTopCargo(){
        setPosition(Constants.kElevatorTopCargo);
        _setpointName = "Top Cargo";
    }
    
    @Override
    public void WriteToDashboard() {
        SmartDashboard.putNumber("leftElevatorSpeed", _leftTalon.getSelectedSensorVelocity());
        SmartDashboard.putNumber("leftElevatorPosition", _leftTalon.getSelectedSensorPosition());
        SmartDashboard.putNumber("rightElevatorSpeed", _rightTalon.getSelectedSensorVelocity());
        SmartDashboard.putNumber("rightElevatorPosition", _rightTalon.getSelectedSensorPosition());

        SmartDashboard.putString("elevatorSetpointName", _setpointName);
    }

    @Override
    public void ResetSensors() {
        _leftTalon.setSelectedSensorPosition(0);
        _rightTalon.setSelectedSensorPosition(0);
    }

}