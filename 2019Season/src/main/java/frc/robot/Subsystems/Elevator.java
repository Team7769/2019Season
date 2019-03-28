package frc.robot.Subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Configuration.Constants;

public class Elevator implements Subsystem {

    private TalonSRX _talon;
    private double _setpoint;
    private String _setpointName;
    
    public Elevator(TalonSRX talon){
        _talon = talon;
        _talon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
        _talon.setSelectedSensorPosition(0);
        _talon.setSensorPhase(true);
        
        _setpointName = "Neutral";
        _setpoint = Constants.kElevatorNeutral;

        setPositionPIDValues();
    }
    public void setPositionPIDValues(){
        _talon.config_kP(0, Constants.kElevatorP);
        _talon.config_kI(0, Constants.kElevatorI);
        _talon.config_kD(0, Constants.kElevatorD);
        _talon.config_kF(0, Constants.kElevatorFF);
        
    }
    public void setSpeed(double speed) {
        speed = speed * 0.5;
        _talon.set(ControlMode.PercentOutput, speed);
    }
    public void setPosition(double position){
        _talon.set(ControlMode.Position, position);
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
    public void setPositionCargoShipCargo(){
        setPosition(Constants.kElevatorCargoShipCargo);
        _setpointName = "Cargo Ship - Cargo";
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
        SmartDashboard.putNumber("elevatorSpeed", _talon.getSelectedSensorVelocity());
        SmartDashboard.putNumber("elevatorPosition", _talon.getSelectedSensorPosition());
        SmartDashboard.putString("elevatorSetpointName", _setpointName);
        if (_talon.getControlMode() == ControlMode.Position){
            SmartDashboard.putNumber("elevatorSetpoint", _talon.getClosedLoopTarget());
        }
    }

    @Override
    public void ResetSensors() {
        _talon.setSelectedSensorPosition(0);
    }

}