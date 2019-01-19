package frc.robot.Subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Configuration.Constants;

public class Elevator implements Subsystem {

    private TalonSRX _leftTalon;
    private TalonSRX _rightTalon;
    
    public Elevator(TalonSRX leftTalon, TalonSRX rightTalon){
        _leftTalon = leftTalon;
        _leftTalon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
        _leftTalon.setSelectedSensorPosition(0);

        _rightTalon = rightTalon;
        _rightTalon.setInverted(true);
    }
    public void setSpeed(double speed) {
        _leftTalon.set(ControlMode.PercentOutput, speed);
        _rightTalon.set(ControlMode.PercentOutput, speed);
    }
    public void setPosition(double position){
        _leftTalon.set(ControlMode.Position, position);
        _rightTalon.set(ControlMode.Position, position);
    }
    public void setPositionLowHatch(){
        setPosition(Constants.kElevatorLowHatch);
    }
    public void setPositionLowCargo(){
        setPosition(Constants.kElevatorLowCargo);
    }
    public void setPositionMidHatch(){
        setPosition(Constants.kElevatorMidHatch);
    }
    public void setPositionMidCargo(){
        setPosition(Constants.kElevatorMidCargo);
    }
    public void setPositionTopCargo(){
        setPosition(Constants.kElevatorTopCargo);
    }
    
    @Override
    public void WriteToDashboard() {
        SmartDashboard.putNumber("leftElevatorSpeed", _leftTalon.getSelectedSensorVelocity());
        SmartDashboard.putNumber("leftElevatorPosition", _leftTalon.getSelectedSensorPosition());
        SmartDashboard.putNumber("rightElevatorSpeed", _rightTalon.getSelectedSensorVelocity());
        SmartDashboard.putNumber("rightElevatorPosition", _rightTalon.getSelectedSensorPosition());
    }

}