package frc.robot.Subsystems;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Configuration.Constants;

public class DriveStraightWrapper implements PIDSource, PIDOutput {

    private DriveTrain _driveTrain;
    private PIDController _rotationPID;

    private double _rotationPower;

    public DriveStraightWrapper(DriveTrain driveTrain){
        _driveTrain = driveTrain;
        _rotationPID = new PIDController(Constants.kDriveRotationP, Constants.kDriveRotationI, Constants.kDriveRotationD, _driveTrain._gyro, new WrapRotationPIDOutput(this));
        _rotationPID.setOutputRange(-1.0, 1.0);
    }
    public void enableRotationPID(double targetAngle){
        if (!_rotationPID.isEnabled()){
            _rotationPID.setSetpoint(targetAngle);
            _rotationPID.enable();
            System.out.println(Timer.getFPGATimestamp() +  ": Enabled rotational PID to heading: " + targetAngle);
        }
    }
    public void disableRotationPID(){
        if (_rotationPID.isEnabled()){
            _rotationPID.disable();
            System.out.println(Timer.getFPGATimestamp() +  ": Disabled rotational PID.");
        }
    }

    @Override
    public void pidWrite(double output) {
        if (output + _rotationPower > 1.0){
            output = 1 - _rotationPower;
        }
        double leftPower = output + _rotationPower;
        double rightPower = -output - _rotationPower;

        _driveTrain.tankDrive(leftPower, rightPower);
    }

    @Override
    public void setPIDSourceType(PIDSourceType pidSource) {
        
    }

    @Override
    public PIDSourceType getPIDSourceType() {
        return PIDSourceType.kDisplacement;
    }

    @Override
    public double pidGet() {
        return _driveTrain._leftEncoder.getPosition();
    }
    public synchronized void setRotationPower(double power){
        this._rotationPower = power;
    }
    private class WrapRotationPIDOutput implements PIDOutput 
	    {

	        private DriveStraightWrapper m_RotationPowerDestination;

	        public WrapRotationPIDOutput(DriveStraightWrapper rotationPowerDesintation) 
	        {
	            if (rotationPowerDesintation == null) {
	                throw new NullPointerException("Given rotationPowerDestination was null");
	            }
	            else{
	                m_RotationPowerDestination = rotationPowerDesintation;            	
	            }
	        }

			@Override
			public void pidWrite(double rotationPower) 
			{
				this.m_RotationPowerDestination.setRotationPower(-rotationPower); // Inverted because it's stupid
			}

	    }

}