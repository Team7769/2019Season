package frc.robot.Subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DriveTrain implements Subsystem {
    private DifferentialDrive _robotDrive;
    private CANSparkMax _leftSpark;
    private CANSparkMax _rightSpark;
    private CANEncoder _leftEncoder;
    private CANEncoder _rightEncoder;
    private CANPIDController _leftSparkPID;
    private CANPIDController _rightSparkPID;

    public DriveTrain(CANSparkMax leftMotor, CANSparkMax rightMotor){
        _robotDrive = new DifferentialDrive(leftMotor, rightMotor);
        _leftSpark = leftMotor;
        _rightSpark = rightMotor;

        _leftEncoder = _leftSpark.getEncoder();
        _rightEncoder = _rightSpark.getEncoder();
        _leftSparkPID = _rightSpark.getPIDController();
        _rightSparkPID = _rightSpark.getPIDController();
    }

    public void curvatureDrive(double speed, double rotation, boolean isQuickTurn) {
        _robotDrive.curvatureDrive(speed, rotation, isQuickTurn);
    }
    public void arcadeDrive(double speed, double rotation){
        _robotDrive.arcadeDrive(speed, rotation);
    }
    public void tankDrive(double leftSpeed, double rightSpeed){
        _robotDrive.tankDrive(leftSpeed, rightSpeed);
    }
    public void WriteToDashboard(){
        SmartDashboard.putNumber("leftDrivePosition", _leftEncoder.getPosition());
        SmartDashboard.putNumber("leftDriveVelocity", _leftEncoder.getVelocity());
        SmartDashboard.putNumber("rightDrivePosition", _rightEncoder.getPosition());
        SmartDashboard.putNumber("rightDriveVelocity", _rightEncoder.getVelocity());
        SmartDashboard.putNumber("leftSparkP", _leftSparkPID.getP());
        SmartDashboard.putNumber("leftSparkI", _leftSparkPID.getI());
        SmartDashboard.putNumber("leftSparkD", _leftSparkPID.getD());
        SmartDashboard.putNumber("leftSparkFF", _leftSparkPID.getFF());
        SmartDashboard.putNumber("rightSparkP", _rightSparkPID.getP());
        SmartDashboard.putNumber("rightSparkI", _rightSparkPID.getI());
        SmartDashboard.putNumber("rightSparkD", _rightSparkPID.getD());
        SmartDashboard.putNumber("rightSparkFF", _rightSparkPID.getFF());
    }

    @Override
    public void ResetSensors() {
        //Do nothing
    }
    

}