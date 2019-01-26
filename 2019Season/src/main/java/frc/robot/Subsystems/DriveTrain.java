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
        try {
            _robotDrive = new DifferentialDrive(leftMotor, rightMotor);
            _leftSpark = leftMotor;
            _rightSpark = rightMotor;
        } catch (Exception ex){
            ex.printStackTrace();
        }

        try {
            _leftEncoder = _leftSpark.getEncoder();
            _rightEncoder = _rightSpark.getEncoder();
            _leftSparkPID = _leftSpark.getPIDController();
            _rightSparkPID = _rightSpark.getPIDController();
        } catch (Exception ex){
            ex.printStackTrace();
        }
    }

    public void curvatureDrive(double speed, double rotation, boolean isQuickTurn) {
        double dampen = 0.6;
        _robotDrive.curvatureDrive(-speed * dampen, rotation * dampen, isQuickTurn);
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
        SmartDashboard.putNumber("rightDriveOutput", _rightSpark.get());
        SmartDashboard.putNumber("leftDriveOutput", _leftSpark.get());
    }

    @Override
    public void ResetSensors() {
        //Do nothing
    }
    

}