package frc.robot.Subsystems;

import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DriveTrain implements Subsystem {
    private DifferentialDrive _robotDrive;

    public DriveTrain(SpeedController leftMotor, SpeedController rightMotor){
        _robotDrive = new DifferentialDrive(leftMotor, rightMotor);
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
        
    }
    

}