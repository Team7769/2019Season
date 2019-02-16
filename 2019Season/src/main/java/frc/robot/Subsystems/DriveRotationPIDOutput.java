package frc.robot.Subsystems;

import edu.wpi.first.wpilibj.PIDOutput;

public class DriveRotationPIDOutput implements PIDOutput {

    public DriveTrain _driveTrain;

    public DriveRotationPIDOutput(DriveTrain driveTrain){
        _driveTrain = driveTrain;
    }

    @Override
    public void pidWrite(double output) {
        _driveTrain.tankDrive(output, -output);
    }

}