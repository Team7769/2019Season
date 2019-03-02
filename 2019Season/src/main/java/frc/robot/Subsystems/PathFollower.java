package frc.robot.Subsystems;

import java.io.IOException;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Timer;
import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.PathfinderFRC;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.followers.EncoderFollower;

public class PathFollower {

    private DriveTrain _driveTrain;
    private Trajectory _leftTrajectory;
    private Trajectory _rightTrajectory;
    private EncoderFollower _leftFollower;
    private EncoderFollower _rightFollower;
    private boolean _followingPath;
    private boolean _reversePath;
    private Notifier _notifier;

    private static final int kTicksPerRev = 1;
    private static final double kWheelDiameter = 6.0 / 12.0;
    private static final double kMaxVelocity = 3.0;

    public PathFollower(DriveTrain driveTrain){
        _driveTrain = driveTrain;
        _followingPath = false;

        _leftFollower = new EncoderFollower();
        _rightFollower = new EncoderFollower();
    }
    public boolean isFinished(){
        return _followingPath;
    }
    public void setPath(String pathName, boolean isReverse){
        try {
            System.out.println(Timer.getFPGATimestamp() + ": Setting robot path to: " + pathName);
            
            //Hack, fixed in new version of Pathfinder
            _leftTrajectory = PathfinderFRC.getTrajectory(pathName + ".right");
            _rightTrajectory = PathfinderFRC.getTrajectory(pathName + ".left");
        } catch (IOException ex){
            System.out.println(Timer.getFPGATimestamp() + ": Unable to find path: " + pathName);
            _leftTrajectory = null;
            _rightTrajectory = null;
            ex.printStackTrace();
            return;
        }
        if (_leftTrajectory == null || _rightTrajectory == null){
            return;
        }
        _driveTrain.setGenericUnits();
        _reversePath = isReverse;
        System.out.println(Timer.getFPGATimestamp() + ": Setting path followers.");
        if (isReverse){
            _leftFollower = new EncoderFollower(_rightTrajectory);
            _rightFollower = new EncoderFollower(_leftTrajectory);
            _leftFollower.configureEncoder(_driveTrain.getRightDistance(), kTicksPerRev, kWheelDiameter);
            _leftFollower.configurePIDVA(1.0, 0.0, 0.0, 1 / kMaxVelocity, 0);

            _rightFollower.configureEncoder(_driveTrain.getLeftDistance(), kTicksPerRev, kWheelDiameter);
            _rightFollower.configurePIDVA(1.0, 0.0, 0.0, 1 / kMaxVelocity, 0);
        } else {
            _leftFollower = new EncoderFollower(_leftTrajectory);
            _rightFollower = new EncoderFollower(_rightTrajectory);
            _leftFollower.configureEncoder(_driveTrain.getLeftDistance(), kTicksPerRev, kWheelDiameter);
            _leftFollower.configurePIDVA(.8, 0.0, 0.0, 1 / kMaxVelocity, 0);

            _rightFollower.configureEncoder(-_driveTrain.getRightDistance(), kTicksPerRev, kWheelDiameter);
            _rightFollower.configurePIDVA(.8, 0.0, 0.0, 1 / kMaxVelocity, 0);
        }
        System.out.println("Start distance: " + _driveTrain.getLeftDistance() + ", " + _driveTrain.getRightDistance());
        System.out.println(Timer.getFPGATimestamp() + ": Starting path notifier.");
    }
    public void startPath(){
        _followingPath = true;
        _notifier = new Notifier(this::followPath);
        _notifier.startPeriodic(_leftTrajectory.get(0).dt);
    }
    private void followPath() {
        if (_leftFollower.isFinished() || _rightFollower.isFinished()) {
            _followingPath = false;
          _notifier.stop();
          } else {
            double left_speed = 0;
            double right_speed = 0;
            double heading = 0;
            double desired_heading = 0;
            double heading_difference = 0;
            double turn = 0;


            if (_reversePath){
                left_speed = -_leftFollower.calculate(_driveTrain.getRightDistance());
                right_speed = -_rightFollower.calculate(_driveTrain.getLeftDistance());
                heading = -_driveTrain.getAngle();
                desired_heading = Pathfinder.r2d(-_leftFollower.getHeading());
                heading_difference = Pathfinder.boundHalfDegrees(desired_heading - heading);
                turn =  0.8 * (-1.0/80.0) * heading_difference;
            } else {
                left_speed = _leftFollower.calculate(_driveTrain.getLeftDistance());
                right_speed = _rightFollower.calculate(-_driveTrain.getRightDistance());
                heading = _driveTrain.getAngle();
                desired_heading = Pathfinder.r2d(_leftFollower.getHeading());
                heading_difference = Pathfinder.boundHalfDegrees(desired_heading - heading);
                turn =  0.8 * (-1.0/80.0) * heading_difference;
            }
            System.out.println("Left Speed: " + left_speed + " + Turn: " + turn);
            System.out.println("Right Speed: " + right_speed + " + Turn: " + turn);
          _driveTrain.tankDrive(left_speed + turn, right_speed - turn);
        }
      }
}