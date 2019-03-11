package frc.robot.Subsystems;

import java.io.IOException;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Timer;
import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.PathfinderFRC;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.followers.DistanceFollower;

public class PathFollower {

    private DriveTrain _driveTrain;
    private Trajectory _leftTrajectory;
    private Trajectory _rightTrajectory;
    private DistanceFollower _leftFollower;
    private DistanceFollower _rightFollower;
    private boolean _followingPath;
    private boolean _reversePath;
    private Notifier _notifier;
    private static final double kMaxVelocity = 8.0;

    public PathFollower(DriveTrain driveTrain){
        _driveTrain = driveTrain;
        _followingPath = false;

        _leftFollower = new DistanceFollower();
        _rightFollower = new DistanceFollower();
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
        _reversePath = isReverse;
        System.out.println(Timer.getFPGATimestamp() + ": Setting path followers.");
        if (isReverse){
            _leftFollower = new DistanceFollower(_rightTrajectory);
            _rightFollower = new DistanceFollower(_leftTrajectory);
            _leftFollower.configurePIDVA(.85, 0.0, 0.0, 1 / kMaxVelocity, 0);

            _rightFollower.configurePIDVA(.85, 0.0, 0.0, 1 / kMaxVelocity, 0);
        } else {
            _leftFollower = new DistanceFollower(_leftTrajectory);
            _rightFollower = new DistanceFollower(_rightTrajectory);
            _leftFollower.configurePIDVA(.85, 0.0, 0.0, 1 / kMaxVelocity, 0);

            _rightFollower.configurePIDVA(.85, 0.0, 0.0, 1 / kMaxVelocity, 0);
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
            int negateTurn = 1;

            if (_reversePath){
                left_speed = _leftFollower.calculate(_driveTrain.getRightDistance());
                right_speed = _rightFollower.calculate(-_driveTrain.getLeftDistance());
                heading = -_driveTrain.getAngle();
                desired_heading = -Pathfinder.r2d(_leftFollower.getHeading());
                heading_difference = Pathfinder.boundHalfDegrees(desired_heading - heading);
                turn =  0.8 * (-1.0/80.0) * heading_difference;
                //negateTurn = -1;
                //System.out.println("Heading: " + heading + " + Desired Heading: " + desired_heading);
                
            } else {
                left_speed = _leftFollower.calculate(_driveTrain.getLeftDistance());
                right_speed = _rightFollower.calculate(-_driveTrain.getRightDistance());
                heading = -_driveTrain.getAngle();
                desired_heading = -Pathfinder.r2d(_leftFollower.getHeading());
                heading_difference = Pathfinder.boundHalfDegrees(desired_heading - heading);
                turn =  0.8 * (-1.0/80.0) * heading_difference;
            }
            System.out.println("Left Speed: " + left_speed + " + Turn: " + turn);
            System.out.println("Right Speed: " + right_speed + " + Turn: " + turn);
            if (_reversePath){
                _driveTrain.runDriveMotors(-(right_speed + turn), left_speed - turn);
            } else {
                _driveTrain.runDriveMotors(left_speed + turn, -(right_speed - turn));
            }
        }
      }
}