package frc.robot.Subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANPIDController.AccelStrategy;

import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Configuration.Constants;
import frc.robot.Utilities.ProfilePoint;
import frc.robot.Utilities.TrapezoidalMotionProfile;

public class DriveTrain implements Subsystem {
    private DifferentialDrive _robotDrive;
    private CANSparkMax _leftSpark;
    private CANSparkMax _rightSpark;
    private CANEncoder _leftEncoder;
    private CANEncoder _rightEncoder;
    private CANPIDController _leftSparkPID;
    private CANPIDController _rightSparkPID;
    private TrapezoidalMotionProfile _profile;
    private Timer _timer;

    private boolean _pidEnabled;
    private double _targetDistanceLeft;
    private double _targetDistanceRight;
    private double _tolerance;

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
            _leftEncoder.setPositionConversionFactor(Constants.kDrivePositionConversion);
            //_leftEncoder.setVelocityConversionFactor(Constants.kDriveVelocityConversion);

            _rightEncoder = _rightSpark.getEncoder();
            _rightEncoder.setPositionConversionFactor(Constants.kDrivePositionConversion);
            //_rightEncoder.setVelocityConversionFactor(Constants.kDriveVelocityConversion);

            _leftSparkPID = _leftSpark.getPIDController();
            _rightSparkPID = _rightSpark.getPIDController();

            //_leftSparkPID.setOutputRange(-0.75, 0.75);
            //_rightSparkPID.setOutputRange(-0.75, 0.75);
            setVelocityPIDValues();

        } catch (Exception ex){
            ex.printStackTrace();
        }
        _pidEnabled = false;
        _targetDistanceLeft = 0;
        _targetDistanceRight = 0;
        _tolerance = 8;
        _profile = null;
        _timer = new Timer();
    }
    public void setSmartMotionParameters(double maxVelocity, double maxAcceleration){

        _leftSparkPID.setSmartMotionMaxAccel(maxAcceleration, 0);
        _leftSparkPID.setSmartMotionMaxVelocity(maxVelocity, 0);
        _leftSparkPID.setSmartMotionMinOutputVelocity(0, 0);

        _rightSparkPID.setSmartMotionMaxAccel(maxAcceleration, 0);
        _rightSparkPID.setSmartMotionMaxVelocity(maxVelocity, 0);
        _rightSparkPID.setSmartMotionMinOutputVelocity(0, 0);
    }
    public void updatePIDFromDashboard() {
        _leftSparkPID.setP(SmartDashboard.getNumber("leftSparkP", Constants.kVelocityPIDP));
        _leftSparkPID.setI(SmartDashboard.getNumber("leftSparkI", Constants.kVelocityPIDI));
        _leftSparkPID.setD(SmartDashboard.getNumber("leftSparkD", Constants.kVelocityPIDD));
        _leftSparkPID.setFF(SmartDashboard.getNumber("leftSparkFF", 0.0));

        _rightSparkPID.setP(SmartDashboard.getNumber("leftSparkP", Constants.kVelocityPIDP));
        _rightSparkPID.setI(SmartDashboard.getNumber("leftSparkI", Constants.kVelocityPIDI));
        _rightSparkPID.setD(SmartDashboard.getNumber("leftSparkD", Constants.kVelocityPIDD));
        _rightSparkPID.setFF(SmartDashboard.getNumber("leftSparkFF", 0.0));
    }
    public void driveDistanceSmartMotion(double distance){
        _targetDistanceLeft = distance;
        _targetDistanceRight = distance;
        _leftSparkPID.setReference(distance, ControlType.kSmartMotion);
        _rightSparkPID.setReference(-distance, ControlType.kSmartMotion);
    }
    public void setTargetDistance(double distance){
        setDistancePIDValues();
        _targetDistanceLeft = distance;
        _targetDistanceRight = distance;
    }
    public void setTargetProfile(double distance) {
        _profile = new TrapezoidalMotionProfile(distance, 50, 40);
        _targetDistanceLeft = distance;
        setVelocityPIDValues();
        _timer.reset();
        _timer.start();
    }    
    private void setVelocityPIDValues(){
        _leftSparkPID.setP(Constants.kVelocityPIDP);
        _leftSparkPID.setI(Constants.kVelocityPIDI);
        _leftSparkPID.setD(Constants.kVelocityPIDD);
        _rightSparkPID.setP(Constants.kVelocityPIDP);
        _rightSparkPID.setI(Constants.kVelocityPIDI);
        _rightSparkPID.setD(Constants.kVelocityPIDD);
        
        _leftSparkPID.setOutputRange(0, 1.0);
        _rightSparkPID.setOutputRange(-1.0, 0);

    }
    private void setDistancePIDValues(){
        _leftSparkPID.setP(Constants.kDistancePIDP);
        _leftSparkPID.setI(Constants.kDistancePIDI);
        _leftSparkPID.setD(Constants.kDistancePIDD);
        
        _rightSparkPID.setP(Constants.kDistancePIDP);
        _rightSparkPID.setI(Constants.kDistancePIDI);
        _rightSparkPID.setD(Constants.kDistancePIDD);

        
        _leftSparkPID.setOutputRange(-0.75, 0.75);
        _rightSparkPID.setOutputRange(-0.75, 0.75);

    }
    public void followProfile(){
        ProfilePoint target = _profile.getAtTime(_timer.get());
        SmartDashboard.putNumber("profileTarget", target.vel);
        SmartDashboard.putNumber("timer", _timer.get());
        SmartDashboard.putNumber("positionTarget", _targetDistanceLeft);
        _leftSparkPID.setReference(target.vel * 100, ControlType.kVelocity);
        _rightSparkPID.setReference(-target.vel * 100, ControlType.kVelocity);

        //_leftSparkPID.setReference(4300, ControlType.kVelocity);
        //_rightSparkPID.setReference(-4300, ControlType.kVelocity);
    }
    public void driveDistance() {
        _leftSparkPID.setReference(_targetDistanceLeft, ControlType.kPosition);
        _rightSparkPID.setReference(_targetDistanceRight, ControlType.kPosition);
        _pidEnabled = true;
    }
    public void stop() {
        _pidEnabled = false;
        _leftSpark.set(0.0);
        _rightSpark.set(0.0);
    }
    public boolean isDistanceOnTarget() {
        double currentDistance = _leftEncoder.getPosition();
        if ((Math.abs(_targetDistanceLeft) - Math.abs(currentDistance)) < _tolerance){
            return true;
        }
        return false;
    }
    public boolean isProfileOnTarget(){
        if (_profile.getAtTime(_timer.get()).vel == 0){
            return true;
        }
        return false;
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
        if (Constants.kIsTestRobot){
            updatePIDFromDashboard();
        }
    }

    @Override
    public void ResetSensors() {
        _leftEncoder.setPosition(0);
        _rightEncoder.setPosition(0);
    }
    

}