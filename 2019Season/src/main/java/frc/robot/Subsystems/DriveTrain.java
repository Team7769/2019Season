package frc.robot.Subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.PIDController;
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
    protected AHRS _gyro;
    protected CANEncoder _leftEncoder;
    private CANEncoder _rightEncoder;
    private CANPIDController _leftSparkPID;
    private CANPIDController _rightSparkPID;
    private PIDController _rotationPID;
    private PIDController _driveStraightPID;
    private DriveStraightWrapper _driveStraightWrapper;
    private PathFollower _pathFollower;

    private boolean _pidEnabled;
    private double _targetDistanceLeft;
    private double _targetDistanceRight;
    private double _targetAngle;
    private double _tolerance;

    private NetworkTable _table = NetworkTableInstance.getDefault().getTable("limelight");
    private NetworkTableEntry _angleEntry = _table.getEntry("tx");

    public DriveTrain(CANSparkMax leftMotor, CANSparkMax rightMotor, AHRS gyro){
        try {
            _robotDrive = new DifferentialDrive(leftMotor, rightMotor);
            _leftSpark = leftMotor;
            _rightSpark = rightMotor;
            _leftSpark.setControlFramePeriodMs(10);
            _rightSpark.setControlFramePeriodMs(10);
            _leftSpark.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 10);
            _rightSpark.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 10);
            _gyro = gyro;
        } catch (Exception ex){
            ex.printStackTrace();
        }

        try {
            _leftEncoder = _leftSpark.getEncoder();
            _leftEncoder.setPositionConversionFactor(Constants.kDrivePositionConversion);
            _leftEncoder.setVelocityConversionFactor(Constants.kDriveVelocityConversion);

            _rightEncoder = _rightSpark.getEncoder();
            _rightEncoder.setPositionConversionFactor(Constants.kDrivePositionConversion);
            _rightEncoder.setVelocityConversionFactor(Constants.kDriveVelocityConversion);

            _leftSparkPID = _leftSpark.getPIDController();
            _rightSparkPID = _rightSpark.getPIDController();

            _rotationPID = new PIDController(Constants.kDriveRotationP, Constants.kDriveRotationI, Constants.kDriveRotationD, _gyro, new DriveRotationPIDOutput(this));
            _rotationPID.setAbsoluteTolerance(Constants.kDriveRotationTolerance);
            _rotationPID.setOutputRange(-0.5, 0.5);

            _driveStraightWrapper = new DriveStraightWrapper(this);

            _driveStraightPID = new PIDController(Constants.kDistancePIDP, Constants.kDistancePIDI, Constants.kDistancePIDD, _driveStraightWrapper, _driveStraightWrapper);
            _driveStraightPID.setOutputRange(1.0, 1.0);
        _pathFollower = new PathFollower(this);
        } catch (Exception ex){
            ex.printStackTrace();
        }
        _pidEnabled = false;
        _targetDistanceLeft = 0;
        _targetDistanceRight = 0;
        _targetAngle = 0;
        _tolerance = 8;
    }
    public void setRealWorldUnits(){
        _leftEncoder.setPositionConversionFactor(Constants.kDrivePositionConversion);
        _leftEncoder.setVelocityConversionFactor(Constants.kDriveVelocityConversion);

        _rightEncoder.setPositionConversionFactor(Constants.kDrivePositionConversion);
        _rightEncoder.setVelocityConversionFactor(Constants.kDriveVelocityConversion);
    }
    public void updatePIDFromDashboard() {
        _leftSparkPID.setP(SmartDashboard.getNumber("leftSparkP", Constants.kVelocityPIDP));
        _leftSparkPID.setI(SmartDashboard.getNumber("leftSparkI", Constants.kVelocityPIDI));
        _leftSparkPID.setD(SmartDashboard.getNumber("leftSparkD", Constants.kVelocityPIDD));
        _leftSparkPID.setFF(SmartDashboard.getNumber("leftSparkFF", Constants.kVelocityPIDFF));

        _rightSparkPID.setP(SmartDashboard.getNumber("rightSparkP", Constants.kVelocityPIDP));
        _rightSparkPID.setI(SmartDashboard.getNumber("rightSparkI", Constants.kVelocityPIDI));
        _rightSparkPID.setD(SmartDashboard.getNumber("rightSparkD", Constants.kVelocityPIDD));
        _rightSparkPID.setFF(SmartDashboard.getNumber("rightSparkFF", Constants.kVelocityPIDFF));
        
        _rotationPID.setP(SmartDashboard.getNumber("rotationPIDP", Constants.kDriveRotationP));
        _rotationPID.setI(SmartDashboard.getNumber("rotationPIDI", Constants.kDriveRotationI));        
        _rotationPID.setD(SmartDashboard.getNumber("rotationPIDD", Constants.kDriveRotationD));
        
        _driveStraightPID.setP(SmartDashboard.getNumber("driveStraightPIDP", Constants.kDistancePIDP));
        _driveStraightPID.setI(SmartDashboard.getNumber("driveStraightPIDI", Constants.kDistancePIDI));        
        _driveStraightPID.setD(SmartDashboard.getNumber("driveStraightPIDD", Constants.kDistancePIDD));
    }
    public void setTargetDistance(double distance){
        setDistancePIDValues();
        _targetDistanceLeft = distance;
        _targetDistanceRight = distance;
    }
    private void setDistancePIDValues(){
        _leftSparkPID.setP(Constants.kDistancePIDP);
        _leftSparkPID.setI(Constants.kDistancePIDI);
        _leftSparkPID.setD(Constants.kDistancePIDD);
        
        _rightSparkPID.setP(Constants.kDistancePIDP);
        _rightSparkPID.setI(Constants.kDistancePIDI);
        _rightSparkPID.setD(Constants.kDistancePIDD);

        _leftSparkPID.setOutputRange(-1.0, 1.0);
        _rightSparkPID.setOutputRange(-1.0, 1.0);
    }
    private void setPathFollowPIDValues(){
        _leftSparkPID.setP(0);
        _leftSparkPID.setI(0);
        _leftSparkPID.setD(0);
        
        _rightSparkPID.setP(0);
        _rightSparkPID.setI(0);
        _rightSparkPID.setD(0);
    }
    /**
     * Drive to a target distance, while attempting to maintain the provided angle.
     * @param distance Distance to travel
     * @param targetAngle Angle to follow
     */
    public void driveDistance(double distance, double targetAngle) {
        //Ensure Sparks are in Coast
        if (_leftSpark.getIdleMode() != IdleMode.kCoast){
            _leftSpark.setIdleMode(IdleMode.kCoast);
            _rightSpark.setIdleMode(IdleMode.kCoast);
        }
        //Stop whatever we are currently doing
        stop();
        _targetDistanceLeft = distance;
        _targetDistanceRight = -distance;
        _pidEnabled = true;

        System.out.println(Timer.getFPGATimestamp() +  ": Enabling Drive Straight PID to heading: " + targetAngle);
        _driveStraightPID.setSetpoint(distance);
        _driveStraightPID.enable();
        _driveStraightWrapper.enableRotationPID(targetAngle);     

        System.out.println(Timer.getFPGATimestamp() +  ": Enabled Drive Straight PID to distance: " + distance + ". Initial error: " + (distance - _leftEncoder.getPosition()));
    }
    /**
     * Drive to a target distance, while maintaining the current angle.
     * @param distance Distance to travel
     */
    public void driveDistance(double distance) {
        if (_leftSpark.getIdleMode() != IdleMode.kCoast){
            _leftSpark.setIdleMode(IdleMode.kCoast);
            _rightSpark.setIdleMode(IdleMode.kCoast);
        }

        //Stop whatever we are currently doing
        stop();
        _targetDistanceLeft = distance;
        _targetDistanceRight = -distance;
        _pidEnabled = true;

        System.out.println(Timer.getFPGATimestamp() +  ": Enabling Drive Straight PID maintaining current heading.");
        _driveStraightPID.setSetpoint(distance);
        _driveStraightPID.enable();
        _driveStraightWrapper.enableRotationPID(_gyro.getAngle());     

        System.out.println(Timer.getFPGATimestamp() +  ": Enabled Drive Straight PID to distance: " + distance + ". Initial error: " + (distance - _leftEncoder.getPosition()));
    }
    /**
     * Turn to the specified angle.
     * @param angle Angle to turn to
     */
    public void runDriveMotors(double leftSpeed, double rightSpeed){
        //_leftSpark.set(leftSpeed);
        //_rightSpark.set(rightSpeed);
        _leftSparkPID.setReference(leftSpeed, ControlType.kVoltage);
        _rightSparkPID.setReference(rightSpeed, ControlType.kVoltage);
    }
    public void turnToAngle(double angle){
        _leftSpark.setIdleMode(IdleMode.kBrake);
        _rightSpark.setIdleMode(IdleMode.kBrake);
        _pidEnabled = true;
        _targetAngle = angle;
        _rotationPID.setSetpoint(angle);
        _rotationPID.enable();
    }

    public void turnViaCamera()
    {
        double angleOffset = _angleEntry.getDouble(0.0);

        turnToAngle(_gyro.getYaw() + angleOffset);
    }
    
    public boolean isAngleOnTarget(){
        if (Math.abs(_targetAngle - _gyro.getAngle()) < Constants.kDriveRotationTolerance && (Math.abs(_rotationPID.get()) < .35)){
            return true;
        }
        return false;
    }
    public void stop() {
        _pidEnabled = false;
        _rotationPID.disable();
        _driveStraightPID.disable();
        _driveStraightWrapper.disableRotationPID();
        //_pathFollower.killPath();
        _leftSpark.set(0.0);
        _rightSpark.set(0.0);
    }
    public boolean isDistanceOnTarget() {
        double currentDistance = _leftEncoder.getPosition();
        if ((Math.abs(_targetDistanceLeft - currentDistance)) < _tolerance){
            return true;
        }
        return false;
    }
    public void curvatureDrive(double speed, double rotation, boolean isQuickTurn) {
        if (_leftSpark.getIdleMode() != IdleMode.kCoast){
            _leftSpark.setIdleMode(IdleMode.kCoast);
            _rightSpark.setIdleMode(IdleMode.kCoast);
        }
        double dampen = 0.90;
        double throttle = Constants.kIsTestRobot ? -speed * dampen : -speed * dampen;
        double turn = Constants.kIsTestRobot ? rotation : rotation;
        _robotDrive.curvatureDrive(throttle, turn, isQuickTurn);
    }
    public void arcadeDrive(double speed, double rotation){
        if (_leftSpark.getIdleMode() != IdleMode.kCoast){
            _leftSpark.setIdleMode(IdleMode.kCoast);
            _rightSpark.setIdleMode(IdleMode.kCoast);
        }
        double dampen = 0.90;
        double throttle = Constants.kIsTestRobot ? -speed * dampen : -speed * dampen;
        double turn = Constants.kIsTestRobot ? rotation : rotation;
        _robotDrive.arcadeDrive(throttle, turn, true);
    }
    public void tankDrive(double leftSpeed, double rightSpeed){
        _robotDrive.tankDrive(leftSpeed, rightSpeed);
    }
    public void WriteToDashboard(){
        if (Constants.kIsTestRobot){
            updatePIDFromDashboard();
        }
        SmartDashboard.putNumber("leftDrivePosition", _leftEncoder.getPosition());
        SmartDashboard.putNumber("leftDriveVelocity", _leftEncoder.getVelocity());
        SmartDashboard.putNumber("rightDrivePosition", _rightEncoder.getPosition());
        SmartDashboard.putNumber("rightDriveVelocity", _rightEncoder.getVelocity());
        SmartDashboard.putNumber("rightDriveOutput", _rightSpark.get());
        SmartDashboard.putNumber("leftDriveOutput", _leftSpark.get());
        SmartDashboard.putNumber("gyroAngle", _gyro.getAngle());
        SmartDashboard.putNumber("rotationPIDSetpoint", _rotationPID.getSetpoint());
        SmartDashboard.putNumber("rotationPIDP", _rotationPID.getP());
        SmartDashboard.putNumber("rotationPIDI", _rotationPID.getI());
        SmartDashboard.putNumber("rotationPIDD", _rotationPID.getD());
        SmartDashboard.putNumber("rotationPIDOutput", _rotationPID.get());
        SmartDashboard.putNumber("driveStraightRotationPIDP", _driveStraightPID.getP());
        SmartDashboard.putNumber("driveStraightRotationPIDI", _driveStraightPID.getI());
        SmartDashboard.putNumber("driveStraightRotationPIDD", _driveStraightPID.getD());
        SmartDashboard.putNumber("targetAngle", _targetAngle);
        SmartDashboard.putBoolean("pidEnabled", _pidEnabled);
        
    }

    @Override
    public void ResetSensors() {
        _leftEncoder.setPosition(0);
        _rightEncoder.setPosition(0);
        _gyro.reset();
        _targetDistanceLeft = 0;
        _targetDistanceRight = 0;
    }
    public void resetEncoders(){
        _leftEncoder.setPosition(0);
        _rightEncoder.setPosition(0);
    }
    public double getLeftDistance(){
        return _leftEncoder.getPosition();
    }
    public double getRightDistance(){
        return _rightEncoder.getPosition();
    }
    public double getAngle(){
        return _gyro.getAngle();
    }
    public void setPath(String pathName, boolean isReverse){
        stop();
        setPathFollowPIDValues();
        _pathFollower.setPath(pathName, isReverse);
    }
    public void startPath(){
        _pathFollower.startPath();
    }
    public boolean isFinishedFollowingPath(){
        return _pathFollower.isFinished();
    }

    public boolean isPIDEnabled()
    {
        return _pidEnabled;
    }

    public void disableRotationPID()
    {
        if(_rotationPID.isEnabled()){ 
            _rotationPID.disable();
            _pidEnabled = false;
         }   
    }


}