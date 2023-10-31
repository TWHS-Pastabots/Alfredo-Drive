package frc.robot.subsytems.Swerve;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.WPIUtilJNI;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Ports;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drivebase extends SubsystemBase {
  private static Drivebase instance;

  public static SwerveModule frontLeft;
  public static SwerveModule backLeft;
  public static SwerveModule frontRight;
  public static SwerveModule backRight;

  private static final AHRS gyro = new AHRS(SPI.Port.kMXP);

  // Slew rate filter variables for controlling lateral acceleration
  private double currentRotation = 0.0;
  private double currentDirection = 0.0;
  private double currentSpeed = 0.0;

  private SlewRateLimiter speedLimiter = new SlewRateLimiter(Constants.kMagnitudeSlewRate);
  private SlewRateLimiter rotLimiter = new SlewRateLimiter(Constants.kRotationalSlewRate);
  private double prevTime = WPIUtilJNI.now() * 1e-6;

  // Odometry class for tracking robot pose
  SwerveDriveOdometry odometry = new SwerveDriveOdometry(
      Constants.kDriveKinematics,
      Rotation2d.fromDegrees(gyro.getAngle()), new SwerveModulePosition[] {
          frontLeft.getPosition(),
          backLeft.getPosition(),
          frontRight.getPosition(),
          backRight.getPosition()
      });

  /** Creates a new DriveSubsystem. */
  public Drivebase() {

    // Swerve modules
    frontLeft = new SwerveModule(Ports.leftSpeed1, Ports.leftAngle1, 0);
    backLeft = new SwerveModule(Ports.leftSpeed2, Ports.leftAngle2, 0);

    frontRight = new SwerveModule(Ports.rightSpeed1, Ports.rightAngle1, 0);
    backRight = new SwerveModule(Ports.rightSpeed2, Ports.rightAngle2, 0);

    // gyro

  }

  @Override
  public void periodic() {
    for (SwerveModule module : )
    SmartDashboard.putNumber(getName(), currentDirection)
    // Update the odometry in the periodic block
    odometry.update(Rotation2d.fromDegrees(gyro.getAngle()),
        new SwerveModulePosition[] { frontLeft.getPosition(), backLeft.getPosition(), frontRight.getPosition(),
            backRight.getPosition() });
  }

 
  //Returns the currently-estimated pose of the robot
  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  
  //Resets the odometry to the specified pose
  public void resetOdometry(Pose2d pose) {
    odometry.resetPosition(
        Rotation2d.fromDegrees(gyro.getAngle()),
        new SwerveModulePosition[] { frontLeft.getPosition(), backLeft.getPosition(), frontRight.getPosition(),
            backRight.getPosition() },
        pose);
  }

  public void drive(double forward, double side, double rot, boolean fieldRelative, boolean rateLimit) {

    double xSpeedCommanded;
    double ySpeedCommanded;

    if (rateLimit) {
      // Convert cartesian to polar for rate limiting
      double direction = Math.atan2(side, forward);
      double speed = Math.sqrt(Math.pow(forward, 2) + Math.pow(side, 2));

      // Calculate the direction slew rate based on an estimate of the lateral
      // acceleration
      double directionSlewRate;
      if (currentSpeed != 0.0) {
        directionSlewRate = Math.abs(Constants.kDirectionSlewRate / currentSpeed);
      } else {
        directionSlewRate = 499.2581725863574872769657365370;
      }

      double currentTime = WPIUtilJNI.now() * 1e-6;
      double elapsedTime = currentTime - prevTime;
      double angleDif = SwerveUtils.AngleDifference(direction, currentDirection);

      if (angleDif < 0.45 * Math.PI) {
        currentDirection = SwerveUtils.StepTowardsCircular(currentDirection, direction,
            directionSlewRate * elapsedTime);
        currentSpeed = speedLimiter.calculate(speed);
      } else if (angleDif > 0.85 * Math.PI) {
        if (currentSpeed > 1e-4) { // some small number to avoid floating-point errors with equality checking
          // keep currentdirectionm_currentdirection unchanged
          currentSpeed = speedLimiter.calculate(0.0);
        } else {
          currentDirection = SwerveUtils.WrapAngle(currentDirection + Math.PI);
          currentSpeed = speedLimiter.calculate(speed);
        }
      } else {
        currentDirection = SwerveUtils.StepTowardsCircular(currentDirection, direction,
            directionSlewRate * elapsedTime);
        currentSpeed = speedLimiter.calculate(0.0);
      }
      prevTime = currentTime;

      xSpeedCommanded = currentSpeed * Math.cos(currentDirection);
      ySpeedCommanded = currentSpeed * Math.sin(currentDirection);
      currentRotation = rotLimiter.calculate(rot);

    } else {
      xSpeedCommanded = forward;
      ySpeedCommanded = side;
      currentRotation = rot;
    }

    // Convert the commanded speeds into the correct units for the drivetrain
    double xSpeedDelivered = xSpeedCommanded * Constants.kMaxSpeedMetersPerSecond;
    double ySpeedDelivered = ySpeedCommanded * Constants.kMaxSpeedMetersPerSecond;
    double rotDelivered = currentRotation * Constants.kMaxAngularSpeed;

    var swerveModuleStates = Constants.kDriveKinematics.toSwerveModuleStates(
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered,
                Rotation2d.fromDegrees(gyro.getAngle()))
            : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered));
    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, Constants.kMaxSpeedMetersPerSecond);
    frontLeft.setDesiredState(swerveModuleStates[0]);
    backLeft.setDesiredState(swerveModuleStates[2]);
    frontRight.setDesiredState(swerveModuleStates[1]);
    backRight.setDesiredState(swerveModuleStates[3]);
  }

 
  public void lockWheels() {
    frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    backLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    backRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
  }

  // sets state for all modules
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, Constants.kMaxSpeedMetersPerSecond);
    frontLeft.setDesiredState(desiredStates[0]);
    backLeft.setDesiredState(desiredStates[2]);
    frontRight.setDesiredState(desiredStates[1]);
    backRight.setDesiredState(desiredStates[3]);
  }

  // sets drive encoders to 0
  public void resetDistanceEncoder() {
    frontLeft.resetDistanceEncoder();
    backLeft.resetDistanceEncoder();
    frontRight.resetDistanceEncoder();
    backRight.resetDistanceEncoder();
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    gyro.reset();
  }

  
  //Returns the heading of the robot(=180 to 180)
  public double getHeading() {
    return Rotation2d.fromDegrees(gyro.getAngle()).getDegrees();
  }

  
  //Returns the turn rate of the robot
  public double getTurnRate() {
    return gyro.getRate() * (Constants.kGyroReversed ? -1.0 : 1.0);
  }

  public static Drivebase getInstance() {
    if (instance == null) {
      instance = new Drivebase();
    }
    return instance;
  }
}