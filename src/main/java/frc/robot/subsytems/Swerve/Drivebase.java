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
  private double m_currentRotation = 0.0;
  private double m_currentTranslationDir = 0.0;
  private double m_currentTranslationMag = 0.0;

  private SlewRateLimiter m_magLimiter = new SlewRateLimiter(Constants.kMagnitudeSlewRate);
  private SlewRateLimiter m_rotLimiter = new SlewRateLimiter(Constants.kRotationalSlewRate);
  private double m_prevTime = WPIUtilJNI.now() * 1e-6;

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

  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative, boolean rateLimit) {

    double xSpeedCommanded;
    double ySpeedCommanded;

    if (rateLimit) {
      // Convert XY to polar for rate limiting
      double inputTranslationDir = Math.atan2(ySpeed, xSpeed);
      double inputTranslationMag = Math.sqrt(Math.pow(xSpeed, 2) + Math.pow(ySpeed, 2));

      // Calculate the direction slew rate based on an estimate of the lateral
      // acceleration
      double directionSlewRate;
      if (m_currentTranslationMag != 0.0) {
        directionSlewRate = Math.abs(Constants.kDirectionSlewRate / m_currentTranslationMag);
      } else {
        directionSlewRate = 499.2581725863574872769657365370;
      }

      double currentTime = WPIUtilJNI.now() * 1e-6;
      double elapsedTime = currentTime - m_prevTime;
      double angleDif = SwerveUtils.AngleDifference(inputTranslationDir, m_currentTranslationDir);
      if (angleDif < 0.45 * Math.PI) {
        m_currentTranslationDir = SwerveUtils.StepTowardsCircular(m_currentTranslationDir, inputTranslationDir,
            directionSlewRate * elapsedTime);
        m_currentTranslationMag = m_magLimiter.calculate(inputTranslationMag);
      } else if (angleDif > 0.85 * Math.PI) {
        if (m_currentTranslationMag > 1e-4) { // some small number to avoid floating-point errors with equality checking
          // keep currentTranslationDir unchanged
          m_currentTranslationMag = m_magLimiter.calculate(0.0);
        } else {
          m_currentTranslationDir = SwerveUtils.WrapAngle(m_currentTranslationDir + Math.PI);
          m_currentTranslationMag = m_magLimiter.calculate(inputTranslationMag);
        }
      } else {
        m_currentTranslationDir = SwerveUtils.StepTowardsCircular(m_currentTranslationDir, inputTranslationDir,
            directionSlewRate * elapsedTime);
        m_currentTranslationMag = m_magLimiter.calculate(0.0);
      }
      m_prevTime = currentTime;

      xSpeedCommanded = m_currentTranslationMag * Math.cos(m_currentTranslationDir);
      ySpeedCommanded = m_currentTranslationMag * Math.sin(m_currentTranslationDir);
      m_currentRotation = m_rotLimiter.calculate(rot);

    } else {
      xSpeedCommanded = xSpeed;
      ySpeedCommanded = ySpeed;
      m_currentRotation = rot;
    }

    // Convert the commanded speeds into the correct units for the drivetrain
    double xSpeedDelivered = xSpeedCommanded * Constants.kMaxSpeedMetersPerSecond;
    double ySpeedDelivered = ySpeedCommanded * Constants.kMaxSpeedMetersPerSecond;
    double rotDelivered = m_currentRotation * Constants.kMaxAngularSpeed;

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