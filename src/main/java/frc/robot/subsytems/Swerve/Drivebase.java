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
import frc.robot.Constants.DriveConstants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drivebase extends SubsystemBase {
  private static Drivebase instance;

  public static SwerveModule frontLeft;
  public static SwerveModule backLeft;
  public static SwerveModule frontRight;
  public static SwerveModule backRight;

  private static AHRS gyro;
  // Slew rate filter variables for controlling lateral acceleration
  private double currentRotation = 0.0;
  private double currentDirection = 0.0;
  private double currentSpeed = 0.0;

  private double prevTime = WPIUtilJNI.now() * 1e-6;

  // Odometry class for tracking robot pose
  SwerveDriveOdometry odometry;

  /** Creates a new DriveSubsystem. */
  public Drivebase() {

    // Swerve modules
    frontLeft = new SwerveModule(Ports.leftSpeed1, Ports.leftAngle1, DriveConstants.kFrontLeftChassisAngularOffset);
    backLeft = new SwerveModule(Ports.leftSpeed2, Ports.leftAngle2, DriveConstants.kBackLeftChassisAngularOffset);

    frontRight = new SwerveModule(Ports.rightSpeed1, Ports.rightAngle1, DriveConstants.kFrontRightChassisAngularOffset);
    backRight = new SwerveModule(Ports.rightSpeed2, Ports.rightAngle2, DriveConstants.kBackRightChassisAngularOffset);

    // gyro
    gyro = new AHRS(SPI.Port.kMXP);

    gyro.calibrate();
    gyro.zeroYaw();

    odometry = new SwerveDriveOdometry(
        DriveConstants.kDriveKinematics,
        Rotation2d.fromDegrees(gyro.getAngle()), new SwerveModulePosition[] {
            frontLeft.getPosition(),
            backLeft.getPosition(),
            frontRight.getPosition(),
            backRight.getPosition()
        });
    System.out.print("Reached: end of constructor");
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Gyro Angle:", Math.toRadians(gyro.getAngle()));
    // Update the odometry in the periodic block
    odometry.update(Rotation2d.fromDegrees(gyro.getAngle()),
        new SwerveModulePosition[] { frontLeft.getPosition(), backLeft.getPosition(), frontRight.getPosition(),
            backRight.getPosition() });
  }

  // Returns the currently-estimated pose of the robot
  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  // Resets the odometry to the specified pose
  public void resetOdometry(Pose2d pose) {
    odometry.resetPosition(
        Rotation2d.fromDegrees(gyro.getAngle()),
        new SwerveModulePosition[] { frontLeft.getPosition(), backLeft.getPosition(), frontRight.getPosition(),
            backRight.getPosition() },
        pose);
  }

  public void drive(double forward, double side, double rot, boolean fieldRelative) {

    double xSpeedCommanded;
    double ySpeedCommanded;

    xSpeedCommanded = forward;
    ySpeedCommanded = side;
    currentRotation = rot;

    // Convert the commanded speeds into the correct units for the drivetrain
    double xSpeedDelivered = xSpeedCommanded * DriveConstants.kMaxSpeedMetersPerSecond;
    double ySpeedDelivered = ySpeedCommanded * DriveConstants.kMaxSpeedMetersPerSecond;
    double rotDelivered = currentRotation * DriveConstants.kMaxAngularSpeed;

    var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered,
                Rotation2d.fromDegrees(-gyro.getAngle()))
            : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered));

    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);

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
        desiredStates, DriveConstants.kMaxSpeedMetersPerSecond);
    frontLeft.setDesiredState(desiredStates[0]);
    backLeft.setDesiredState(desiredStates[2]);
    frontRight.setDesiredState(desiredStates[1]);
    backRight.setDesiredState(desiredStates[3]);
  }

  // sets drive encoders to 0
  public void resetEncoders() {
    frontLeft.resetEncoders();
    backLeft.resetEncoders();
    frontRight.resetEncoders();
    backRight.resetEncoders();
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    gyro.reset();
  }

  // Returns the heading of the robot(=180 to 180)
  public double getHeading() {
    return Rotation2d.fromDegrees(gyro.getAngle()).getDegrees();
  }

  // Returns the turn rate of the robot
  public double getTurnRate() {
    return gyro.getRate() * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  }

  public static Drivebase getInstance() {
    if (instance == null) {
      instance = new Drivebase();
    }
    return instance;
  }
}