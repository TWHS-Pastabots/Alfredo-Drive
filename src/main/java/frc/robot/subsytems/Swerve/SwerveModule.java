package frc.robot.subsytems.Swerve;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.WPIUtilJNI;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.I2C.Port;
import frc.robot.Ports;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwerveModule {
    
    private static CANSparkMax driveSparkMAX;
    private static CANSparkMax turnSparkMAX;

    private static RelativeEncoder distanceEncoder;
    private static AbsoluteEncoder angleEncoder;

    private static PIDController drivePIDController;
    private static PIDController turnPIDController;
    private static SparkMaxPIDController testPIDController;

    private static int chassisAngularOffSet = 0;

    private static SwerveModuleState desiredState = new SwerveModuleState(0, new Rotation2d());

    public SwerveModule(int speedSparkID, int angleSparkID, int chassisAngularOffSet) {

        driveSparkMAX = new CANSparkMax(speedSparkID, MotorType.kBrushless);

        driveSparkMAX.setInverted(false);
        driveSparkMAX.setIdleMode(IdleMode.kBrake);
        driveSparkMAX.setSmartCurrentLimit(10);
        driveSparkMAX.setOpenLoopRampRate(0.2);
        driveSparkMAX.burnFlash();

        turnSparkMAX = new CANSparkMax(angleSparkID,  MotorType.kBrushless);
        turnSparkMAX.setIdleMode(IdleMode.kBrake);
        turnSparkMAX.setSmartCurrentLimit(10);
        turnSparkMAX.burnFlash();

        distanceEncoder = driveSparkMAX.getEncoder();
        //convert to meters for positon and meters/second for velocity
        distanceEncoder.setPositionConversionFactor(Constants.toMeters);
        distanceEncoder.setVelocityConversionFactor(Constants.toMeters / 60);


        angleEncoder = turnSparkMAX.getAbsoluteEncoder(Type.kDutyCycle);
        angleEncoder.setInverted(true);
        //convert to radians and raidans/second
        angleEncoder.setPositionConversionFactor(Constants.toRadians);
        angleEncoder.setVelocityConversionFactor(Constants.toRadians / 60);

        drivePIDController = new PIDController(0, 0, 0);
        drivePIDController.setSetpoint(0);

        turnPIDController = new PIDController(0, 0, 0);
        turnPIDController.setSetpoint(0);
        turnPIDController.enableContinuousInput(0, Constants.toRadians);

        testPIDController = driveSparkMAX.getPIDController();

        this.chassisAngularOffSet = chassisAngularOffSet;
        desiredState.angle = new Rotation2d(angleEncoder.getPosition());
        distanceEncoder.setPosition(0);

    }


        public SwerveModulePosition getPosition(){
            return new SwerveModulePosition(distanceEncoder.getPosition(), new Rotation2d(angleEncoder.getPosition() - chassisAngularOffSet));
        }

        public void setDesiredState(SwerveModuleState desiredState){
            SwerveModuleState correctedDesiredState = new SwerveModuleState();
            correctedDesiredState.speedMetersPerSecond = desiredState.speedMetersPerSecond;
            correctedDesiredState.angle = desiredState.angle.plus(Rotation2d.fromRadians(chassisAngularOffSet));

            SwerveModuleState optimizedDesiredState = SwerveModuleState.optimize(correctedDesiredState, new Rotation2d(angleEncoder.getPosition()));

            testPIDController.setReference(chassisAngularOffSet, null);
            distanceEncoder.calculate(optimizedDesiredState.speedMetersPerSecond);
            turnPIDController.calculate(optimizedDesiredState.angle.getRadians());

            this.desiredState = desiredState;
        }

        public void resetDistanceEncoder(){
            distanceEncoder.setPosition(0);
        }
}
