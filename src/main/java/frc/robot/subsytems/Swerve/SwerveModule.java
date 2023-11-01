package frc.robot.subsytems.Swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;

public class SwerveModule {

    private static CANSparkMax driveSparkMAX;
    private static CANSparkMax turnSparkMAX;

    private static RelativeEncoder driveEncoder;
    private static AbsoluteEncoder turnEncoder;

    private static SparkMaxPIDController drivePIDController;
    private static SparkMaxPIDController turnPIDController;

    private int chassisAngularOffSet = 0;

    private SwerveModuleState setState = new SwerveModuleState(0, new Rotation2d());

    public SwerveModule(int speedSparkID, int angleSparkID, int chassisAngularOffSet) {

        driveSparkMAX = new CANSparkMax(speedSparkID, MotorType.kBrushless);

        driveSparkMAX.setInverted(false);
        driveSparkMAX.setIdleMode(IdleMode.kBrake);
        driveSparkMAX.setSmartCurrentLimit(15);
        driveSparkMAX.setOpenLoopRampRate(0.2);
        driveSparkMAX.burnFlash();

        turnSparkMAX = new CANSparkMax(angleSparkID, MotorType.kBrushless);
        turnSparkMAX.setIdleMode(IdleMode.kBrake);
        turnSparkMAX.setSmartCurrentLimit(15);
        turnSparkMAX.burnFlash();

        driveEncoder = driveSparkMAX.getEncoder();
        // convert to meters for positon and meters/second for velocity
        driveEncoder.setPositionConversionFactor(Constants.toMeters);
        driveEncoder.setVelocityConversionFactor(Constants.toMeters / 60);

        turnEncoder = turnSparkMAX.getAbsoluteEncoder(Type.kDutyCycle);
        turnEncoder.setInverted(true);
        // convert to radians and raidans/second
        turnEncoder.setPositionConversionFactor(Constants.toRadians);
        turnEncoder.setVelocityConversionFactor(Constants.toRadians / 60);

        drivePIDController = driveSparkMAX.getPIDController();
        drivePIDController.setFeedbackDevice(driveEncoder);

        turnPIDController = turnSparkMAX.getPIDController();
        turnPIDController.setFeedbackDevice(turnEncoder);
        turnPIDController.setPositionPIDWrappingEnabled(true);
        turnPIDController.setPositionPIDWrappingMinInput(0);
        turnPIDController.setPositionPIDWrappingMaxInput(Constants.toRadians);

        // PID values
        turnPIDController.setP(1);

        drivePIDController.setP(1);

        this.chassisAngularOffSet = chassisAngularOffSet;
        setState.angle = new Rotation2d(turnEncoder.getPosition());
        driveEncoder.setPosition(0);
    }

    public SwerveModulePosition getPosition() {
        System.out.print("Reached: getPostion");
        return new SwerveModulePosition(driveEncoder.getPosition(),
                new Rotation2d(turnEncoder.getPosition() - chassisAngularOffSet));
    }

    public void setDesiredState(SwerveModuleState desiredState) {
        setState.speedMetersPerSecond = desiredState.speedMetersPerSecond;
        setState.angle = desiredState.angle.plus(Rotation2d.fromRadians(chassisAngularOffSet));

        setState = SwerveModuleState.optimize(setState, new Rotation2d(turnEncoder.getPosition()));

        drivePIDController.setReference(setState.speedMetersPerSecond, CANSparkMax.ControlType.kVelocity);
        turnPIDController.setReference(setState.angle.getRadians(), CANSparkMax.ControlType.kPosition);

        setState = desiredState;
    }

    public void resetDistanceEncoder() {
        driveEncoder.setPosition(0);
    }
}
