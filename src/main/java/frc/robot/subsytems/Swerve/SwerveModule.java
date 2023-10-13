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

    private static RelativeEncoder distanceEncoder;
    private static AbsoluteEncoder angleEncoder;

    private static SparkMaxPIDController drivePIDController;
    private static SparkMaxPIDController turnPIDController;

    private int chassisAngularOffSet = 0;

    private SwerveModuleState setState = new SwerveModuleState(0, new Rotation2d());

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

        drivePIDController = driveSparkMAX.getPIDController();
        drivePIDController.setFeedbackDevice(distanceEncoder);

        turnPIDController = driveSparkMAX.getPIDController();
        turnPIDController.setFeedbackDevice(angleEncoder);
        turnPIDController.setPositionPIDWrappingEnabled(true);
        turnPIDController.setPositionPIDWrappingMinInput(0);
        turnPIDController.setPositionPIDWrappingMaxInput(Constants.toRadians);

        //PID values
        turnPIDController.setP(0);
        turnPIDController.setI(0);
        turnPIDController.setD(0);

        drivePIDController.setP(0);
        drivePIDController.setI(0);
        drivePIDController.setD(0);

        this.chassisAngularOffSet = chassisAngularOffSet;
        setState.angle = new Rotation2d(angleEncoder.getPosition());
        distanceEncoder.setPosition(0);

    }


        public SwerveModulePosition getPosition(){
            return new SwerveModulePosition(distanceEncoder.getPosition(), new Rotation2d(angleEncoder.getPosition() - chassisAngularOffSet));
        }

        public void setDesiredState(SwerveModuleState desiredState){
            setState.speedMetersPerSecond = desiredState.speedMetersPerSecond;
            setState.angle = desiredState.angle.plus(Rotation2d.fromRadians(chassisAngularOffSet));

            setState = SwerveModuleState.optimize(setState, new Rotation2d(angleEncoder.getPosition()));

            drivePIDController.setReference(setState.speedMetersPerSecond, CANSparkMax.ControlType.kVelocity);
            turnPIDController.setReference(setState.angle.getRadians(), CANSparkMax.ControlType.kPosition);

            setState = desiredState;
        }

        public void resetDistanceEncoder(){
            distanceEncoder.setPosition(0);
        }
}
