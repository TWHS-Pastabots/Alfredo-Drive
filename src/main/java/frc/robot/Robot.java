package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsytems.Swerve.Drivebase;

public class Robot extends TimedRobot {

  private Drivebase drivebase;

  private static PS4Controller driver;

  private static CANSparkMax leftFrontTurnSparkMax;
  private static CANSparkMax leftFrontDriveSparkMax;
  private static CANSparkMax leftBackTurnSparkMax;
  private static CANSparkMax leftBackDriveSparkMax;

  private static CANSparkMax rightFrontTurnSparkMax;
  private static CANSparkMax rightFrontDriveSparkMax;
  private static CANSparkMax rightBackTurnSparkMax;
  private static CANSparkMax rightBackDriveSparkMax;

  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  @Override
  public void robotInit() {

    // drivebase = Drivebase.getInstance();

    driver = new PS4Controller(0);
    leftFrontTurnSparkMax = new CANSparkMax(Ports.leftAngle1,
        MotorType.kBrushless);
    leftFrontDriveSparkMax = new CANSparkMax(Ports.leftSpeed1,
        MotorType.kBrushless);

    leftBackTurnSparkMax = new CANSparkMax(Ports.leftAngle2,
        MotorType.kBrushless);
    leftBackDriveSparkMax = new CANSparkMax(Ports.leftSpeed2,
        MotorType.kBrushless);

    rightFrontTurnSparkMax = new CANSparkMax(Ports.rightAngle1,
        MotorType.kBrushless);
    rightFrontDriveSparkMax = new CANSparkMax(Ports.rightSpeed1,
        MotorType.kBrushless);

    rightBackTurnSparkMax = new CANSparkMax(Ports.rightAngle2,
        MotorType.kBrushless);
    rightBackDriveSparkMax = new CANSparkMax(Ports.rightSpeed2,
        MotorType.kBrushless);

    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);
  }

  @Override
  public void robotPeriodic() {

    // double xSpeed = driver.getRawAxis(Controller.PS_AXIS_LEFT_X);
    // double ySpeed = driver.getRawAxis(Controller.PS_AXIS_LEFT_Y);

    // double slant =
    // Math.sqrt(Math.pow(driver.getRawAxis(Controller.PS_AXIS_RIGHT_X), 2) +
    // Math.pow(driver.getRawAxis(Controller.PS_AXIS_RIGHT_Y), 2));

    // double rot = slant;

    leftFrontDriveSparkMax.set(.5);
    leftFrontTurnSparkMax.set(.5);

    leftBackDriveSparkMax.set(.5);
    leftBackTurnSparkMax.set(.5);

    rightFrontDriveSparkMax.set(.5);
    rightFrontTurnSparkMax.set(.5);

    rightBackDriveSparkMax.set(.5);
    rightBackTurnSparkMax.set(.5);

    // drivebase.drive(xSpeed, ySpeed, rot, true, true);
  }

  @Override
  public void autonomousInit() {
    m_autoSelected = m_chooser.getSelected();
    m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    System.out.println("Auto selected: " + m_autoSelected);
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    switch (m_autoSelected) {
      case kCustomAuto:
        // Put custom auto code here
        break;
      case kDefaultAuto:
      default:
        // Put default auto code here
        break;
    }
  }

  @Override
  public void teleopInit() {
  }

  @Override
  public void teleopPeriodic() {

    // drivebase.drive(driver.getRawAxis(Controller.PS_AXIS_LEFT_X),
    // driver.getRawAxis(Controller.PS_AXIS_LEFT_Y),
    // driver.getRawAxis(Controller.PS_AXIS_RIGHT_X), true, false);

  }

  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
  }

  @Override
  public void testInit() {
  }

  @Override
  public void testPeriodic() {
  }

  @Override
  public void simulationInit() {
  }

  @Override
  public void simulationPeriodic() {
  }
}
