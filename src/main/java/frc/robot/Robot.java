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

  private static TorqueLogiPro driver;

  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  @Override
  public void robotInit() {

    drivebase = Drivebase.getInstance();

    // driver = new PS4Controller(0);
    driver = new TorqueLogiPro(0);

    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);
  }

  @Override
  public void robotPeriodic() {

    // double xSpeed = driver.getRawAxis(Controller.PS_AXIS_LEFT_X);
    // double ySpeed = driver.getRawAxis(Controller.PS_AXIS_LEFT_Y);

    // double rot = driver.getRawAxis(Controller.PS_AXIS_RIGHT_X);

    double xSpeed = -driver.getRoll();
    double ySpeed = -driver.getPitch();
    double rot = driver.getYaw();
    if (Math.abs(rot) < .1)
      rot = 0;

    drivebase.drive(xSpeed, ySpeed, rot, true);

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
