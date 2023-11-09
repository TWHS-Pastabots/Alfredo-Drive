package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsytems.Swerve.Drivebase;
import frc.robot.auton.*;

public class Robot extends TimedRobot {

  private Drivebase drivebase;

  private DriveTest driveTest;

  private static TorqueLogiPro driver;

  private static final String kDefaultAuto = "DriveTest";
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
    double ySpeed = -driver.getRoll();
    double xSpeed = -driver.getPitch();
    double rot = 0;

    if (driver.getTrigger()) {
      rot = driver.getYaw();
    }

    if (driver.getButtonByIndex(7)) {
      drivebase.lockWheels();
    } else {
      drivebase.drive(xSpeed, ySpeed, rot, true);
    }

    drivebase.periodic();
  }

  @Override
  public void autonomousInit() {

    driveTest = new DriveTest();

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
        driveTest.execute();
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
