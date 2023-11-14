package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsytems.*;
import frc.robot.subsystems.Arm.ArmControlSpeed;
import frc.robot.subsystems.Arm.ArmControlState;
import frc.robot.subsystems.Arm.ArmState;
import frc.robot.subsystems.Swerve.Drivebase;
import frc.robot.subsystems.*;
import frc.robot.TorqueLogiPro;

public class Robot extends TimedRobot {

  private Drivebase drivebase;
  private Intake intake;
  private Arm arm;

  // private DriveTest driveTest;

  private static TorqueLogiPro driver;
  private static XboxController operator;

  private boolean outtake;
  private boolean cycle;
  private boolean manual;

  private static final String kDefaultAuto = "DriveTest";
  private static final String kCustomAuto = "My Auto";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  @Override
  public void robotInit() {

    drivebase = Drivebase.getInstance();
    intake = Intake.getInstance();
    arm = Arm.getInstance();

    driver = new TorqueLogiPro(0);
    operator = new XboxController(1);

    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);
  }

  @Override
  public void robotPeriodic() {

    drivebase.periodic();
    SmartDashboard.putBoolean("Arm Manual:", manual);
  }

  @Override
  public void autonomousInit() {

    // driveTest = new DriveTest();

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
        // driveTest.execute();
        break;
    }
  }

  @Override
  public void teleopInit() {
  }

  @Override
  public void teleopPeriodic() {

    /* Drive Controls */
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

    /* Arm Controls */

    // finer control when holding L1
    if (operator.getRawButton(Controller.XBOX_LB)) {
      arm.setControlSpeed(ArmControlSpeed.FINE);
    } else {
      arm.setControlSpeed(ArmControlSpeed.FULL);
    }

    // manage arm control states
    if (driver.getRawButton(1)) {
      if (arm.controlState == ArmControlState.MANUAL) {
        arm.setControlState(ArmControlState.PID);
        manual = false;
      } else {
        arm.setControlState(ArmControlState.MANUAL);
        manual = true;
      }
    }

    // manage arm PID states & update
    // the logic for whether or not the PID/manual mode actually runs is in
    // Arm.java

    if (operator.getRawButton(Controller.XBOX_X)) {
      arm.setState(ArmState.EXTENDED);
    } else if (operator.getRawButton(Controller.XBOX_Y)) {
      arm.setState(ArmState.GROUND_INTAKE);
    } else if (operator.getRawButton(Controller.XBOX_LB)) {
      arm.setState(ArmState.RETRACTED);
    } else if (operator.getRawButtonPressed(Controller.XBOX_RB)) {
      if (cycle) {
        arm.setState(ArmState.MID);
        cycle = false;
      } else {
        arm.setState(ArmState.LOW);
        cycle = true;
      }
    }

    arm.update(operator.getRawAxis(Controller.PS_AXIS_RIGHT_Y) * .5,
        operator.getRawAxis(Controller.PS_AXIS_LEFT_Y) * .5);

    if (arm.state != ArmState.RETRACTED) {
      outtake = operator.getRawButton(Controller.XBOX_A);
    } else {
      outtake = false;
    }
    boolean intakeButton = operator.getRawButton(Controller.XBOX_B);
    intake.update(outtake, intakeButton);

    /* Drive Controls */

    // slow driving while holding left bumper, fast while holding right bumper
    arm.update(0, 0);

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
