package frc.robot.auton;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.*;
import frc.robot.subsystems.Arm.*;

public class ArmCommand extends CommandBase {
    private Arm arm;
    private Intake intake;
    
    private boolean ended = false;
    private double endTime = 2;
    private double time = 2;

    public ArmCommand(){
    }

    @Override
    public void initialize() {
        arm = Arm.getInstance();
        intake = Intake.getInstance();

        intake = Intake.getInstance();
        time = Timer.getFPGATimestamp();
        endTime += time;
    }

    @Override
    public void execute() {
        arm.setState(ArmState.SHOOT);

        if (arm.hasReachedTargetPose()) {
          intake.update(false, true);

          time = Timer.getFPGATimestamp();
          if (time >= endTime) {
              ended = true;
              intake.update(false, false);
          }
        }

    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return ended;
    }
}
