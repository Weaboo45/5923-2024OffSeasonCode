package frc.robot.commands.manual.JoyStickCommands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.YeetCannonPID;

public class JoystickArmCommand extends Command{
    private YeetCannonPID subsystem;
    private ClimberSubsystem climbSub;
    private Supplier<Boolean> intakeButtonForward, armUp, armDown, climbUp, climbDown;

    public JoystickArmCommand(YeetCannonPID subsystem, ClimberSubsystem climbSub,
     Supplier<Boolean> intakeButtonForward,
     Supplier<Boolean> armUp, Supplier<Boolean> armDown,
     Supplier<Boolean> climbUp, Supplier<Boolean> climbDown){
        addRequirements(subsystem, climbSub);
        this.climbSub = climbSub;
        this.subsystem = subsystem;

        this.climbUp = climbUp;
        this.climbDown = climbDown;

        this.intakeButtonForward = intakeButtonForward;

        this.armUp = armUp;
        this.armDown = armDown;
    }

    // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
