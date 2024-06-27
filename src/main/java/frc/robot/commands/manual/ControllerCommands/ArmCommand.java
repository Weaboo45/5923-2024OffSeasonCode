package frc.robot.commands.manual.ControllerCommands;

import edu.wpi.first.wpilibj2.command.Command;

import java.util.function.Supplier;
import frc.robot.subsystems.YeetCannonPID;

public class ArmCommand extends Command{
    private YeetCannonPID subsystem;
    private Supplier<Boolean> intakeButtonForward, intakeButtonBackward, shooterButton;
    //private boolean forward, backward, shooterOn;

    public ArmCommand(YeetCannonPID subsystem, Supplier<Boolean> intakeButtonForward, 
    Supplier<Boolean> intakeButtonBackward, Supplier<Boolean> shooterButton){
        addRequirements(subsystem);
        this.subsystem = subsystem;
        this.intakeButtonForward = intakeButtonForward;
        this.intakeButtonBackward = intakeButtonBackward;
        this.shooterButton = shooterButton;
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
