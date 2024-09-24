package frc.robot.commands.autoCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.PIDSubsystems.*;
import java.util.function.Supplier;

public class PIDButtons extends Command{
    private Supplier<Boolean> ampButton, shootButton;
    private ArmSubsystem armSub;
    private YeetCannonPID shootSub;
    public PIDButtons(ArmSubsystem armSub, YeetCannonPID shootSub,
     Supplier<Boolean> ampButton, Supplier<Boolean> shootButton){
        addRequirements(armSub, shootSub);
        this.armSub = armSub;
        this.shootSub = shootSub;
        this.ampButton = ampButton;
        this.shootButton = shootButton;
    }

    @Override
    public void initialize(){
        if(ampButton.get() && !shootButton.get()){
            armSub.setSetpoint(90.0);
        }
        if(shootButton.get() && !ampButton.get()){
            armSub.setSetpoint(60.0);
            shootSub.setSetpoint(20.0);
        }
        if(!shootButton.get() && !ampButton.get()) {
            armSub.setSetpoint(45.0);
            shootSub.setSetpoint(0.0);
        }
    }

    @Override
    public void execute(){

    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
