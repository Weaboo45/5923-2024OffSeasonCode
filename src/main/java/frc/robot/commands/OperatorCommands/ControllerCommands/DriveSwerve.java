/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.OperatorCommands.ControllerCommands;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveSubsystems.SwerveDrivetrain;


public class DriveSwerve extends Command {
  /*
   * Creates a new DriveMecanum.
   */

  private SwerveDrivetrain drivetrain;
  private Supplier<Double>  y, x, z;
  private Supplier<Boolean> fieldTOrientated, resetGyro, formX, rateLim;
  boolean fieldDrive = true, onOff = false, yesX = false;
  double speed = 3.5, speedMult = 1.0;

  public DriveSwerve(SwerveDrivetrain drivetrain, Supplier<Double> yDirect, Supplier<Double> xDirect, 
  Supplier<Double> rotation, Supplier<Boolean> fieldTOrientated, Supplier<Boolean> resetGyro,
  Supplier<Boolean> formX, Supplier<Boolean> rateLim) {
    addRequirements(drivetrain);
    this.drivetrain = drivetrain;
    this.y = yDirect;
    this.x = xDirect;
    this.z = rotation;
    this.resetGyro = resetGyro;
    this.fieldTOrientated = fieldTOrientated; // toggle
    this.formX = formX;
    this.rateLim = rateLim;
  }

// Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    SmartDashboard.putNumber("Speed Multiplier", speedMult);

    if(formX.get()){
      yesX = !yesX;
    }

    if(yesX){
      drivetrain.setX();
      drivetrain.drive(0, 0, 0, fieldDrive, onOff);
    }

    if(rateLim.get()){
      onOff = !onOff;
    }

    speedMult = SmartDashboard.getNumber("Speed Multiplier", 1);

    if(resetGyro.get()){
      drivetrain.zeroHeading();
    }

    SmartDashboard.putBoolean("Field Drive", fieldDrive);
    if(fieldTOrientated.get()){
      fieldDrive = !fieldDrive;
    }

    /* Get Values, Deadband */
    double translationVal = MathUtil.applyDeadband(y.get(), Constants.SPEED_DEADBAND);
    double strafeVal = MathUtil.applyDeadband(x.get(), Constants.STRAFING_DEADBAND);
    double rotationVal = MathUtil.applyDeadband(z.get(), Constants.ROTATION_DEADBAND);

    drivetrain.drive(translationVal * speedMult, strafeVal * speedMult,
      rotationVal * 4, fieldDrive, onOff);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //drivetrain.stopModules();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}