/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;

import java.util.Map;

import com.kauailabs.navx.frc.AHRS;
//import com.pathplanner.lib.auto.AutoBuilder;
//import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.SerialPort.Port;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
//import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

//import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.OperatorCommands.ControllerCommands.*;
import frc.robot.commands.OperatorCommands.JoyStickCommands.*;
import frc.robot.commands.autoCommands.PIDButtons;
import frc.robot.subsystems.PIDSubsystems.*;
import frc.robot.subsystems.SwerveSubsystems.SwerveDrivetrain;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  //ParallelCommandGroup ampScore = new ParallelCommandGroup(
    //new AutoIntake(scoreSub), new AutoShoot(scoreSub));
  
  /*
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  //private final SendableChooser<Command> m_chooser;
    //private final ScoreAmp scoreAmp = new ScoreAmp(scoreSub);

  public RobotContainer() {
    //m_chooser = AutoBuilder.buildAutoChooser();

    configureInitialDefaultCommands();
    configureBindings();
    configureShuffleboardData();
    configureSmartDashboard();

    //NamedCommands.registerCommand("ScoreAmp", ampScore);

    //SmartDashboard.putData("Auto Mode", m_chooser);
  }

  // The robot's subsystems and commands are defined here...
  /// SHUFFLEBOARD TAB ///
  private final ShuffleboardTab m_tab = Shuffleboard.getTab("Competition Robot");

  /// SUBSYSTEMS ///
  public static final SwerveDrivetrain drivetrain = new SwerveDrivetrain();
  public static final ArmSubsystem armSub = new ArmSubsystem();
  public static final YeetCannonPID yeetSub = new YeetCannonPID();

  /// OI DEVICES / HARDWARE ///
  private final XboxController xbox = new XboxController(0);
  private final Joystick stick = new Joystick(1);
  private static final AHRS ahrs = new AHRS(Port.kMXP);

  CommandXboxController commandXbox = new CommandXboxController(0);

  //Joystick Button Commands
  JoystickButton shooterOn = new JoystickButton(stick, 1);

  //intake buttons
  JoystickButton intakeFoward = new JoystickButton(stick, 2);
  JoystickButton intakeBackward = new JoystickButton(stick, 3);

  /// COMMANDS ///
  // Xbox controls
  private final DriveSwerve drivetrainXbox = new DriveSwerve(drivetrain, ()-> -xbox.getLeftY(), ()-> xbox.getLeftX(), ()-> -xbox.getRightX(),
   ()-> xbox.getRightBumperReleased(), ()-> xbox.getLeftBumper(), ()-> xbox.getXButtonPressed(), ()-> xbox.getBButtonPressed()); 
  //    RB toggles field orintation         LB resets heading             forms X with wheels     enables the acceleration limit

  // Joystick Controls
  private final DriveJoystickSwerve driveJoystick = new DriveJoystickSwerve(drivetrain, () -> stick.getY(), () -> stick.getX(), () -> stick.getTwist(),
   () -> stick.getRawButton(7), () -> stick.getRawButton(8), () -> stick.getThrottle());

  private final PIDButtons buttons = new PIDButtons(armSub, yeetSub, ()-> stick.getRawButton(8), ()-> stick.getRawButton(7));

  /// SHUFFLEBOARD METHODS ///
  /**
   * Use this command to define {@link Shuffleboard} buttons using a
   * {@link ShuffleboardTab} and its add() function. You can put already defined
   * Commands,
   */
  private void configureShuffleboardData() {
    Shuffleboard.selectTab(m_tab.getTitle());

    ShuffleboardLayout drivingStyleLayout = m_tab.getLayout("driving styles", BuiltInLayouts.kList)
    .withPosition(0, 0).withSize(2, 2)
    .withProperties(Map.of("label position", "BOTTOM"));

    drivingStyleLayout.add("Xbox Drive",
    new InstantCommand(() -> drivetrain.setDefaultCommand(drivetrainXbox), drivetrain));

    drivingStyleLayout.add("Joystick Drive",
    new InstantCommand(() -> drivetrain.setDefaultCommand(driveJoystick), drivetrain));
 
    ShuffleboardLayout gyroSensor = m_tab.getLayout("NavX", BuiltInLayouts.kGrid)
    .withPosition(2, 0).withSize(1, 3)
    .withProperties(Map.of("label position", "BOTTOM"));

    gyroSensor.addNumber("Gyro", ()-> ahrs.getYaw()).withWidget(BuiltInWidgets.kGyro);

    gyroSensor.add("Reset",
    new InstantCommand(()-> drivetrain.zeroHeading()));

    ShuffleboardLayout controllerLayout = m_tab.getLayout("Controller Vals", BuiltInLayouts.kGrid)
    .withPosition(4, 0).withSize(2, 6)
    .withProperties(Map.of("label position", "BOTTOM"));
    controllerLayout.addNumber("left y", () -> -xbox.getLeftY())
    .withPosition(0, 0).withSize(2, 1).withWidget(BuiltInWidgets.kNumberBar);
    controllerLayout.addNumber("left x", () -> xbox.getLeftX())
    .withPosition(0, 1).withSize(2, 1).withWidget(BuiltInWidgets.kNumberBar);
    controllerLayout.addNumber("left trigger", () -> xbox.getLeftTriggerAxis())
    .withPosition(0, 2).withSize(2, 1).withWidget(BuiltInWidgets.kNumberBar);
    controllerLayout.addNumber("right y", () -> -xbox.getRightY())
    .withPosition(2, 0).withSize(2, 1).withWidget(BuiltInWidgets.kNumberBar);
    controllerLayout.addNumber("right x", () -> xbox.getRightX())
    .withPosition(2, 1).withSize(2, 1).withWidget(BuiltInWidgets.kNumberBar);
    controllerLayout.addNumber("right trigger", () -> xbox.getRightTriggerAxis())
    .withPosition(2, 2).withSize(2, 1).withWidget(BuiltInWidgets.kNumberBar);
  }

  private void configureSmartDashboard(){
    //match Auto
    //m_chooser.addOption("Amp Auto", new PathPlannerAuto("AmpAuto"));
    //m_chooser.addOption("Right Side Path", new PathPlannerAuto("Right Side Path"));

    //test Autos
    //m_chooser.addOption("Square", new PathPlannerAuto("SquareAuto"));
    //m_chooser.addOption("Test Auto", new PathPlannerAuto("Test Auto"));
    //m_chooser.addOption("Intake and shooter test", new PathPlannerAuto("IntakeShoot"));
    //SmartDashboard.putData(m_chooser);
  }

  /**   
   * Use this method to define the default commands of subsystems. 
   * Default commands are ran whenever no other commands are using a specific subsystem.
   */
  private void configureInitialDefaultCommands() {
    drivetrain.setDefaultCommand(drivetrainXbox);
    armSub.setDefaultCommand(buttons);
    yeetSub.setDefaultCommand(buttons);
  }
  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureBindings() {
    //ampScoring.whileTrue( new ParallelCommandGroup( new AutoIntake(scoreSub), new AutoShoot(scoreSub) ) );

    //shooter disable
    shooterOn.onTrue
    (Commands.run(
      ()-> {
        yeetSub.intakeOff();
      }, yeetSub));

      //intake buttons
    intakeFoward.whileTrue
    (Commands.run(
      ()-> {
          yeetSub.intakeFoward();
      }, yeetSub));

    intakeBackward.whileTrue
    (Commands.run(
      ()-> {
        yeetSub.intakeBackward();
      }, yeetSub));
  }

    

  /**
   * Disables all ProfiledPIDSubsystem and PIDSubsystem instances. This should be called on robot
   * disable to prevent integral windup.
   */
  public void disablePIDSubsystems() {
    //armSub.disable();
  }
  
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  //public Command getAutonomousCommand() {
    // Executes the autonomous command chosen in smart dashboard

    // Load the path you want to follow using its name in the GUI
        //PathPlannerPath path = PathPlannerPath.fromPathFile("Test Path");

        // Create a path following command using AutoBuilder. This will also trigger event markers.
        //return AutoBuilder.followPath(path);
    //return m_chooser.getSelected();
  //}

  public void displayValues() {
  SmartDashboard.putData(drivetrain);
  //SmartDashboard.putData(m_chooser);
  }
}