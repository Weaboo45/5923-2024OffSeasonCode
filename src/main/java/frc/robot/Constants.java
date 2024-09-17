/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import frc.lib.SwerveModuleConstants;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants.  This class should not be used for any other purpose.  All constants should be
 * declared globally (i.e. public static).  Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final Mode currentMode = Mode.REAL;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  /* Module Specific Constants */
    /* Front Left Module - Module 0 */
    public static final class Mod0 {
      public static final int driveMotorID = 1;
      public static final int angleMotorID = 5;
      public static final int canCoderID = 9;
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(0);
      public static final boolean driveMotorInverted = false;
      public static final boolean angleMotorInverted = false;
      public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
          canCoderID, angleOffset, driveMotorInverted, angleMotorInverted);
    }

    /* Front Right Module - Module 1 */
    public static final class Mod1 {
      public static final int driveMotorID = 3;
      public static final int angleMotorID = 7;
      public static final int canCoderID = 10;
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(0);
      public static final boolean driveMotorInverted = false;
      public static final boolean angleMotorInverted = false;
      public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
          canCoderID, angleOffset, driveMotorInverted, angleMotorInverted);
    }

    /* Back Left Module - Module 2 */
    public static final class Mod2 {
      public static final int driveMotorID = 4;
      public static final int angleMotorID = 6;
      public static final int canCoderID = 11;
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(0);
      public static final boolean driveMotorInverted = false;
      public static final boolean angleMotorInverted = false;
      public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
          canCoderID, angleOffset, driveMotorInverted, angleMotorInverted);
    }

    /* Back Right Module - Module 3 */
    public static final class Mod3 {
      public static final int driveMotorID = 2;
      public static final int angleMotorID = 8;
      public static final int canCoderID = 12;
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(0);
      public static final boolean driveMotorInverted = false;
      public static final boolean angleMotorInverted = false;
      public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
          canCoderID, angleOffset, driveMotorInverted, angleMotorInverted);
    }

    // Amp limits
    public static int PEAK_LIMIT = 40;
    public static int ENABLE_LIMIT = 30;

    // MEASUREMENTS
        // Drivetrain measurements
        public static double CENTER_TO_WHEEL_X = Units.inchesToMeters(28/2); // Length
        public static double CENTER_TO_WHEEL_Y = Units.inchesToMeters(28/2); // width
        public static double WHEEL_DIAMETER = Units.inchesToMeters(4);
        public static double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * Math.PI;

        //Swerve Kinematics
        public static SwerveDriveKinematics DRIVE_KIN = new SwerveDriveKinematics(
            new Translation2d(-CENTER_TO_WHEEL_X, CENTER_TO_WHEEL_Y), //mod 0
            new Translation2d(-CENTER_TO_WHEEL_X, -CENTER_TO_WHEEL_Y), //mod 1
            new Translation2d(CENTER_TO_WHEEL_X, -CENTER_TO_WHEEL_Y), // mod 2
            new Translation2d(CENTER_TO_WHEEL_X, CENTER_TO_WHEEL_Y)); // mod 3

    // Drivetrain deadbands
    public static double ROTATION_DEADBAND = .25;   //.25
    public static double STRAFING_DEADBAND = .25;  //.75
    public static double SPEED_DEADBAND = .25; //.3

    //Drivetrain maxes
    public static double DRIVETRAIN_MAX_SPEED = 4.8; // m/s
    public static double DRIVETRAIN_MAX_TURN_SPEED = Math.PI * 2; // rads/s

    public static final double kFreeSpeedRpm = 5820; //neo motor rmp free speed

    //Drive motor Conversion Factors
    public static final double DRIVE_MOTOR_GEAR_RATIO = 6.75;
    public static final double TURN_MOTOR_GEAR_RATIO = 150.0/7;

    public static final double kFreeWheelSpeedRps = (kFreeSpeedRpm * WHEEL_CIRCUMFERENCE) / DRIVE_MOTOR_GEAR_RATIO;

    public static final double DRIVE_MOTOR_PCONVERSION = WHEEL_DIAMETER * Math.PI / DRIVE_MOTOR_GEAR_RATIO;
    public static final double TURN_MOTOR_PCONVERSION = 2 * Math.PI; //360 / TURN_MOTOR_GEAR_RATIO //in radians

    public static final double DRIVE_MOTOR_VCONVERSION = DRIVE_MOTOR_PCONVERSION / 60.0;

    public static final double TURN_MOTOR_VCONVERSION = (2 * Math.PI) / 60.0; // TURN_MOTOR_GEAR_RATIO / 60.0 //in radians

    // PID CONSTANTS
        // Drivetrain PID needs tuning
        public static double DRIVE_P = 0.04; //may need tuning
        public static double DRIVE_I = 0.00;
        public static double DRIVE_D = 0.00;
        public static double DRIVE_FF = 1 / kFreeWheelSpeedRps;

        public static double ROTATE_P = 0.01; //.01
        public static double ROTATE_I = 0.00; //0.00
        public static double ROTATE_D = 0.0005; //.0005
        public static double ROTATE_FF = 0.0;

    // Autonomous drivetrain PID
    public static double AUTON_KP = 0;
    public static double AUTON_KI = 0;
    public static double AUTON_KD = 0;
    public static double AUTON_DISTANCE_SETPOINT = 0; // feet 3

    //Odometry
    public static final boolean invertGyro = false;

    public static final double driveKS = 0.1;
    public static final double driveKV = 2.3;
    public static final double driveKA = 0.3;

    public static final Translation2d MODULE_OFFSET = new Translation2d(CENTER_TO_WHEEL_X, CENTER_TO_WHEEL_Y);

    public static final class ArmConstants {
      //Motor IDs
      public static final int rightArmMotorID = 10;
      public static final int leftArmMotorID = 11;

      //Encoder ID
      public static final int kEncoderPort = 0;
    
      //Arm motor Conversion Factors
      public static final double ARM_MOTOR_GEAR_RATIO = 0.1875;
      public static final double ARM_DIAMETER = Units.inchesToMeters(35);

      public static final double ARM_MOTOR_PCONVERSION = ARM_DIAMETER * Math.PI / ARM_MOTOR_GEAR_RATIO;
      public static final double ARM_MOTOR_VCONVERSION = ARM_MOTOR_PCONVERSION / 60;

      //PID values
      public static final double kP = 0.05;
      public static final double kI = 0.00;
      public static final double kD = 0.05;
      public static final double kF = 0.00;

      //Button Setpoints in degrees
      public static final double homePoint = 40;
      public static final double apmPoint = 90;
    }

    public static final class ClimberConstants{
      public static final int rightClimberID = 15;
      public static final int leftClimberID = 16;

      public static final int rightCLimberServoID = 1;
      public static final int leftClimberServoID = 9;
    }

    public static final class ShooterConstants {
      //Shooter System IDs
      public static final int topShooterMotorID = 12; //12
      public static final int bottomShooterMotorID = 13; //13
      public static final int intakeMotorID = 14; //14

      //PID values
      public static final double topkP = 0.00015;
      public static final double topkI = 0.0000007;
      public static final double topkFF = 0.0002;
      //if needed bottom pid is the same vals
    }
}