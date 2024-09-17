// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.SwerveSubsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.IdleMode;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.revrobotics.RelativeEncoder;

import frc.lib.SwerveModuleConstants;

import frc.robot.Constants;

public class SwerveModules {
  
  private final CANSparkMax driveMotor;
  private final CANSparkMax turnMotor;

  private final RelativeEncoder driveEncoder;
  private final RelativeEncoder turnEncoder;

  private CANcoderConfiguration configs = new CANcoderConfiguration();
  private CANcoder absoluteEncoder;

  private final SparkPIDController drivePIDController;
  private final SparkPIDController turnPIDController;

  private double m_chassisAngularOffset = 0;
  private SwerveModuleState m_desiredState = new SwerveModuleState(0.0, new Rotation2d());

  /**
   * Constructs a MAXSwerveModule and configures the driving and turning motor,
   * encoder, and PID controller. This configuration is specific to the REV
   * MAXSwerve Module built with NEOs, SPARKS MAX, and a Through Bore
   * Encoder.
   */
  public SwerveModules(int moduleNumber, SwerveModuleConstants moduleConstants) {
    driveMotor = new CANSparkMax(moduleConstants.driveMotorID, MotorType.kBrushless);
    turnMotor = new CANSparkMax(moduleConstants.angleMotorID, MotorType.kBrushless);

    absoluteEncoder = new CANcoder(moduleConstants.cancoderID);
    configAngleEncoder();
    // Factory reset, so we get the SPARKS MAX to a known state before configuring
    // them. This is useful in case a SPARK MAX is swapped out.
    driveMotor.restoreFactoryDefaults();
    turnMotor.restoreFactoryDefaults();

    // Setup encoders and PID controllers for the driving and turning SPARKS MAX.
    driveEncoder = driveMotor.getEncoder();
    turnEncoder = turnMotor.getEncoder();
    
    drivePIDController = driveMotor.getPIDController();
    turnPIDController = turnMotor.getPIDController();
    drivePIDController.setFeedbackDevice(driveEncoder);
    turnPIDController.setFeedbackDevice(turnEncoder);

    // Apply position and velocity conversion factors for the driving encoder. The
    // native units for position and velocity are rotations and RPM, respectively,
    // but we want meters and meters per second to use with WPILib's swerve APIs.
    driveEncoder.setPositionConversionFactor(Constants.DRIVE_MOTOR_PCONVERSION);
    driveEncoder.setVelocityConversionFactor(Constants.DRIVE_MOTOR_VCONVERSION);

    // Apply position and velocity conversion factors for the turning encoder. We
    // want these in radians and radians per second to use with WPILib's swerve
    // APIs.
    turnEncoder.setPositionConversionFactor(Constants.TURN_MOTOR_PCONVERSION);
    turnEncoder.setVelocityConversionFactor(Constants.TURN_MOTOR_VCONVERSION);

    // Invert the turning encoder, since the output shaft rotates in the opposite direction of
    // the steering motor in the MAXSwerve Module.
    //turnEncoder.setInverted(false);

    // Enable PID wrap around for the turning motor. This will allow the PID
    // controller to go through 0 to get to the setpoint i.e. going from 350 degrees
    // to 10 degrees will go through 0 rather than the other direction which is a
    // longer route.
    turnPIDController.setPositionPIDWrappingEnabled(true);
    turnPIDController.setPositionPIDWrappingMinInput(0); //radians
    turnPIDController.setPositionPIDWrappingMaxInput(Constants.TURN_MOTOR_PCONVERSION); //radians

    // Set the PID gains for the driving motor. Note these are example gains, and you
    // may need to tune them for your own robot!
    drivePIDController.setP(Constants.DRIVE_P);
    drivePIDController.setI(Constants.DRIVE_I);
    drivePIDController.setD(Constants.DRIVE_D);
    drivePIDController.setFF(Constants.DRIVE_FF);
    drivePIDController.setOutputRange(-1, 1);

    // Set the PID gains for the turning motor. Note these are example gains, and you
    // may need to tune them for your own robot!
    turnPIDController.setP(Constants.ROTATE_P);
    turnPIDController.setI(Constants.ROTATE_I);
    turnPIDController.setD(Constants.ROTATE_D);
    turnPIDController.setFF(Constants.ROTATE_FF);
    turnPIDController.setOutputRange(-1, 1);

    driveMotor.setIdleMode(IdleMode.kBrake);
    turnMotor.setIdleMode(IdleMode.kBrake);
    driveMotor.setSmartCurrentLimit(40);
    turnMotor.setSmartCurrentLimit(30);

    driveMotor.setInverted(moduleConstants.driveMotorInverted);
    turnMotor.setInverted(moduleConstants.angleMotorInverted);

    // Save the SPARK MAX configurations. If a SPARK MAX browns out during
    // operation, it will maintain the above configurations.
    driveMotor.burnFlash();
    turnMotor.burnFlash();

    resetToAbsolute();
    m_chassisAngularOffset = 0;
    m_desiredState.angle = Rotation2d.fromDegrees(absoluteEncoder.getPosition().getValueAsDouble()); //new Rotation2d(turnEncoder.getPosition())
    //driveEncoder.setPosition(0);
  }

  private void configAngleEncoder() {
    configs.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
    //configs.MountPose.MagnetOffset = 0.26;
    //configs.MountPose.SensorDirection = SensorDirectionValue.Clockwise_Positive;
    absoluteEncoder.getConfigurator().apply(configs);
    absoluteEncoder.getPosition().setUpdateFrequency(100);
    absoluteEncoder.getVelocity().setUpdateFrequency(100);
  }

  public void resetToAbsolute() {
    double absolutePosition = getCanCoder().getDegrees();
    turnEncoder.setPosition(absolutePosition);
  }

  public Rotation2d getCanCoder() {
    return Rotation2d.fromDegrees(absoluteEncoder.getPosition().getValueAsDouble());
  }

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    // Apply chassis angular offset to the encoder position to get the position
    // relative to the chassis.
    return new SwerveModuleState(driveEncoder.getVelocity(),
        new Rotation2d(turnEncoder.getPosition() - m_chassisAngularOffset));
  }

  /**
   * Returns the current position of the module.
   *
   * @return The current position of the module.
   */
  public SwerveModulePosition getPosition() {
    // Apply chassis angular offset to the encoder position to get the position
    // relative to the chassis.
    return new SwerveModulePosition(
        driveEncoder.getPosition(),
        new Rotation2d(turnEncoder.getPosition() - m_chassisAngularOffset));
  }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    // Apply chassis angular offset to the desired state.
    SwerveModuleState correctedDesiredState = new SwerveModuleState();
    correctedDesiredState.speedMetersPerSecond = desiredState.speedMetersPerSecond;
    correctedDesiredState.angle = desiredState.angle.plus(Rotation2d.fromRadians(m_chassisAngularOffset));

    // Optimize the reference state to avoid spinning further than 90 degrees.
    SwerveModuleState optimizedDesiredState = SwerveModuleState.optimize(correctedDesiredState,
        new Rotation2d(turnEncoder.getPosition()));

    // Command driving and turning SPARKS MAX towards their respective setpoints.
    drivePIDController.setReference(optimizedDesiredState.speedMetersPerSecond, CANSparkMax.ControlType.kVelocity);
    turnPIDController.setReference(optimizedDesiredState.angle.getRadians(), CANSparkMax.ControlType.kPosition);

    m_desiredState = desiredState;
  }

  /** Zeroes all the SwerveModule encoders. */
  public void resetEncoders() {
    driveEncoder.setPosition(0);
  }
}