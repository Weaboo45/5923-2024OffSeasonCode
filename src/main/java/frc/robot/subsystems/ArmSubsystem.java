package frc.robot.subsystems;

//import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import frc.robot.Constants.ArmConstants;

public class ArmSubsystem extends SubsystemBase {
  private final CANSparkMax rightArmMotor;
  private final CANSparkMax leftArmMotor;

  private final DutyCycleEncoder absoluteEncoder = new DutyCycleEncoder(ArmConstants.kEncoderPort);
  private final RelativeEncoder rightArmEncoder, leftArmEncoder;

  private final SparkPIDController rightArmController, leftArmController;

  private final DigitalInput zeroSwitch = new DigitalInput(1), ninetySwitch = new DigitalInput(9);

  public double kP = 0, kI = 0, kD = 0, kFF = 0, setPoint = 0, homePoint = 0;
  
  public ArmSubsystem() {
    rightArmMotor = new CANSparkMax(ArmConstants.rightArmMotorID, MotorType.kBrushless);
    leftArmMotor = new CANSparkMax(ArmConstants.leftArmMotorID, MotorType.kBrushless);

    rightArmEncoder = rightArmMotor.getEncoder();
    leftArmEncoder = leftArmMotor.getEncoder();

    rightArmController = rightArmMotor.getPIDController();
    leftArmController = leftArmMotor.getPIDController();

    configLeftArmMotor();
    configRightArmMotor();
    resetArmEncoders();

    leftArmMotor.follow(rightArmMotor, true);

    // PID values
    SmartDashboard.putNumber("Arm P", kP);
    SmartDashboard.putNumber("Arm I", kI);
    SmartDashboard.putNumber("Arm D", kD);
    SmartDashboard.putNumber("Arm FF", kFF);

    // Setpoint Val
    SmartDashboard.putNumber("Arm Setpoint", setPoint);
  }

  @Override
  public void periodic() {
    resetArmEncoders();

    SmartDashboard.putNumber("Shooter Angle", getRevEncoder());
    SmartDashboard.putNumber("Arm Angle", getArmAngle());

    SmartDashboard.putBoolean("Home Switch", zeroSwitch.get());
    SmartDashboard.putBoolean("90 Switch", ninetySwitch.get());

    //Changable PID vals from dashboard
    double newSet = SmartDashboard.getNumber("Arm Setpoint", 0);
      if(newSet != setPoint){  setPoint = newSet; }
    double newP = SmartDashboard.getNumber("Arm P", 0);
      if(newP != kP){  kP = newP; rightArmController.setP(kP); }
    double newI = SmartDashboard.getNumber("Arm I", 0);
      if(newI != kI){  kI = newI; rightArmController.setI(kI); }
    double newD = SmartDashboard.getNumber("Arm D", 0);
      if(newD != kD){  kD = newD; rightArmController.setD(kD); }
    double newFF = SmartDashboard.getNumber("Arm FF", 0);
      if(newFF != kFF){  kD = newD; rightArmController.setFF(kFF); }


    if(zeroSwitch.get() && (rightArmMotor.getAppliedOutput() <= 0)){
      rightArmMotor.set(0);
      leftArmMotor.set(0);
    } else {
      if(ninetySwitch.get() && (rightArmMotor.getAppliedOutput() >= 0)){
        rightArmMotor.set(0);
        leftArmMotor.set(0);
      } else {
          rightArmController.setReference(setPoint, ControlType.kPosition, 0, kFF);
      }
    }

    Logger.recordOutput("Shooter Angle", getRevEncoder());
    Logger.recordOutput("Arm Angle", getArmAngle());
  }

  private void configRightArmMotor() {
    rightArmMotor.restoreFactoryDefaults();
    rightArmMotor.setSmartCurrentLimit(15);
    rightArmMotor.setInverted(true);
    rightArmMotor.setIdleMode(IdleMode.kBrake);
    rightArmMotor.enableVoltageCompensation(12);
    rightArmMotor.burnFlash();

    //rightArmEncoder.setPositionConversionFactor(ArmConstants.ARM_MOTOR_PCONVERSION);
    rightArmController.setOutputRange(-1, 1);
  }

  private void configLeftArmMotor() {
    leftArmMotor.restoreFactoryDefaults();
    leftArmMotor.setSmartCurrentLimit(15);
    leftArmMotor.setIdleMode(IdleMode.kBrake);
    leftArmMotor.enableVoltageCompensation(12);
    leftArmMotor.burnFlash();

    leftArmController.setOutputRange(-1, 1);
    //leftArmEncoder.setPositionConversionFactor(ArmConstants.ARM_MOTOR_PCONVERSION);
  }

  private void resetArmEncoders(){
    rightArmEncoder.setPosition(getRevEncoder());
    leftArmEncoder.setPosition(getRevEncoder());
  }

  private double getArmAngle(){
    return (rightArmEncoder.getPosition() + leftArmEncoder.getPosition()) / 2;
  }

  /** Shooter Angle */
  public double getRevEncoder(){
    return absoluteEncoder.getDistance() * 360;
  }

  public double getArmError(){
    return setPoint - rightArmEncoder.getPosition();
  }
}
