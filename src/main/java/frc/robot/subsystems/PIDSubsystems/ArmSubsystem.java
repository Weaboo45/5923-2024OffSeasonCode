package frc.robot.subsystems.PIDSubsystems;

//import edu.wpi.first.math.controller.SimpleMotorFeedforward;
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

import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;

public class ArmSubsystem extends SubsystemBase {
  private final CANSparkMax rightArmMotor;
  private final CANSparkMax leftArmMotor;

  private final DutyCycleEncoder absoluteEncoder = new DutyCycleEncoder(ArmConstants.kEncoderPort);
  private final RelativeEncoder rightArmEncoder, leftArmEncoder;

  private final SparkPIDController rightArmController;

  private final DigitalInput zeroSwitch = new DigitalInput(1), ninetySwitch = new DigitalInput(9);

  public double setpoint = 45;
  
  public ArmSubsystem() {
    rightArmMotor = new CANSparkMax(ArmConstants.rightArmMotorID, MotorType.kBrushless);
    leftArmMotor = new CANSparkMax(ArmConstants.leftArmMotorID, MotorType.kBrushless);

    rightArmEncoder = rightArmMotor.getEncoder();
    leftArmEncoder = leftArmMotor.getEncoder();

    rightArmController = rightArmMotor.getPIDController();

    configLeftArmMotor();
    configRightArmMotor();
    resetArmEncoders();

    leftArmMotor.follow(rightArmMotor, true);

    // PID values
    /*Left for example purposes 
    SmartDashboard.putNumber("Arm P", kP);
    SmartDashboard.putNumber("Arm I", kI);
    SmartDashboard.putNumber("Arm D", kD);
    SmartDashboard.putNumber("Arm FF", kF);
    */

    rightArmController.setP(Constants.ArmConstants.kP);
    rightArmController.setI(Constants.ArmConstants.kI);
    rightArmController.setD(Constants.ArmConstants.kD);
    rightArmController.setFF(Constants.ArmConstants.kF); //test kFF .5 doubled output

    // Setpoint Val
    SmartDashboard.putNumber("Arm Setpoint", getSetpoint());
  }

  private void configRightArmMotor() {
    rightArmMotor.restoreFactoryDefaults();
    rightArmMotor.setSmartCurrentLimit(15);
    rightArmMotor.setInverted(true);
    rightArmMotor.setIdleMode(IdleMode.kBrake);
    rightArmMotor.enableVoltageCompensation(12); 
    rightArmMotor.burnFlash();
    rightArmController.setOutputRange(-.5, .5);
  }

  private void configLeftArmMotor() {
    leftArmMotor.restoreFactoryDefaults();
    leftArmMotor.setSmartCurrentLimit(15);
    leftArmMotor.setIdleMode(IdleMode.kBrake);
    leftArmMotor.enableVoltageCompensation(12);
    leftArmMotor.burnFlash();
  }

  private void resetArmEncoders(){
    rightArmEncoder.setPosition(getRevEncoder()); //try subtracting
    leftArmEncoder.setPosition(getRevEncoder());
  }

  @Override
  public void periodic() {
    resetArmEncoders();

    SmartDashboard.putNumber("Arm Error", getArmError());

    SmartDashboard.putNumber("Shooter Angle", getRevEncoder());
    SmartDashboard.putNumber("Arm Angle", getArmAngle());

    SmartDashboard.putBoolean("Home Switch", zeroSwitch.get());
    SmartDashboard.putBoolean("90 Switch", ninetySwitch.get());

    //Changable PID vals from dashboard
    /*Left for example purposes 
    double newSet = SmartDashboard.getNumber("Arm Setpoint", homePoint);
      if(newSet != getSetpoint()){  setSetpoint(newSet); }
      */

    if(zeroSwitch.get() && (rightArmMotor.getAppliedOutput() > 0)){ //try using softlimits on the motors
      rightArmMotor.set(0);
      leftArmMotor.set(0);
    } else {
      if(ninetySwitch.get() && (rightArmMotor.getAppliedOutput() < 0)){
        rightArmMotor.set(0);
        leftArmMotor.set(0);
      } else {
          rightArmController.setReference(getSetpoint(), ControlType.kPosition);
      }
    }

    Logger.recordOutput("Shooter Angle", getRevEncoder());
    Logger.recordOutput("Arm Angle", getArmAngle());
  }

  /** Arm Angle in degrees */
  private double getArmAngle(){
    double avg = rightArmEncoder.getPosition();
    return avg;
  }

  /** Shooter Angle in degrees */ //shooter is ~60 degrees offset from arm so when arm angle == 0, shooter angle == 60 in CAD
  public double getRevEncoder(){
    return absoluteEncoder.getDistance() * 360;
  }

  //Methods for PID
  public double getArmError(){
    return getSetpoint() - rightArmEncoder.getPosition();
  }

  /**The new arm set point in degrees */
  public void setSetpoint(double newPoint){
    setpoint = newPoint;
  }

  public double getSetpoint(){
    return setpoint;
  }
}
