package frc.robot.subsystems.PIDSubsystems;

import frc.robot.Constants.ShooterConstants;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class YeetCannonPID extends SubsystemBase {
  //shooter motors
  private CANSparkMax topShooterMotor;
  private CANSparkMax bottomShooterMotor;

  //intake motor
  private CANSparkMax intakeMotor;

  //relative encoders
  private RelativeEncoder topEncoder, bottomEncoder;

  //shooter PID controllers
  private SparkPIDController topController;

  //PID variables
  public double maxRPM, setpoint = 0, mpsSetpoint = 0, maxMPS = 31;

  public YeetCannonPID(){
    //motors
    topShooterMotor = new CANSparkMax(ShooterConstants.topShooterMotorID, MotorType.kBrushless);
    bottomShooterMotor = new CANSparkMax(ShooterConstants.bottomShooterMotorID, MotorType.kBrushless);
    intakeMotor = new CANSparkMax(ShooterConstants.intakeMotorID, MotorType.kBrushed);
        
    //encoders
    topEncoder = topShooterMotor.getEncoder();
    bottomEncoder = bottomShooterMotor.getEncoder();

    //PID controller
    topController = topShooterMotor.getPIDController();

    //Max RPM
    maxRPM = 4000;

    //SmartDashboard.putNumber("Setpoint", setpoint);
    //SmartDashboard.putNumber("MPS Setpoint", mps);

    configBottorShooterMotor();
    configTopShooterMotor();
    configIntakeMotor();
    resetEncoders();
    configControllers();

    bottomShooterMotor.follow(topShooterMotor);
  }

  private void configTopShooterMotor() {
    topShooterMotor.restoreFactoryDefaults();
    topShooterMotor.setSmartCurrentLimit(30);
    topShooterMotor.setIdleMode(IdleMode.kCoast);
    topShooterMotor.enableVoltageCompensation(12);
    topShooterMotor.setInverted(true);
    topShooterMotor.burnFlash();
  }
    
  private void configBottorShooterMotor() {
    bottomShooterMotor.restoreFactoryDefaults();
    bottomShooterMotor.setSmartCurrentLimit(30);
    bottomShooterMotor.setIdleMode(IdleMode.kCoast);
    bottomShooterMotor.enableVoltageCompensation(12);
    bottomShooterMotor.setInverted(true);
    bottomShooterMotor.burnFlash();
  }
    
  private void configIntakeMotor() {
    intakeMotor.restoreFactoryDefaults();
    intakeMotor.setSmartCurrentLimit(30);
    intakeMotor.setIdleMode(IdleMode.kBrake);
    intakeMotor.enableVoltageCompensation(12);
    intakeMotor.burnFlash();
  }

  private void configControllers(){
    topController.setP(ShooterConstants.topkP);
    topController.setI(ShooterConstants.topkI);
    topController.setFF(ShooterConstants.topkFF);
    topController.setOutputRange(-1, 1);
  }
    
  public void resetEncoders(){
    topEncoder.setPosition(0);
    bottomEncoder.setPosition(0);
  }

  public double getTopError(){
    return setpoint - topEncoder.getVelocity();
  }

  public double getBottomError(){
    return setpoint - bottomEncoder.getVelocity();
  }

  public double avgRPM(){
    return (topEncoder.getVelocity() + bottomEncoder.getVelocity()) / 2;
  }

  /** returns the current wheel speed in meters / second  */
  public double getMPS(){
    return .0762 * (( 2 * Math.PI) / 60 ) * avgRPM();
    //test at 3000 RPM and result should be 23.93 m/s
  }

  public double mpsToRPM(double val){
    return (60 / ((2 * Math.PI) * .0762)) * val;
  }

  public void intakeFoward(){
    intakeMotor.set(1.0);
  }
  
  public void intakeBackward(){
    intakeMotor.set(-1.0);
  }
  
  public void intakeOff(){
    intakeMotor.set(0.0);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Shooter Setpoint MPS", getSetpoint());
    //Kept here for example
    //read set point
    //double newSetpoint = SmartDashboard.getNumber("Setpoint", 0);
    //if((newSetpoint != setpoint)&& newSetpoint <= maxRPM) { setpoint = newSetpoint;}

    /* Kept here for example 
    double newMPS = SmartDashboard.getNumber("MPS Setpoint", 0);
    if((newMPS != mpsSetpoint)&& newMPS <= maxMPS){
      mpsSetpoint = newMPS;
      setpoint= mpsToRPM(mpsSetpoint);
    }
    */

    topController.setReference(getSetpoint(), ControlType.kVelocity);
    //topController.setReference(setpoint, ControlType.kVelocity, 0, ShooterConstants.topkFF); //original

    SmartDashboard.putNumber("Top RPM", topEncoder.getVelocity());
    SmartDashboard.putNumber("MPS of Shooter", getMPS());

    Logger.recordOutput("Top RPM Error", getTopError());

    Logger.recordOutput("Shooter rpm", topEncoder.getVelocity());
  }

  /** Setpoint in meters / second */
  public void setSetpoint(double newMPS){
    if(newMPS > maxMPS){
      mpsSetpoint = mpsToRPM(maxMPS);
    } else {
      mpsSetpoint = mpsToRPM(newMPS);
    }
  }

  public double getSetpoint(){
    return setpoint;
  }
}
