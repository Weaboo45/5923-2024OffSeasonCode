package frc.robot.subsystems;

import frc.robot.Constants.ShooterConstants;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

//import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class YeetCannonPID extends SubsystemBase {
  //private final ShuffleboardTab tab = Shuffleboard.getTab("Competition Robot");
    //shooter motors
    private CANSparkMax topShooterMotor;
    private CANSparkMax bottomShooterMotor;

    //intake motor
    private CANSparkMax intakeMotor;

    //relative encoders
    private RelativeEncoder topEncoder, bottomEncoder;

    //shooter PID controllers
    private SparkPIDController topController, bottomController;

    //PID variables
    public double maxRPM, setpoint = 0;

    //private final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(
      //Constants.ShooterConstants.driveKS, Constants.ShooterConstants.driveKV, Constants.ShooterConstants.driveKA);

    public YeetCannonPID(){
        //motors
        topShooterMotor = new CANSparkMax(ShooterConstants.topShooterMotorID, MotorType.kBrushless);
        bottomShooterMotor = new CANSparkMax(ShooterConstants.bottomShooterMotorID, MotorType.kBrushless);
        intakeMotor = new CANSparkMax(ShooterConstants.intakeMotorID, MotorType.kBrushed);
        
        //encoders
        topEncoder = topShooterMotor.getEncoder();
        bottomEncoder = bottomShooterMotor.getEncoder();

        //PID controllers
        topController = topShooterMotor.getPIDController();
        bottomController = bottomShooterMotor.getPIDController();

        //Max RPM
        maxRPM = 4000;

        SmartDashboard.putNumber("Setpoint", setpoint);

        configBottorShooterMotor();
        configTopShooterMotor();
        configIntakeMotor();
        resetEncoders();
        configControllers();
    }

    private void configTopShooterMotor() {
        topShooterMotor.restoreFactoryDefaults();
        topShooterMotor.setSmartCurrentLimit(30);
        topShooterMotor.setIdleMode(IdleMode.kCoast);
        topShooterMotor.enableVoltageCompensation(12);
        topShooterMotor.burnFlash();
      }
    
      private void configBottorShooterMotor() {
        bottomShooterMotor.restoreFactoryDefaults();
        bottomShooterMotor.setSmartCurrentLimit(30);
        bottomShooterMotor.setIdleMode(IdleMode.kCoast);
        bottomShooterMotor.enableVoltageCompensation(12);
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

        bottomController.setP(ShooterConstants.bottomkP);
        bottomController.setI(ShooterConstants.bottomkI);
        bottomController.setFF(ShooterConstants.bottomkFF);
        bottomController.setOutputRange(-1, 1);
      }
    
      public void resetEncoders(){
        topEncoder.setPosition(0);
        bottomEncoder.setPosition(0);
      }

      public double getError(){
        double err = setpoint - -topEncoder.getVelocity();
        return err;
      }

      @Override
      public void periodic() {
        //read set point
        double newSetpoint = SmartDashboard.getNumber("Setpoint", 0);
        if((newSetpoint != setpoint)&& newSetpoint <= maxRPM) { setpoint = newSetpoint;}

        //topController.setReference(-setpoint, CANSparkMax.ControlType.kVelocity);
        topController.setReference(-setpoint, ControlType.kVelocity, 0, ShooterConstants.topkFF);
        bottomController.setReference(-setpoint, ControlType.kVelocity, 0, ShooterConstants.bottomkFF);

        SmartDashboard.putNumber("Top RPM", -topEncoder.getVelocity());
        SmartDashboard.putNumber("Bottom RPM", -bottomEncoder.getVelocity());

        SmartDashboard.putNumber("RPM Error", getError());
        Logger.recordOutput("RPM Error", getError());

        Logger.recordOutput("Top shooter rpm", -topEncoder.getVelocity());
        Logger.recordOutput("Bottom shooter rpm", bottomEncoder.getVelocity());
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
}
