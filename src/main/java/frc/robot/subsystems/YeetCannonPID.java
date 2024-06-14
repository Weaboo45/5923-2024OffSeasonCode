package frc.robot.subsystems;

import frc.robot.Constants;

import org.littletonrobotics.junction.AutoLog;
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
    private SparkPIDController topController, bottomController;

    //PID variables
    public double kP, kI, kD, kIz, kFF, maxRPM, setpoint;

    public YeetCannonPID(){
        //motors
        topShooterMotor = new CANSparkMax(Constants.topShooterMotorID, MotorType.kBrushless);
        bottomShooterMotor = new CANSparkMax(Constants.bottomShooterMotorID, MotorType.kBrushless);
        intakeMotor = new CANSparkMax(Constants.intakeMotorID, MotorType.kBrushed);
        
        //encoders
        topEncoder = topShooterMotor.getEncoder();
        bottomEncoder = bottomShooterMotor.getEncoder();

        //PID controllers
        topController = topShooterMotor.getPIDController();
        bottomController = bottomShooterMotor.getPIDController();

        //PID Values
        kP = 0; 
        kI = 0;
        kD = 0; 
        kIz = 0; 
        kFF = 0;
        maxRPM = 500;

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
        topController.setP(kP);
        topController.setI(kI);
        topController.setD(kD);
        topController.setOutputRange(-1, 1);

        bottomController.setP(kP);
        bottomController.setI(kI);
        bottomController.setD(kD);
        bottomController.setOutputRange(-1, 1);
      }
    
      public void resetEncoders(){
        topEncoder.setPosition(0);
        bottomEncoder.setPosition(0);
      }

      @Override
      public void periodic() {
        //read set point
        double newSetpoint = SmartDashboard.getNumber("Setpoint", 0);
        if((newSetpoint != setpoint)) { setpoint = newSetpoint;}
        
        //read PID coefficients from SmartDashboard
        double p = SmartDashboard.getNumber("P Gain", 0);
        double i = SmartDashboard.getNumber("I Gain", 0);
        double d = SmartDashboard.getNumber("D Gain", 0);
        //double iz = SmartDashboard.getNumber("I Zone", 0);
        double ff = SmartDashboard.getNumber("Feed Forward", 0);
        //double max = SmartDashboard.getNumber("Max Output", 0);
        //double min = SmartDashboard.getNumber("Min Output", 0);

        //if PID coefficients on SmartDashboard have changed, write new values to controller
        if((p != kP)) { topController.setP(p); kP = p; }
        if((i != kI)) { topController.setI(i); kI = i; }
        if((d != kD)) { topController.setD(d); kD = d; }
        if((ff != kFF)) { topController.setFF(ff); kFF = ff; }
        //if((iz != kIz)) { m_pidController.setIZone(iz); kIz = iz; }
        //if((max != kMaxOutput) || (min != kMinOutput)) { 
        //m_pidController.setOutputRange(min, max); 
        //kMinOutput = min; kMaxOutput = max; 
      //}

        if((p != kP)) { bottomController.setP(p); kP = p; }
        if((i != kI)) { bottomController.setI(i); kI = i; }
        if((d != kD)) { bottomController.setD(d); kD = d; }
        if((ff != kFF)) { bottomController.setFF(ff); kFF = ff; }


        topController.setReference(setpoint, CANSparkMax.ControlType.kVelocity);
        bottomController.setReference(setpoint, CANSparkMax.ControlType.kVelocity);
        
        SmartDashboard.putNumber("Top RPM", topEncoder.getVelocity());
        SmartDashboard.putNumber("Bottom RPM", bottomEncoder.getVelocity());

        Logger.recordOutput("Top shooter rpm", topEncoder.getVelocity());
        Logger.recordOutput("Bottom shooter rpm", bottomEncoder.getVelocity());
    }
}
