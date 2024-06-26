// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Constants.PivotConstants;

public class Pivot extends SubsystemBase {
  /** Creates a new Pivot. */
  
  //Shuffleboard Data
  private ShuffleboardTab tab; 
  private GenericEntry absoluteAngleEntry;
  private GenericEntry targetAngle;
  private GenericEntry pivotSpeedEntry;

  private GenericEntry angleEntry;

  private CANSparkMax pivotLeftMotor;
  private CANSparkMax pivotRightMotor;

  private CANcoder pivotAbsoluteEncoder;
  private ArmFeedforward pivotFeedforward;
  private ProfiledPIDController pidController;

  private Rotation2d angleSetpoint;
  private boolean isManualRunning = false;

  public Pivot() {
    //Shuffleboard Setup
    tab = Shuffleboard.getTab("Pivot");
    absoluteAngleEntry = tab.add("Absolute Angle", 0.0).getEntry();
    pivotSpeedEntry = tab.add("Pivot Speed", 0.0).getEntry();
    targetAngle = tab.add("Target Angle", 0.0).getEntry();

    angleEntry = tab.add("Set Angle", 0.0).getEntry();
    
    pivotLeftMotor = new CANSparkMax(PivotConstants.pivotLeftMotorID, MotorType.kBrushless);
    pivotRightMotor = new CANSparkMax(PivotConstants.pivotRightMotorID, MotorType.kBrushless);

    tab.add("Pivot", this);

    pivotAbsoluteEncoder = new CANcoder(PivotConstants.pivotAbsoluteEncoderID);

    CANcoderConfiguration canConfig = new CANcoderConfiguration();
    
    //Put sensor 0 to 360 degrees absolute.
    canConfig.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1;
    pivotAbsoluteEncoder.getConfigurator().apply(canConfig);

    pivotFeedforward = new ArmFeedforward(PivotConstants.pivotKS, PivotConstants.pivotKG, PivotConstants.pivotKV);

    pidController = new ProfiledPIDController(PivotConstants.pivotKP, PivotConstants.pivotKI, PivotConstants.pivotKD, new Constraints(PivotConstants.pivotVelocity, PivotConstants.pivotAcceleration));

    pidController.setTolerance(0);

    pidController.setIZone(Units.degreesToRadians(3.5)); //Applies I only when the error is within 3.5 degrees.
    
    pivotLeftMotor.restoreFactoryDefaults();
    pivotRightMotor.restoreFactoryDefaults();

    pivotLeftMotor.setInverted(false);
    pivotRightMotor.setInverted(true);

    pivotRightMotor.follow(pivotLeftMotor, true);

    pivotLeftMotor.setSmartCurrentLimit(PivotConstants.pivotCurrentLimit);
    pivotRightMotor.setSmartCurrentLimit(PivotConstants.pivotCurrentLimit);

    //Set motor to brake mode
    pivotLeftMotor.setIdleMode(IdleMode.kBrake);
    pivotRightMotor.setIdleMode(IdleMode.kBrake);

    pivotLeftMotor.burnFlash();
    pivotRightMotor.burnFlash();

    angleSetpoint = getCANcoder();
  }

  public void setPivotAngle(Rotation2d angle){
    angleSetpoint = Rotation2d.fromRadians(angle.getRadians());
    pidController.setGoal(angleSetpoint.getRadians());
    isManualRunning = false;
  }

  public boolean reachedSetpoint(){
    return Math.abs(getCANcoder().minus(angleSetpoint).getDegrees()) < 1.5;
  }

  public double angleErrorDegrees(){
    return Math.abs(getCANcoder().minus(angleSetpoint).getDegrees());
  }

  public double getAngleSetpoint(){
    return angleSetpoint.getDegrees();
  }

  public void setSpeed(double speed){
    isManualRunning = true;
    pivotLeftMotor.set(speed);
  }

  public Command manualControl(DoubleSupplier speed){
    return new Command(){
      {
        addRequirements(Pivot.this);
      }

      double endManualTime;

      @Override
      public void initialize(){
        endManualTime = Timer.getFPGATimestamp();
      }

      @Override
      public void execute(){
        if(Math.abs(speed.getAsDouble()) > 0.05){
          isManualRunning = true;

          setSpeed(speed.getAsDouble());
          angleSetpoint = getCANcoder();
          endManualTime = Timer.getFPGATimestamp();
          pidController.reset(getCANcoder().getRadians());
          pidController.setGoal(angleSetpoint.getRadians());
        }

        else if(Timer.getFPGATimestamp() - endManualTime < 0.250){
          setSpeed(0.0);
          isManualRunning = true;
          angleSetpoint = getCANcoder();
          pidController.reset(getCANcoder().getRadians());
          pidController.setGoal(angleSetpoint.getRadians());
        }

        else{
          isManualRunning = false;
        }
      }

      @Override
      public void end(boolean interrupted){
        setSpeed(0.0);
        isManualRunning = false;
      }
    };
  }

  public Rotation2d getCANcoder(){
    return Rotation2d.fromRotations(pivotAbsoluteEncoder.getAbsolutePosition().getValue()).minus(PivotConstants.pivotAngleOffset);
  }

  //*Returns a value from -1 to 1 */
  public Rotation2d getPivotSpeed(){
    return Rotation2d.fromRotations(pivotAbsoluteEncoder.getVelocity().getValueAsDouble());
  }

  @Override
  public void periodic() {
    double output = 0.0;

    absoluteAngleEntry.setDouble(getCANcoder().getDegrees());
    pivotSpeedEntry.setDouble(getPivotSpeed().getDegrees());
    targetAngle.setDouble(angleSetpoint.getDegrees());

    /*double newAngle = angleEntry.getDouble(0.0);

    if(Math.abs(newAngle - angleSetpoint.getDegrees()) > 0.1){
      setPivotAngle(Rotation2d.fromDegrees(newAngle));
    }*/

    if(!isManualRunning){
      output = pivotFeedforward.calculate(pidController.getSetpoint().position, pidController.getSetpoint().velocity);
      
      //Add PID controller output if the error is greater than 0.25 degrees.
      if(Math.abs(angleSetpoint.minus(getCANcoder()).getDegrees()) > 0.25){
        output += pidController.calculate(getCANcoder().getRadians());
      }
      
      pivotLeftMotor.set(output / 12.0);
    } 
  }
}