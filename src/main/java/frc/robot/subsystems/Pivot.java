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
import edu.wpi.first.wpilibj.DigitalInput;
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
  private GenericEntry positionError;
  private GenericEntry pivotSpeedEntry;
  private GenericEntry downLimitSwitchEntry;
  private GenericEntry upLimitSwitchEntry;

  private CANSparkMax pivotLeftMotor;
  private CANSparkMax pivotRightMotor;

  private CANcoder pivotAbsoluteEncoder;
  private ArmFeedforward pivotFeedforward;
  private ProfiledPIDController pidController;

  private DigitalInput pivotDownLimitSwitch;
  private DigitalInput pivotUpLimitSwitch;

  private Rotation2d angleSetpoint;

  private boolean isCommandRunning = false;
  private boolean isHoldingPosition = false;

  public Pivot() {
    //Shuffleboard Setup
    tab = Shuffleboard.getTab("Pivot");
    absoluteAngleEntry = tab.add("Absolute Angle", 0.0).getEntry();
    pivotSpeedEntry = tab.add("Pivot Speed", 0.0).getEntry();
    downLimitSwitchEntry = tab.add("Down Limit Switch", false).getEntry();
    upLimitSwitchEntry = tab.add("Up Limit Switch", false).getEntry();
    positionError = tab.add("Position Error", 0.0).getEntry();
    
    pivotLeftMotor = new CANSparkMax(PivotConstants.pivotLeftMotorID, MotorType.kBrushless);
    pivotRightMotor = new CANSparkMax(PivotConstants.pivotRightMotorID, MotorType.kBrushless);

    tab.add("Pivot", this);


    pivotAbsoluteEncoder = new CANcoder(PivotConstants.pivotAbsoluteEncoderID);

    CANcoderConfiguration canConfig = new CANcoderConfiguration();
    
    //Put sensor 0 to 360 degrees absolute.
    canConfig.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1;
    pivotAbsoluteEncoder.getConfigurator().apply(canConfig);

    pivotFeedforward = new ArmFeedforward(PivotConstants.pivotKS, PivotConstants.pivotKG, PivotConstants.pivotKV);

    pidController = new ProfiledPIDController(PivotConstants.pivotKP, PivotConstants.pivotKI, PivotConstants.pivotKD, new Constraints(Units.degreesToRadians(90), Units.degreesToRadians(140)));

    pidController.setTolerance(0);

    pivotDownLimitSwitch = new DigitalInput(PivotConstants.pivotDownLimitSwitchID);
    pivotUpLimitSwitch = new DigitalInput(PivotConstants.pivotUpLimitSwitchID);
    
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

  public Command setPivotAngle(Rotation2d angle){
    return new Command() {
      {
        addRequirements(Pivot.this);
      }
    
      @Override 
      public void initialize() {
        pidController.reset(getCANcoder().getRadians());
        pidController.setGoal(angle.getRadians());
        angleSetpoint = angle;
        isCommandRunning = true;
        isHoldingPosition = false;
      }

      @Override
      public void execute() {
        isCommandRunning = true;
        isHoldingPosition = false;
        double output = pidController.calculate(getCANcoder().getRadians()) + pivotFeedforward.calculate(pidController.getSetpoint().position, pidController.getSetpoint().velocity);
        pivotLeftMotor.set(output / 12.0);
        positionError.setDouble(Units.radiansToDegrees(pidController.getPositionError()));
      }

      @Override
      public boolean isFinished() {
        double error = Math.abs(getCANcoder().getDegrees() - angle.getDegrees());
        return error < 4.0;
      }

      @Override
      public void end(boolean interrupted) {
        isCommandRunning = false;
        isHoldingPosition = false;
      }
    }; 
  }
 
  public void setSpeed(double speed){
    if(getPivotUpLimitSwitch() && speed > 0){
      speed = 0;
    }

    else if(getPivotDownLimitSwitch() && speed < 0){
      speed = 0;
    }

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
        isCommandRunning = true;
        isHoldingPosition = false;
      }

      @Override
      public void execute(){
        if(Math.abs(speed.getAsDouble()) > 0.05){
          isHoldingPosition = false;
          isCommandRunning = true;

          setSpeed(speed.getAsDouble());
          angleSetpoint = getCANcoder();
          endManualTime = Timer.getFPGATimestamp();
        }

        else if(Timer.getFPGATimestamp() - endManualTime < 0.250){
          isHoldingPosition = false;
          isCommandRunning = true;

          angleSetpoint = getCANcoder();
          setSpeed(0.0);
        }

        else{
          isCommandRunning = false;
        }
      }

      @Override
      public void end(boolean interrupted){
        setSpeed(0.0);
        isCommandRunning = false;
        isHoldingPosition = false;
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

  public boolean getPivotUpLimitSwitch(){
    return pivotUpLimitSwitch.get();
  }

  public boolean getPivotDownLimitSwitch(){
    return pivotDownLimitSwitch.get();
  }

  @Override
  public void periodic() {
    absoluteAngleEntry.setDouble(getCANcoder().getDegrees());
    pivotSpeedEntry.setDouble(getPivotSpeed().getDegrees());
    downLimitSwitchEntry.setBoolean(pivotDownLimitSwitch.get());
    upLimitSwitchEntry.setBoolean(pivotUpLimitSwitch.get());

    if(!isHoldingPosition && !isCommandRunning) {
      pidController.reset(getCANcoder().getRadians());
      pidController.setGoal(angleSetpoint.getRadians());
      isHoldingPosition = true;
    }

    if(!isCommandRunning){
      double output = pivotFeedforward.calculate(pidController.getSetpoint().position, pidController.getSetpoint().velocity);
      output += Math.abs(getCANcoder().minus(angleSetpoint).getDegrees()) < 0.5? pidController.calculate(getCANcoder().getRadians()) : 0;
      pivotLeftMotor.set(output / 12.0);
    }

    //Limit Switch Hard Stops
    if(getPivotUpLimitSwitch() && getPivotSpeed().getDegrees() > 0.0)
    {
      setSpeed(0.0);
    }

    else if(getPivotDownLimitSwitch() && getPivotSpeed().getDegrees() < 0.0){
      setSpeed(0.0);
    }
  }
}