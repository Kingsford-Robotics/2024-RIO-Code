// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.*;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Constants.ElevatorConstants;

public class Elevator extends SubsystemBase {
  /** Creates a new Elevator. */
  
  //Shuffleboard Setup
  ShuffleboardTab tab;
  GenericEntry elevatorHeightEntry;
  GenericEntry elevatorSpeedEntry;
  GenericEntry topLimitSwitchEntry;
  GenericEntry bottomLimitSwitchEntry;
  GenericEntry homeLimitSwitchEntry;

  GenericEntry setElevatorHeighEntry;
  
  private TalonFX elevatorMotor;

  private DigitalInput topLimitSwitch;
  private DigitalInput bottomLimitSwitch;
  private DigitalInput homeLimitSwitch;

  private MotionMagicVoltage elevatorMotionMagicVoltage;

  private double position;

  private StatusSignal<Double> elevatorMotorVelocity;
  private StatusSignal<Double> elevatorMotorHeight;

  private boolean topLimitPressed = false;
  private boolean bottomLimitPressed = false;

  //private Servo linearActuator;
  
  public Elevator() {
    tab = Shuffleboard.getTab("Elevator");
    elevatorHeightEntry = tab.add("Elevator Height", 0.0).getEntry();
    elevatorSpeedEntry = tab.add("Elevator Speed", 0.0).getEntry();
    topLimitSwitchEntry = tab.add("Top Limit Switch", false).getEntry();
    bottomLimitSwitchEntry = tab.add("Bottom Limit Switch", false).getEntry();
    homeLimitSwitchEntry = tab.add("Home Limit Switch", false).getEntry();

    setElevatorHeighEntry = tab.add("Set Elevator Height", 0.0).getEntry();
  
    elevatorMotor = new TalonFX(ElevatorConstants.elevatorMotorID);
    topLimitSwitch = new DigitalInput(ElevatorConstants.topLimitSwitchID);
    bottomLimitSwitch = new DigitalInput(ElevatorConstants.bottomLimitSwitchID);
    homeLimitSwitch = new DigitalInput(ElevatorConstants.homeLimitSwitchID);

    TalonFXConfiguration talonFXConfigs = new TalonFXConfiguration();
    
    talonFXConfigs.Feedback.SensorToMechanismRatio = ElevatorConstants.sensorToMechRatio;
    //talonFXConfigs.CurrentLimits.SupplyCurrentLimit = ElevatorConstants.currentLimit;

    Slot0Configs slot0Configs = talonFXConfigs.Slot0;

    slot0Configs.kV = ElevatorConstants.kV;
    slot0Configs.kA = ElevatorConstants.kA;
    slot0Configs.kG = ElevatorConstants.kG;

    slot0Configs.kP = ElevatorConstants.kP;
    slot0Configs.kI = ElevatorConstants.kI;

    // set Motion Magic settings
    MotionMagicConfigs motionMagicConfigs = talonFXConfigs.MotionMagic;
    motionMagicConfigs.MotionMagicCruiseVelocity = ElevatorConstants.cruiseVelocity;
    motionMagicConfigs.MotionMagicAcceleration = ElevatorConstants.acceleration;
    motionMagicConfigs.MotionMagicJerk = ElevatorConstants.jerk;

    elevatorMotor.getConfigurator().apply(talonFXConfigs, 0.050);
    elevatorMotor.setNeutralMode(NeutralModeValue.Brake);

    elevatorMotionMagicVoltage = new MotionMagicVoltage(0);
    elevatorMotionMagicVoltage.Slot = 0;

    elevatorMotor.setInverted(false);

    elevatorMotorVelocity = elevatorMotor.getVelocity();
    elevatorMotorHeight = elevatorMotor.getPosition();

    resetPosition(Units.inchesToMeters(12.2)); //Adjust this to starting height in full home.
    position = getHeight();

    /*linearActuator = new Servo(ElevatorConstants.linearActuatorID);
    linearActuator.setBoundsMicroseconds(2000, 1800, 1500, 1200, 1000);

    linearActuator.setSpeed(-1);*/
  }

  public void setSpeed(double speed) {
    //Stops the elevator if it hits the top or bottom limit switch.
    if(getTopLimitSwitch() && speed > 0.01) {
      speed = 0;
    } else if(getBottomLimitSwitch() && speed < -0.01) {
      speed = 0;
    }

    //Apply soft limits. Limit speed if within soft limit range.
    if(getHeight() >= ElevatorConstants.elevatorMaxTravel - ElevatorConstants.softLimit && speed > 0) {
      speed = Math.min(speed, 0.25);
    } else if(getHeight() <= 0 + ElevatorConstants.softLimit && speed < 0) {
      speed = Math.max(speed, -0.25);
    }

    elevatorMotor.set(speed);
  }

  public void setHeight(double height){
     if(height >= 0 && height <= ElevatorConstants.elevatorMaxTravel) {
                position = height;
                elevatorMotor.setControl(elevatorMotionMagicVoltage.withPosition(position));
    } 
  }

  public boolean reachedSetpoint(){
    return Math.abs(position - getHeight()) < ElevatorConstants.errorThreshold;
  }

  /*public void deployActuator(){
    linearActuator.setSpeed(0.30);
  }

  public void retractActuator(){
    linearActuator.setSpeed(-1.0);
  }*/

  public Command manualControl(DoubleSupplier speed) {
    return new Command() {
      {
        addRequirements(Elevator.this);
      }

      double endManualTime;
      boolean isHoldingPosition;

      @Override
      public void initialize() {
        endManualTime = Timer.getFPGATimestamp();
        position = getHeight();
        isHoldingPosition = false;
      }

      @Override
      public void execute() {
        if(Math.abs(speed.getAsDouble()) > 0.05){
          setSpeed(speed.getAsDouble());
          endManualTime = Timer.getFPGATimestamp();
          isHoldingPosition = false;
        }

        else if(Timer.getFPGATimestamp() - endManualTime < 0.250){
          setSpeed(0.0);
          isHoldingPosition = false;
        }

        else if(!isHoldingPosition){
          position = getHeight();
          elevatorMotor.setControl(elevatorMotionMagicVoltage.withPosition(position));
          isHoldingPosition = true;
        }
      }

      @Override
      public void end(boolean interrupted) {
        position = getHeight();
        elevatorMotor.setControl(elevatorMotionMagicVoltage.withPosition(position));
      }
    };
  }
  public void resetPosition(double position)
  {
    elevatorMotor.setPosition(position);
  }

  public void stop() {
    elevatorMotor.set(0);
  }

  public boolean getTopLimitSwitch() {
    return topLimitSwitch.get();
  }

  public boolean getBottomLimitSwitch() {
    return bottomLimitSwitch.get();
  }

  public boolean getHomeLimitSwitch() {
    return homeLimitSwitch.get();
  }

  /**
   * @return the height of the elevator in meters
   */
  public double getHeight() {
    elevatorMotorHeight.refresh();
    return elevatorMotorHeight.getValueAsDouble();
  }

  /**
   * @return velocity in meters per second
   */
  public double getVelocity() {
    elevatorMotorVelocity.refresh();
    return elevatorMotorVelocity.getValueAsDouble();
  }

  public void resetLimitCheck(){
    topLimitPressed = false;
    bottomLimitPressed = false;
  }

  public boolean getTopLimitPressed(){
    return topLimitPressed;
  }

  public boolean getBottomLimitPressed(){
    return bottomLimitPressed;
  }

  @Override
  public void periodic() {
    double velocity = getVelocity();
    double height = getHeight();

    double percentOutput = elevatorMotor.getDutyCycle().getValue();

    elevatorHeightEntry.setDouble(Units.metersToInches(height));
    elevatorSpeedEntry.setDouble(Units.metersToInches(velocity));
    topLimitSwitchEntry.setBoolean(getTopLimitSwitch());
    bottomLimitSwitchEntry.setBoolean(getBottomLimitSwitch());
    homeLimitSwitchEntry.setBoolean(getHomeLimitSwitch());

    // Apply hard limits. Stop the elevator if it hits the top or bottom limit switch.
    if(getTopLimitSwitch() && percentOutput > 0.01)
    {
      setSpeed(0.0);
      resetPosition(ElevatorConstants.elevatorMaxTravel);
    }

    else if(getBottomLimitSwitch() && percentOutput < -0.01)
    {  
      setSpeed(0.0);  
      resetPosition(0.0);
    }
  }
}