// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.*;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Constants.ElevatorConstants;

public class Elevator extends SubsystemBase {
  /** Creates a new Elevator. */
  private TalonFX elevatorMotor;

  private DigitalInput topLimitSwitch;
  private DigitalInput bottomLimitSwitch;

  private MotionMagicVoltage elevatorMotionMagicVoltage;
  
  public Elevator() {
    elevatorMotor = new TalonFX(ElevatorConstants.elevatorMotorID);
    topLimitSwitch = new DigitalInput(ElevatorConstants.topLimitSwitchID);
    bottomLimitSwitch = new DigitalInput(ElevatorConstants.bottomLimitSwitchID);

    TalonFXConfiguration talonFXConfigs = new TalonFXConfiguration();
    
    talonFXConfigs.Feedback.SensorToMechanismRatio = ElevatorConstants.sensorToMechRatio;
    talonFXConfigs.CurrentLimits.SupplyCurrentLimit = ElevatorConstants.currentLimit;

    Slot0Configs slot0Configs = talonFXConfigs.Slot0;

    slot0Configs.kS = ElevatorConstants.kS;
    slot0Configs.kV = ElevatorConstants.kV;
    slot0Configs.kA = ElevatorConstants.kA;
    slot0Configs.kG = ElevatorConstants.kG;

    slot0Configs.kP = ElevatorConstants.kP;
    slot0Configs.kI = ElevatorConstants.kI;
    slot0Configs.kD = ElevatorConstants.kD;

    // set Motion Magic settings
    MotionMagicConfigs motionMagicConfigs = talonFXConfigs.MotionMagic;
    motionMagicConfigs.MotionMagicCruiseVelocity = ElevatorConstants.cruiseVelocity;
    motionMagicConfigs.MotionMagicAcceleration = ElevatorConstants.acceleration;
    motionMagicConfigs.MotionMagicJerk = ElevatorConstants.jerk;

    elevatorMotor.getConfigurator().apply(talonFXConfigs, 0.050);
    elevatorMotor.setNeutralMode(NeutralModeValue.Brake);

    elevatorMotionMagicVoltage = new MotionMagicVoltage(0);
    elevatorMotionMagicVoltage.Slot = 0;
  }

  public void setSpeed(double speed) {
    //Stops the elevator if it hits the top or bottom limit switch.
    if(getTopLimitSwitch() && speed > 0) {
      speed = 0;
    }

    else if(getBottomLimitSwitch() && speed < 0) {
      speed = 0;
    }

    //Apply soft limits. Limit speed if within soft limit range.
    if(getHeight() >= ElevatorConstants.elevatorMaxTravel - ElevatorConstants.softLimit && speed > 0) {
      speed = Math.min(speed, 0.1);
    }

    else if(getHeight() <= 0 + ElevatorConstants.softLimit && speed < 0) {
      speed = Math.max(speed, -0.1);
    }

    elevatorMotor.set(speed);
  }

  public void setHeight(double height){
    if(height >= 0 && height <= ElevatorConstants.elevatorMaxTravel) {
      elevatorMotor.setControl(elevatorMotionMagicVoltage.withPosition(height));
    }
  }

  public void stop() {
    elevatorMotor.set(0);
  }

  public boolean reachedSetpoint(){
    return Math.abs(elevatorMotor.getClosedLoopError().getValue()) < ElevatorConstants.errorThreshold;
  }

  public boolean getTopLimitSwitch() {
    return topLimitSwitch.get();
  }

  public boolean getBottomLimitSwitch() {
    return bottomLimitSwitch.get();
  }

  /**
   * @return the height of the elevator in meters
   */
  public double getHeight() {
    return elevatorMotor.getPosition().getValue();
  }

  /**
   * @return velocity in meters per second
   */
  public double getVelocity() {
    return elevatorMotor.getVelocity().getValue();
  }

  @Override
  public void periodic() {

    //Apply hard limits. Stop the elevator if it hits the top or bottom limit switch.
    if(getTopLimitSwitch() && elevatorMotor.getVelocity().getValue() > 0) {
      setSpeed(0);
    }

    else if(getBottomLimitSwitch() && elevatorMotor.getVelocity().getValue() < 0) {
      setSpeed(0);
    }
  }
}