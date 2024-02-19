// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.*;
import com.ctre.phoenix6.signals.ControlModeValue;
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
  
  private TalonFX elevatorMotor;

  private DigitalInput topLimitSwitch;
  private DigitalInput bottomLimitSwitch;

  private MotionMagicVoltage elevatorMotionMagicVoltage;

  private double position;
  
  public Elevator() {
    tab = Shuffleboard.getTab("Elevator");
    elevatorHeightEntry = tab.add("Elevator Height", 0.0).getEntry();
    elevatorSpeedEntry = tab.add("Elevator Speed", 0.0).getEntry();
    topLimitSwitchEntry = tab.add("Top Limit Switch", false).getEntry();
    bottomLimitSwitchEntry = tab.add("Bottom Limit Switch", false).getEntry();
  
    elevatorMotor = new TalonFX(ElevatorConstants.elevatorMotorID);
    topLimitSwitch = new DigitalInput(ElevatorConstants.topLimitSwitchID);
    bottomLimitSwitch = new DigitalInput(ElevatorConstants.bottomLimitSwitchID);

    TalonFXConfiguration talonFXConfigs = new TalonFXConfiguration();
    
    talonFXConfigs.Feedback.SensorToMechanismRatio = ElevatorConstants.sensorToMechRatio;
    talonFXConfigs.CurrentLimits.SupplyCurrentLimit = ElevatorConstants.currentLimit;

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

    position = getHeight();
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

  public Command setHeight(double height) {
    return new Command() {
      {
        addRequirements(Elevator.this);
      }  
        @Override
        public void initialize() {
            if(height >= 0 && height <= ElevatorConstants.elevatorMaxTravel) {
                position = height;
                elevatorMotor.setControl(elevatorMotionMagicVoltage.withPosition(position));
            }
        }

        @Override
        public boolean isFinished() {
          return Math.abs(height - getHeight()) < ElevatorConstants.errorThreshold;
        }
    };
}

  public Command GetElevatorTeleop(DoubleSupplier speed){
    return new Command(){
      {
        addRequirements(Elevator.this);
      }

      double endManualTime;

      @Override
      public void initialize(){
        endManualTime = Timer.getFPGATimestamp();
      }

      @Override
      public void execute(){
        if(Math.abs(speed.getAsDouble()) > 0.05){
          setSpeed(speed.getAsDouble());
          position = getHeight();
          endManualTime = Timer.getFPGATimestamp();
        }

        else if(Timer.getFPGATimestamp() - endManualTime < 0.250){
          position = getHeight();
          setSpeed(0.0);
        }

        else if (elevatorMotor.getControlMode().getValue() != ControlModeValue.MotionMagicVoltage){
          elevatorMotor.setControl(elevatorMotionMagicVoltage.withPosition(position));
        }
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
    elevatorHeightEntry.setDouble(Units.metersToInches(getHeight()));
    elevatorSpeedEntry.setDouble(Units.metersToInches(getVelocity()));
    topLimitSwitchEntry.setBoolean(getTopLimitSwitch());
    bottomLimitSwitchEntry.setBoolean(getBottomLimitSwitch());

    //Apply hard limits. Stop the elevator if it hits the top or bottom limit switch.

    if(getTopLimitSwitch())
    {
      if(elevatorMotor.getVelocity().getValue() > 0.0)
      {
        setSpeed(0);
      }
    }

     else if(getBottomLimitSwitch()) {
      resetPosition(0.0);

      if(elevatorMotor.getVelocity().getValue() < 0.0)
      {
        setSpeed(0);
      }
    }
  }
}