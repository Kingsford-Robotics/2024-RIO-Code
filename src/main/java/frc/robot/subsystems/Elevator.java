// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.*;
import com.ctre.phoenix6.signals.GravityTypeValue;
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

    TalonFXConfiguration elevatorConfig = new TalonFXConfiguration();
    elevatorConfig.Slot0.
      withGravityType(GravityTypeValue.Elevator_Static).
      withKP(0.1).
      withKI(0.0).
      withKD(0.0).
      withKS(0).
      withKV(0.0).
      withKA(0.0).
      withKG(0.0);

    elevatorConfig.Feedback.SensorToMechanismRatio = ElevatorConstants.sensorToMechRatio;

    elevatorMotor.getConfigurator().apply(elevatorConfig);

    elevatorMotor.setNeutralMode(NeutralModeValue.Brake);

    elevatorMotionMagicVoltage = new MotionMagicVoltage(0);

  }

  public void setSpeed(double speed) {
    //Stops the elevator if it hits the top or bottom limit switch.
    if(getTopLimitSwitch() && speed > 0) {
      speed = 0;
    }

    else if(getBottomLimitSwitch() && speed < 0) {
      speed = 0;
    }
    elevatorMotor.set(speed);
  }

  public void setHeight(double height){
    if(height >= 0 && height <= ElevatorConstants.elevatorMaxTravel) {
      elevatorMotor.setControl(elevatorMotionMagicVoltage.withPosition(height));
    }
  }

  public boolean isAtHeight(){
    return elevatorMotor.getClosedLoopError().getValue() < ElevatorConstants.errorThreshold;
  }

  public boolean getTopLimitSwitch() {
    return topLimitSwitch.get();
  }

  public boolean getBottomLimitSwitch() {
    return bottomLimitSwitch.get();
  }

  public double getHeight() {
    return elevatorMotor.getPosition().getValue();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    if(getTopLimitSwitch() && elevatorMotor.getVelocity().getValue() > 0) {
      setSpeed(0);
    }

    else if(getBottomLimitSwitch() && elevatorMotor.getVelocity().getValue() < 0) {
      setSpeed(0);
    }
  }
}