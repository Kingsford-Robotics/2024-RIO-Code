// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonSRXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Constants.ShooterConstants;

public class Shooter extends SubsystemBase {
  /** Creates a new Shooter. */

  ShuffleboardTab tab;
  GenericEntry speedEntry;
  GenericEntry errorEntry;
  GenericEntry atSetpointEntry;


  private TalonSRX shooterLeftMotor;
  private TalonSRX shooterRightMotor;

  public Shooter() {
    tab = Shuffleboard.getTab("Shooter");
    speedEntry = tab.add("Shooter Speed", 0).getEntry();
    errorEntry = tab.add("Shooter Error", 0).getEntry();
    atSetpointEntry = tab.add("At Setpoint", false).getEntry();

    shooterLeftMotor = new TalonSRX(ShooterConstants.shooterLeftMotorID);
    shooterRightMotor = new TalonSRX(ShooterConstants.shooterRightMotorID);

    shooterLeftMotor.configFactoryDefault();
    shooterRightMotor.configFactoryDefault();

    shooterRightMotor.setInverted(true);
    shooterRightMotor.follow(shooterLeftMotor);

    shooterLeftMotor.configSelectedFeedbackSensor(TalonSRXFeedbackDevice.CTRE_MagEncoder_Relative,0,0);
    shooterLeftMotor.config_kP(0, ShooterConstants.shooterKP);
    shooterLeftMotor.config_kI(0, ShooterConstants.shooterKI);
    shooterLeftMotor.config_kD(0, ShooterConstants.shooterKD);
    shooterLeftMotor.config_kF(0, ShooterConstants.shooterKF);

    shooterLeftMotor.configClosedloopRamp(ShooterConstants.shooterRampRate);
    shooterRightMotor.configClosedloopRamp(ShooterConstants.shooterRampRate);

    shooterLeftMotor.configVoltageCompSaturation(12);
    shooterLeftMotor.enableVoltageCompensation(true);
    shooterRightMotor.configVoltageCompSaturation(12);
    shooterRightMotor.enableVoltageCompensation(true);

    shooterLeftMotor.configAllowableClosedloopError(0, ShooterConstants.shooterTolerance);
  }

  public void setShooterSpeed(double speed) {
    shooterLeftMotor.set(ControlMode.Velocity, speed);
  }

  public double getShooterSpeed() {
    return shooterLeftMotor.getSelectedSensorVelocity();
  }

  public double getShooterError() {
    return shooterLeftMotor.getClosedLoopError();
  }

  public boolean atSetpoint() {
    return Math.abs(shooterLeftMotor.getClosedLoopError()) < ShooterConstants.shooterTolerance;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    // Update the Shuffleboard values
    speedEntry.setDouble(getShooterSpeed());
    errorEntry.setDouble(getShooterError());
    atSetpointEntry.setBoolean(atSetpoint());
  }
}