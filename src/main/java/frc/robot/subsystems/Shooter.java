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
    speedEntry = tab.add("Shooter Speed (RPM)", 0).getEntry();
    errorEntry = tab.add("Shooter Error (RPM)", 0).getEntry();
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
    shooterLeftMotor.config_kF(0, ShooterConstants.shooterKF);

    shooterLeftMotor.configClosedloopRamp(ShooterConstants.shooterRampRate);
    shooterRightMotor.configClosedloopRamp(ShooterConstants.shooterRampRate);

    shooterLeftMotor.configVoltageCompSaturation(12);
    shooterLeftMotor.enableVoltageCompensation(true);
    shooterRightMotor.configVoltageCompSaturation(12);
    shooterRightMotor.enableVoltageCompensation(true);

    //Converts the tolerance from RPM to encoder units and divides by 2 to ensure that the error is within the tolerance.
    shooterLeftMotor.configAllowableClosedloopError(0, ShooterConstants.shooterToleranceRPM * 4096 / 600 / 2);
  }

  /**
   * Sets the shooter speed in RPM.
   */
  public void setShooterRPM(double speed) {
    shooterLeftMotor.set(ControlMode.Velocity, speed * 4096 / 600);
  }

  /*
   * Sets the shooter speed in percent output.
   */
  public void setShooterPercent(double percent){
    shooterLeftMotor.set(ControlMode.PercentOutput, percent);
  }

  /**
   * Returns the shooter speed.
   * @return The shooter speed in RPM.
   */
  public double getShooterRPM() {
    return shooterLeftMotor.getSelectedSensorVelocity() / 4096 * 600;
  }

  /**
   * Returns the shooter error.
   * @return The shooter speed error in RPM.
   */
  public double getShooterErrorRPM() {
    return shooterLeftMotor.getClosedLoopError() / 4096 * 600;
  }

  /**
   * Returns whether the shooter is at the setpoint based on the RPM tolerance.
   */
  public boolean atSetpoint() {
    return Math.abs(getShooterErrorRPM()) < ShooterConstants.shooterToleranceRPM;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    // Update the Shuffleboard values
    speedEntry.setDouble(getShooterRPM());
    errorEntry.setDouble(getShooterErrorRPM());
    atSetpointEntry.setBoolean(atSetpoint());
  }
}