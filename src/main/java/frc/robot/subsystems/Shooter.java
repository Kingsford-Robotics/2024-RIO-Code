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

  private TalonSRX shooterLeftMotor;
  private TalonSRX shooterRightMotor;

  public Shooter() {
    tab = Shuffleboard.getTab("Shooter");
    speedEntry = tab.add("Shooter Speed (RPM)", 0).getEntry();

    shooterLeftMotor = new TalonSRX(ShooterConstants.shooterLeftMotorID);
    shooterRightMotor = new TalonSRX(ShooterConstants.shooterRightMotorID);

    shooterLeftMotor.configFactoryDefault();
    shooterRightMotor.configFactoryDefault();

    shooterLeftMotor.setInverted(true);
    shooterRightMotor.setInverted(false);

    shooterRightMotor.follow(shooterLeftMotor);

    shooterLeftMotor.configSelectedFeedbackSensor(TalonSRXFeedbackDevice.CTRE_MagEncoder_Relative,0,0);

    shooterLeftMotor.configVoltageCompSaturation(12);
    shooterLeftMotor.enableVoltageCompensation(true);
    shooterRightMotor.configVoltageCompSaturation(12);
    shooterRightMotor.enableVoltageCompensation(true);
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

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    // Update the Shuffleboard values
    speedEntry.setDouble(getShooterRPM());
  }
}