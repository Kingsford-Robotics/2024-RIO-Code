// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonSRXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Constants.ShooterConstants;

public class Shooter extends SubsystemBase {
  /** Creates a new Shooter. */

  private TalonSRX shooterLeftMotor;
  private TalonSRX shooterRightMotor;

  public Shooter() {
    shooterLeftMotor = new TalonSRX(ShooterConstants.shooterLeftMotorID);
    shooterRightMotor = new TalonSRX(ShooterConstants.shooterRightMotorID);

    shooterLeftMotor.configFactoryDefault();
    shooterRightMotor.configFactoryDefault();

    shooterLeftMotor.setInverted(true);
    shooterLeftMotor.follow(shooterRightMotor);

    shooterLeftMotor.configSelectedFeedbackSensor(TalonSRXFeedbackDevice.CTRE_MagEncoder_Relative,0,0);
    shooterLeftMotor.config_kP(0, ShooterConstants.shooterKP);
    shooterLeftMotor.config_kI(0, ShooterConstants.shooterKI);
    shooterLeftMotor.config_kD(0, ShooterConstants.shooterKD);
    shooterLeftMotor.config_kF(0, ShooterConstants.shooterKF);
  }

  public void setShooterSpeed(double speed) {
    shooterLeftMotor.set(ControlMode.Velocity, speed);
  }

  /*
   * This method updates the PID values for the shooter motor from the Constants class.
   */
  public void updatePIDFConstants(){
    shooterLeftMotor.config_kP(0, ShooterConstants.shooterKP);
    shooterLeftMotor.config_kI(0, ShooterConstants.shooterKI);
    shooterLeftMotor.config_kD(0, ShooterConstants.shooterKD);
    shooterLeftMotor.config_kF(0, ShooterConstants.shooterKF);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
