// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Constants;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */

  private TalonSRX intakeMotor;
  private DigitalInput beamBreak;

  public Intake() {
    intakeMotor = new TalonSRX(Constants.IntakeConstants.intakeMotorID);
    intakeMotor.configAllSettings(new TalonSRXConfiguration());

    beamBreak = new DigitalInput(Constants.IntakeConstants.beamBreakID);

    intakeMotor.setNeutralMode(NeutralMode.Coast);
  }

  /**
   * Sets the percent output of the intake motor [-1, 1].
   * @param speed
   */
  public void setSpeed(double speed) {
    intakeMotor.set(ControlMode.PercentOutput, speed);
  }

  /**
   * Returns the state of the beam break sensor. True if the beam is broken, false if not.
   * @return
   */
  public boolean getBeamBreak() {
    return beamBreak.get();
  }
}