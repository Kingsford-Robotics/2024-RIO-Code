// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.can.VictorSPXConfiguration;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Constants;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */

  private VictorSPX intakeMotor;
  private DigitalInput beamBreak;

  public Intake() {
    intakeMotor = new VictorSPX(Constants.IntakeConstants.intakeMotorID);
    intakeMotor.configAllSettings(new VictorSPXConfiguration());

    beamBreak = new DigitalInput(Constants.IntakeConstants.beamBreakID);
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

  @Override
  public void periodic() {
  }
}
