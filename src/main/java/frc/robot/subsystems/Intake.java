// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Constants;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */

  //Shuffleboard Setup
  private ShuffleboardTab tab; 
  private GenericEntry intakeSpeedEntry;
  private GenericEntry beamBreakStateEntry;

  private TalonSRX intakeMotor;
  private DigitalInput beamBreak;

  public Intake() {
    tab = Shuffleboard.getTab("Intake");
    intakeSpeedEntry = tab.add("Intake Speed", 0.0).getEntry();
    beamBreakStateEntry = tab.add("Beam Break State", false).getEntry();

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

  @Override
  public void periodic() {
    intakeSpeedEntry.setDouble(intakeMotor.getMotorOutputPercent());
    beamBreakStateEntry.setBoolean(getBeamBreak());
  }
}