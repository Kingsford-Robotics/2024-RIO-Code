// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.opencv.core.Core;
import org.opencv.core.Mat;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.RobotContainer.targetMode;

public class CompetitionData extends SubsystemBase {
  /** Creates a new CompetitionData. */
  
  private GenericEntry operatorMode;
  private GenericEntry elevatorHeight;
  private GenericEntry matchTime;
  private GenericEntry batteryVoltage;

  private RobotContainer m_RobotContainer;
  private Elevator m_Elevator;

  private ShuffleboardTab tab;

  private Command retractActuator;
  
  public CompetitionData(RobotContainer robotContainer, Elevator elevator) {
    this.m_RobotContainer = robotContainer;
    this.m_Elevator = elevator;

    retractActuator = new InstantCommand(() -> elevator.retractActuator(), elevator);
    
    tab = Shuffleboard.getTab("Competition");
    
    operatorMode = tab.add("Mode", "Speaker").getEntry();
    elevatorHeight = tab.add("Elevator Height", 0.0).getEntry();
    matchTime = tab.add("Match Time", 0.0).getEntry();
    batteryVoltage = tab.add("Battery Voltage", 0.0).getEntry();
    tab.add("Retract Actuator", retractActuator);
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    operatorMode.setString(m_RobotContainer.m_TargetMode == targetMode.kSpeaker? "Speaker": "Amp");
    elevatorHeight.setDouble(Units.metersToInches(m_Elevator.getHeight()));  
    matchTime.setDouble(Timer.getMatchTime());
    batteryVoltage.setDouble(RobotController.getBatteryVoltage());
  }
}
