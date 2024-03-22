// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.RobotContainer.targetMode;

public class CompetitionData extends SubsystemBase {
  /** Creates a new CompetitionData. */
  
  private GenericEntry operatorMode;
  private GenericEntry elevatorHeight;
  private GenericEntry matchTime;
  private GenericEntry batteryVoltage;
  private GenericEntry heading;

  private RobotContainer m_RobotContainer;
  private Elevator m_Elevator;
  private Swerve m_Swerve;

  private ShuffleboardTab tab;

  //private UsbCamera chainCamera;
  
  public CompetitionData(RobotContainer robotContainer, Elevator elevator, Swerve swerve) {
    this.m_RobotContainer = robotContainer;
    this.m_Elevator = elevator;
    this.m_Swerve = swerve;

    CameraServer.startAutomaticCapture(0);
    
    tab = Shuffleboard.getTab("Competition");
    
    operatorMode = tab.add("Mode", "Speaker").getEntry();
    elevatorHeight = tab.add("Elevator Height", 0.0).getEntry();
    matchTime = tab.add("Match Time", 0.0).getEntry();
    batteryVoltage = tab.add("Battery Voltage", 0.0).getEntry();
    heading = tab.add("Heading", 0.0).getEntry();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    String targetString;

    switch (m_RobotContainer.m_TargetMode) {
      case kSpeaker:
        targetString = "Speaker";
        break;
      case kAmp:
        targetString = "Amp";
        break;
      case kMidfield:
        targetString = "Midfield";
        break;
      case kNear:
        targetString = "Near";
        break;
      case kPodium:
        targetString = "Podium";
        break;
      default:
        targetString = "Unknown";
        break;
    }

    operatorMode.setString(targetString);
    elevatorHeight.setDouble(Units.metersToInches(m_Elevator.getHeight()));  
    matchTime.setDouble(Timer.getMatchTime());
    batteryVoltage.setDouble(RobotController.getBatteryVoltage());
    heading.setDouble(m_Swerve.getHeading().getDegrees());
  }
}
