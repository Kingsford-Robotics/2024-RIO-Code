// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.MjpegServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.util.PixelFormat;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.RobotContainer.targetMode;

public class CompetitionData extends SubsystemBase {
  /** Creates a new CompetitionData. */
  
  private GenericEntry operatorMode;
  private GenericEntry elevatorHeight;

  private RobotContainer m_RobotContainer;
  private Elevator m_Elevator;

  private ShuffleboardTab tab;
  
  private UsbCamera backCamera;

  private MjpegServer backCameraServ;
  
  public CompetitionData(RobotContainer robotContainer, Elevator elevator) {
    this.m_RobotContainer = robotContainer;
    this.m_Elevator = elevator;
    
    tab = Shuffleboard.getTab("Competition");
    backCamera = CameraServer.startAutomaticCapture(0);
    backCamera.setVideoMode(PixelFormat.kMJPEG, 480, 320, 10);
    backCameraServ = new MjpegServer("Turntable", 1182);

    backCameraServ.setSource(backCamera);
    tab.add("Back Camera", backCameraServ.getSource());

    operatorMode = tab.add("Mode", "Speaker").getEntry();
    elevatorHeight = tab.add("Elevator Height", 0.0).getEntry();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    operatorMode.setString(m_RobotContainer.m_TargetMode == targetMode.kSpeaker? "Speaker": "Amp");
    elevatorHeight.setDouble(Units.metersToInches(m_Elevator.getHeight()));  
  }
}
