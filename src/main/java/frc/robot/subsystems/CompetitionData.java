// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.io.ObjectInputStream.GetField;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.cscore.MjpegServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.util.PixelFormat;
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

  private RobotContainer m_RobotContainer;
  private Elevator m_Elevator;

  private ShuffleboardTab tab;

  private CvSink cvSink;
  private UsbCamera backCamera;
  private UsbCamera chainCamera;

  private boolean isChainCameraActive = false;
  
  public CompetitionData(RobotContainer robotContainer, Elevator elevator) {
    this.m_RobotContainer = robotContainer;
    this.m_Elevator = elevator;
    
    tab = Shuffleboard.getTab("Competition");
    
    operatorMode = tab.add("Mode", "Speaker").getEntry();
    elevatorHeight = tab.add("Elevator Height", 0.0).getEntry();
    matchTime = tab.add("Match Time", 0.0).getEntry();
    batteryVoltage = tab.add("Battery Voltage", 0.0).getEntry();

    backCamera = CameraServer.startAutomaticCapture(0);
    backCamera.setResolution(320, 240);

    chainCamera = CameraServer.startAutomaticCapture(1);
    chainCamera.setResolution(320, 240);

    cvSink = CameraServer.getVideo();
    switchCamera(false); // Start with backCamera

    Thread m_visionThread;
    m_visionThread = new Thread(() -> {
        // Setup a CvSource. This will send images back to the Dashboard
        CvSource outputStream = CameraServer.putVideo("Camera", 320, 240);

        // Mats are very memory expensive. Lets reuse this Mat.
        Mat mat = new Mat();

        while (!Thread.interrupted()) {
            if (cvSink.grabFrame(mat) == 0) {
                outputStream.notifyError(cvSink.getError());
                continue;
            }

            // Only flip the image if the chainCamera is active
            if (isChainCameraActive) {
                Core.flip(mat, mat, 0);
            }

            outputStream.putFrame(mat);
        }
    });
    m_visionThread.setDaemon(true);
    m_visionThread.start();
  }

  public void switchCamera(boolean useChainCamera) {
    isChainCameraActive = useChainCamera;

    if (useChainCamera) {
        cvSink.setSource(chainCamera);
    } else {
        cvSink.setSource(backCamera);
    }
  }

  public boolean isChainCamera(){
    return isChainCameraActive;
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
