// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;

public class Limelight extends SubsystemBase {
  private ShuffleboardTab tab;
  private GenericEntry txEntry;
  private GenericEntry tyEntry;
  private GenericEntry tzEntry;
  private GenericEntry tXDist;
  private GenericEntry targetId;
  private GenericEntry distanceEntry;

  private GenericEntry kP;
  private GenericEntry kI;
  private GenericEntry kD;
  private GenericEntry feedforward;

  public Limelight() {
    tab = Shuffleboard.getTab("Limelight");

    txEntry = tab.add("tx", 0.0).getEntry();
    tyEntry = tab.add("ty", 0.0).getEntry();
    tzEntry = tab.add("tz", 0.0).getEntry();
    tXDist = tab.add("txDist", 0.0).getEntry();
    targetId = tab.add("targetId", 0).getEntry();
    distanceEntry = tab.add("Distance", 0.0).getEntry();

    kP = tab.add("kP", 0.0).getEntry();
    kI = tab.add("kI", 0.0).getEntry();
    kD = tab.add("kD", 0.0).getEntry();
    feedforward = tab.add("Feedforward", 0.0).getEntry();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    txEntry.setDouble(LimelightHelpers.getTX("limelight"));
    tyEntry.setDouble(LimelightHelpers.getTY("limelight"));
    tzEntry.setDouble(Units.metersToFeet(LimelightHelpers.getCameraPose3d_TargetSpace("limelight").getZ()));
    tXDist.setDouble(Units.metersToFeet(LimelightHelpers.getCameraPose3d_TargetSpace("limelight").getX()));
    targetId.setDouble(LimelightHelpers.getFiducialID("limelight"));

    double distance = 
      Math.sqrt(
          Math.pow(LimelightHelpers.getCameraPose3d_TargetSpace("limelight").getX(), 2) + 
          Math.pow(LimelightHelpers.getCameraPose3d_TargetSpace("limelight").getZ(), 2)
      );

    distanceEntry.setDouble(Units.metersToFeet(distance));
  }
}


//Red Speaker 3
//Blue Speaker 7
//Red Amp 5
//Blue Amp 6