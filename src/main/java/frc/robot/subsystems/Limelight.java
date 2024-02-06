// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Limelight extends SubsystemBase {
  public enum LedMode {
    kOff(0), kOn(1), kBlink(2);

    public final int value;

    LedMode(int value) {
      this.value = value;
    }
  }

  private LedMode ledMode;
  private int tx;
  private int ty;
  private int tz;

  private NetworkTableInstance inst;
  private NetworkTable limelightTable;

  private ShuffleboardTab limelightTab;
  private GenericEntry txEntry;
  private GenericEntry tyEntry;
  private GenericEntry tzEntry;


  public Limelight() {
    ledMode = LedMode.kOff;
    limelightTab = Shuffleboard.getTab("Limelight");

    txEntry = limelightTab.add("tx", 0.0).getEntry();
    tyEntry = limelightTab.add("ty", 0.0).getEntry();
    tzEntry = limelightTab.add("tz", 0.0).getEntry();

    
  }

  public void setLedMode(LedMode ledMode) {
    this.ledMode = ledMode;
    NetworkTableInstance.getDefault().getTable("limelight-rok").getEntry("ledMode").setNumber(ledMode.value);
  }

  public double getTx() {
    //Get target x pose
    return NetworkTableInstance.getDefault().getTable("limelight-rok").getEntry("botpose_targetspace").getDoubleArray(new double[3])[0];
  }

  public double getTy() {
    //Get target y pose
    return NetworkTableInstance.getDefault().getTable("limelight-rok").getEntry("botpose_targetspace").getDoubleArray(new double[3])[1];
  }

  public double getTz() {
    //Get target z pose
    return NetworkTableInstance.getDefault().getTable("limelight-rok").getEntry("botpose_targetspace").getDoubleArray(new double[3])[2];
  }

  public double getAngle()
  {
    return NetworkTableInstance.getDefault().getTable("limelight-rok").getEntry("tx").getDouble(0.0);
  }

  public boolean isTargetFound()
  {
    return NetworkTableInstance.getDefault().getTable("limelight-rok").getEntry("tv").getDouble(0.0) == 1.0;
  }

  public void setPipeline(int pipeline)
  {
    NetworkTableInstance.getDefault().getTable("limelight-rok").getEntry("pipeline").setNumber(pipeline);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    txEntry.setDouble(getTx());
    tyEntry.setDouble(getTy());
    tzEntry.setDouble(getTz());

    SmartDashboard.putBoolean("Target Found", isTargetFound());
  }
}
