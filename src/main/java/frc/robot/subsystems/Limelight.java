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


  private NetworkTable table;
  private LedMode ledMode;

  private ShuffleboardTab tab;
  private GenericEntry txEntry;
  private GenericEntry tyEntry;
  private GenericEntry tzEntry;


  public Limelight() {
    ledMode = LedMode.kOff;
    tab = Shuffleboard.getTab("Limelight");

    txEntry = tab.add("tx", 0.0).getEntry();
    tyEntry = tab.add("ty", 0.0).getEntry();
    tzEntry = tab.add("tz", 0.0).getEntry();

    table = NetworkTableInstance.getDefault().getTable("limelight");
  }

  public void setLedMode(LedMode ledMode) {
    this.ledMode = ledMode;
    table.getEntry("ledMode").setNumber(ledMode.value);
  }

  public double getTx() {
    //Get target x pose
    return NetworkTableInstance.getDefault().getTable("limelight").getEntry("botpose_targetspace").getDoubleArray(new double[3])[0];
  }

  public double getTy() {
    //Get target y pose
    return table.getEntry("botpose_targetspace").getDoubleArray(new double[3])[1];
  }

  public double getTz() {
    //Get target z pose
    return table.getEntry("botpose_targetspace").getDoubleArray(new double[3])[2];
  }

  public double getAngle()
  {
    return table.getEntry("tx").getDouble(0.0);
  }

  public boolean isTargetFound()
  {
    return table.getEntry("tv").getDouble(0.0) == 1.0;
  }

  public void setPipeline(int pipeline)
  {
    table.getEntry("pipeline").setNumber(pipeline);
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
