// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ResourceBundle.Control;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Constants.PivotConstants;

public class Pivot extends SubsystemBase {
  /** Creates a new Pivot. */

  CANSparkMax pivotLeftMotor;
  CANSparkMax pivotRightMotor;

  public Pivot() {
    pivotLeftMotor = new CANSparkMax(PivotConstants.pivotLeftMotorID, MotorType.kBrushless);
    pivotRightMotor = new CANSparkMax(PivotConstants.pivotRightMotorID, MotorType.kBrushless);

    pivotLeftMotor.restoreFactoryDefaults();
    pivotRightMotor.restoreFactoryDefaults();

    pivotRightMotor.setInverted(true);
    pivotRightMotor.follow(pivotLeftMotor, true);

    pivotLeftMotor.setSmartCurrentLimit(PivotConstants.pivotCurrentLimit);
    pivotRightMotor.setSmartCurrentLimit(PivotConstants.pivotCurrentLimit);

    pivotLeftMotor.setOpenLoopRampRate(PivotConstants.openLoopRamp);
    pivotRightMotor.setOpenLoopRampRate(PivotConstants.openLoopRamp);

    pivotLeftMotor.setClosedLoopRampRate(PivotConstants.closedLoopRamp);
    pivotRightMotor.setClosedLoopRampRate(PivotConstants.closedLoopRamp);

    pivotLeftMotor.burnFlash();
    pivotRightMotor.burnFlash();
  }


  //TODO: Use smart motion profile to control angle. Remember to use cosine gravity compensation.
  public void setPivotAngle(double angle){
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
