// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Constants;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
//import frc.robot.Controllers.Logitech;
import frc.robot.Controllers.Thrustmaster;

/** Add your docs here. */
public class OIConstants {
    //Deadband for Joystick
    public static final double stickDeadband = 0.1;

    //Drive Speeds
    public static final double highTranslationSpeed = 1.0;
    public static final double lowTranslationSpeed = 0.2;

    public static final double highRotationSpeed = 1.0;
    public static final double lowRotationSpeed = 0.2;

    public static final double translateRampTime = 0.6;
    public static final double turnRampTime = 0.6;

    /*Drive Joysticks Setup*/
    public static final Thrustmaster driveLeft = new Thrustmaster(0);
    public static final Thrustmaster driveRight = new Thrustmaster(1);

    /*Controller*/
    public static final XboxController coDriverController = new XboxController(2);

    //Drive Joysticks Suppliers
    public static final Supplier<Double> translationSupplier = () -> driveLeft.getYAxis();
    public static final Supplier<Double> strafeSupplier = () -> driveLeft.getXAxis();
    public static final Supplier<Double> rotationSupplier = () -> driveRight.getXAxis();
    public static final BooleanSupplier slowSpeed = () -> driveLeft.getStickRightButton().getAsBoolean();
    public static final BooleanSupplier robotCentric = () -> driveRight.getStickLeftButton().getAsBoolean();

    //Drive Joystick Buttons
    public static final JoystickButton deployIntake = driveLeft.getTrigger();
    public static final JoystickButton shoot = driveRight.getTrigger();
    public static final JoystickButton homeButton = driveLeft.getStickCenterButton();
    public static final JoystickButton resetGyro = driveRight.getStickCenterButton();

    //Handheld Controller Suppliers
    public static final DoubleSupplier elevatorSpeed = () -> coDriverController.getLeftY();
    public static final DoubleSupplier pivotSpeed = () -> coDriverController.getRightY();
    public static final DoubleSupplier shooterSpeed = () -> coDriverController.getLeftTriggerAxis();
    public static final DoubleSupplier intakeSpeed = () -> coDriverController.getRightTriggerAxis();

    public static final JoystickButton speakerTarget = new JoystickButton(coDriverController, XboxController.Button.kY.value);
    public static final JoystickButton ampTarget = new JoystickButton(coDriverController, XboxController.Button.kA.value);
    public static final JoystickButton trapTarget = new JoystickButton(coDriverController, XboxController.Button.kB.value);

    public static final JoystickButton manualActive = new JoystickButton(coDriverController, XboxController.Button.kLeftBumper.value);
    public static final JoystickButton cameraToggle = new JoystickButton(coDriverController, XboxController.Button.kRightBumper.value);

    public static final JoystickButton reverseIntake = new JoystickButton(coDriverController, XboxController.Button.kX.value);

    public static final JoystickButton climberDeploy = new JoystickButton(coDriverController, XboxController.Button.kStart.value);
    public static final JoystickButton climberRetract = new JoystickButton(coDriverController, XboxController.Button.kBack.value);

    //Down is 180, Right is 90, Up is 0, Left is 270
    public static final POVButton podiumManualShot = new POVButton(coDriverController, 0);
    public static final POVButton nearManualShot = new POVButton(coDriverController, 180);

    public static final POVButton snapFront = driveLeft.getUpPovButton();
    public static final POVButton snapBack = driveLeft.getDownPovButton();
    public static final POVButton snapRight = driveLeft.getRightPovButton();
    public static final POVButton snapLeft = driveLeft.getLeftPovButton();
}