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
    public static final double lowTranslationSpeed = 0.15;

    public static final double highRotationSpeed = 1.0;
    public static final double lowRotationSpeed = 0.15;

    public static final double translateRampTime = 0.5;
    public static final double turnRampTime = 0.5;

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

    //Handheld Controller Suppliers
    public static final DoubleSupplier elevatorSpeed = () -> coDriverController.getLeftY();
    public static final DoubleSupplier pivotSpeed = () -> coDriverController.getRightY();

    public static final JoystickButton speakerTarget = new JoystickButton(coDriverController, XboxController.Button.kY.value);
    public static final JoystickButton ampTarget = new JoystickButton(coDriverController, XboxController.Button.kA.value);
    public static final JoystickButton passTarget = new JoystickButton(coDriverController, XboxController.Button.kB.value);

    public static final JoystickButton climbDeploy = new JoystickButton(coDriverController, XboxController.Button.kLeftBumper.value);
    public static final JoystickButton climbRetract = new JoystickButton(coDriverController, XboxController.Button.kRightBumper.value);

    public static final JoystickButton reverseIntake = new JoystickButton(coDriverController, XboxController.Button.kBack.value);

    public static final POVButton snapFront = new POVButton(coDriverController, 0);
    public static final POVButton snapBack = new POVButton(coDriverController, 180);
    public static final POVButton snapRight = new POVButton(coDriverController, 90);
    public static final POVButton snapLeft = new POVButton(coDriverController, 270);
}