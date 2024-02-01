// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.IntSupplier;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;

/** Add your docs here. */
public class OIConstants {

    /*Drive Joysticks Setup*/
    public static final Joystick driveJoystickLeft = new Joystick(0);
    public static final Joystick driveJoystickRight = new Joystick(1);

    //Drive Joysticks Suppliers
    public static final Supplier<Double> translationSupplier = () -> driveJoystickRight.getRawAxis(Joystick.kDefaultYChannel);
    public static final Supplier<Double> strafeSupplier = () -> driveJoystickRight.getRawAxis(Joystick.kDefaultXChannel);
    public static final Supplier<Double> rotationSupplier = () -> driveJoystickLeft.getRawAxis(Joystick.kDefaultXChannel);
    public static final BooleanSupplier slowSpeed = () -> driveJoystickRight.getRawButton(ThrustmasterJoystick.thumbButton);
    public static final BooleanSupplier robotCentric = () -> driveJoystickRight.getRawButton(ThrustmasterJoystick.trigger);

    public static final double highTranslationSpeed = 1.0;
    public static final double lowTranslationSpeed = 0.15;

    public static final double highRotationSpeed = 1.0;
    public static final double lowRotationSpeed = 0.15;

    public static final double translationDeadBand = 0.1;
    public static final double turnDeadBand = 0.1;

    //Drive Joysticks Buttons
    public static final JoystickButton resetGyro = new JoystickButton(driveJoystickRight, ThrustmasterJoystick.button3);    //Resets the gyro to 0 degrees.
    public static final JoystickButton calibrateArm = new JoystickButton(driveJoystickRight, ThrustmasterJoystick.button4); //Calibrates the arm encoder from CANCoder.
    public static final JoystickButton alignPlace = new JoystickButton(driveJoystickRight, ThrustmasterJoystick.button5);  //Aligns the robot to the target and places the cone or cube.
    public static final JoystickButton toggleFront = new JoystickButton(driveJoystickRight, ThrustmasterJoystick.button6);  //Toggles front from turntable to arm side of robot.

    //Sets the center of the rotation to the selected wheel while held. Returns to center of robot when released.
    private static final JoystickButton centerOfRotationFrontLeft = new JoystickButton(driveJoystickLeft, ThrustmasterJoystick.button5);
    private static final JoystickButton centerOfRotationFrontRight = new JoystickButton(driveJoystickLeft, ThrustmasterJoystick.button6); 
    private static final JoystickButton centerOfRotationBackLeft = new JoystickButton(driveJoystickLeft, ThrustmasterJoystick.button3);
    private static final JoystickButton centerOfRotationBackRight = new JoystickButton(driveJoystickLeft, ThrustmasterJoystick.button4);

    public static final IntSupplier centerOfRotation = () -> {
        if (OIConstants.centerOfRotationFrontLeft.getAsBoolean()) {
            return 0;
        } else if (OIConstants.centerOfRotationFrontRight.getAsBoolean()) {
            return 1;
        } else if (OIConstants.centerOfRotationBackLeft.getAsBoolean()) {
            return 2;
        } else if (OIConstants.centerOfRotationBackRight.getAsBoolean()) {
            return 3;
        } else {
            return -1;
        }
    };

    /*Operator Joystick Setup*/
    public static final XboxController coDriverController = new XboxController(2);

    //Handheld Controller
    public static final JoystickButton openClaw = new JoystickButton(coDriverController, XboxController.Button.kLeftBumper.value);
    public static final JoystickButton closeClaw = new JoystickButton(coDriverController, XboxController.Button.kRightBumper.value);

    public static final JoystickButton highGround = new JoystickButton(coDriverController, XboxController.Button.kY.value);
    public static final JoystickButton lowGround = new JoystickButton(coDriverController, XboxController.Button.kB.value);
    public static final JoystickButton turntablePickup = new JoystickButton(coDriverController, XboxController.Button.kX.value);
    public static final JoystickButton armHome = new JoystickButton(coDriverController, XboxController.Button.kA.value);

    public static final JoystickButton toggleConeCube = new JoystickButton(coDriverController, XboxController.Button.kBack.value);
    public static final JoystickButton toggleRamp = new JoystickButton(coDriverController, XboxController.Button.kStart.value);

    public static final DoubleSupplier armSpeed = () -> coDriverController.getLeftY();
    public static final DoubleSupplier elevatorSpeed = () -> coDriverController.getRightY();

    //Set turntable speed to right and left triggers
    public static final DoubleSupplier turntableSpeed = () -> 
    {
        if (coDriverController.getLeftTriggerAxis() > 0.1) {
            return coDriverController.getLeftTriggerAxis();
        } else if (coDriverController.getRightTriggerAxis() > 0.1) {
            return -coDriverController.getRightTriggerAxis();
        } else {
            return 0;
        }
    };
    
    public static final POVButton increaseLevel = new POVButton(coDriverController, 0);
    public static final POVButton decreaseLevel = new POVButton(coDriverController, 180);

    public static final POVButton alignRight = new POVButton(coDriverController, 90);
    public static final POVButton alignLeft = new POVButton(coDriverController, 270);
}