// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Constants;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.IntSupplier;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.Controllers.IJoystick;
import frc.robot.Controllers.Thrustmaster;

/** Add your docs here. */
public class OIConstants {
    //Deadband for Joystick
    public static final double stickDeadband = 0.05;

    //Drive Speeds
    public static final double highTranslationSpeed = 1.0;
    public static final double lowTranslationSpeed = 0.15;

    public static final double highRotationSpeed = 1.0;
    public static final double lowRotationSpeed = 0.15;
    
    public static final double translateRampTime = 0.5;
    public static final double turnRampTime = 0.5;


    /*Drive Joysticks Setup*/
    public static final IJoystick driveJoystickLeft = new Thrustmaster(0);
    public static final IJoystick driveJoystickRight = new Thrustmaster(1);

    /*Controller*/
    public static final XboxController coDriverController = new XboxController(2);

    //Drive Joysticks Suppliers
    public static final Supplier<Double> translationSupplier = () -> 0.0;
    public static final Supplier<Double> strafeSupplier = () -> 0.0;
    public static final Supplier<Double> rotationSupplier = () -> 0.0;
    public static final BooleanSupplier slowSpeed = () -> false;
    public static final BooleanSupplier robotCentric = () -> false;


    //Drive Joysticks Buttons
    public static final JoystickButton resetGyro = new JoystickButton(driveJoystickRight, Thrustmaster.button3);    //Resets the gyro to 0 degrees.
    public static final JoystickButton calibrateArm = new JoystickButton(driveJoystickRight, Thrustmaster.button4); //Calibrates the arm encoder from CANCoder.
    public static final JoystickButton alignPlace = new JoystickButton(driveJoystickRight, Thrustmaster.button5);  //Aligns the robot to the target and places the cone or cube.
    public static final JoystickButton toggleFront = new JoystickButton(driveJoystickRight, Thrustmaster.button6);  //Toggles front from turntable to arm side of robot.

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