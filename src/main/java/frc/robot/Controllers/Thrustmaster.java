// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Controllers;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;

/** Add your docs here. */
public class Thrustmaster{
    private Joystick joystick;

    public Thrustmaster(int port) {
        joystick = new Joystick(port);
    }

    public double getXAxis() {
        return joystick.getRawAxis(Joystick.kDefaultXChannel);
    }

    public double getYAxis() {
        return joystick.getRawAxis(Joystick.kDefaultYChannel);
    }
    
    public double getTwistAxis() {
        return joystick.getRawAxis(Joystick.kDefaultZChannel);
    }

    public double getThrottle() {
        return joystick.getRawAxis(Joystick.kDefaultThrottleChannel);
    }
 
    public JoystickButton getTrigger() {
        return new JoystickButton(joystick, 1);
    }

    public JoystickButton getStickLeftButton() {
        return new JoystickButton(joystick, 3);
    }

    public JoystickButton getStickRightButton() {
        return new JoystickButton(joystick, 4);
    }

    public JoystickButton getStickCenterButton() {
        return new JoystickButton(joystick, 2);
    }

    public JoystickButton getFingerLeftTop1() {
        return new JoystickButton(joystick, 5);
    }

    public JoystickButton getFingerLeftBottom1() {
        return new JoystickButton(joystick, 10);
    }

    public JoystickButton getFingerLeftTop2() {
        return new JoystickButton(joystick, 6);
    }

    public JoystickButton getFingerLeftBottom2() {
        return new JoystickButton(joystick, 9);
    }

    public JoystickButton getFingerLeftTop3() {
        return new JoystickButton(joystick, 7);
    }

    public JoystickButton getFingerLeftBottom3() {
        return new JoystickButton(joystick, 8);
    }

    public JoystickButton getFingerRightTop1() {
        return new JoystickButton(joystick, 13);
    }

    public JoystickButton getFingerRightBottom1() {
        return new JoystickButton(joystick, 14);
    }

    public JoystickButton getFingerRightTop2() {
        return new JoystickButton(joystick, 12);
    }

    public JoystickButton getFingerRightBottom2() {
        return new JoystickButton(joystick, 15);
    }

    public JoystickButton getFingerRightTop3() {
        return new JoystickButton(joystick, 11);
    }

    public JoystickButton getFingerRightBottom3() {
        return new JoystickButton(joystick, 16);
    }

    public POVButton getUpPovButton() {
        return new POVButton(joystick, 0);
    }

    public POVButton getRightPovButton() {
        return new POVButton(joystick, 90);
    }

    public POVButton getDownPovButton() {
        return new POVButton(joystick, 180);
    }

    public POVButton getLeftPovButton() {
        return new POVButton(joystick, 270);
    }
}