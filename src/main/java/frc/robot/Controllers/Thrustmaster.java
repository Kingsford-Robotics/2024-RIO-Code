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
        return new JoystickButton(joystick, 0);
    }

    public JoystickButton getTopLeftButton() {
        return new JoystickButton(joystick, 0);
    }

    public JoystickButton getTopRightButton() {
        return new JoystickButton(joystick, 1);
    }

    public JoystickButton getBottomLeftButton() {
        return new JoystickButton(joystick, 2);
    }

    public JoystickButton getBottomRightButton() {
        return new JoystickButton(joystick, 3);
    }

    public JoystickButton getButtonLeftTop1() {
        return new JoystickButton(joystick, 4);
    }

    public JoystickButton getButtonLeftBottom1() {
        return new JoystickButton(joystick, 5);
    }

    public JoystickButton getButtonLeftTop2() {
        return new JoystickButton(joystick, 6);
    }

    public JoystickButton getButtonLeftBottom2() {
        return new JoystickButton(joystick, 7);
    }

    public JoystickButton getButtonLeftTop3() {
        return new JoystickButton(joystick, 8);
    }

    public JoystickButton getButtonLeftBottom3() {
        return new JoystickButton(joystick, 9);
    }

    public JoystickButton getButtonRightTop1() {
        return new JoystickButton(joystick, 10);
    }

    public JoystickButton getButtonRightBottom1() {
        return new JoystickButton(joystick, 11);
    }

    public JoystickButton getButtonRightTop2() {
        return new JoystickButton(joystick, 12);
    }

    public JoystickButton getButtonRightBottom2() {
        return new JoystickButton(joystick, 13);
    }

    public JoystickButton getButtonRightTop3() {
        return new JoystickButton(joystick, 14);
    }

    public JoystickButton getButtonRightBottom3() {
        return new JoystickButton(joystick, 15);
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