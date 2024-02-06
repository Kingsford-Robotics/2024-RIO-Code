// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Controllers;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;

/** Add your docs here. */
public class Logitech{
    private Joystick joystick;

    public Logitech(int port) {
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

    public JoystickButton getSideButton() {
        return new JoystickButton(joystick, 2);
    }

    public JoystickButton getButton3() {
        return new JoystickButton(joystick, 3);
    }

    public JoystickButton getButton4() {
        return new JoystickButton(joystick, 4);
    }

    public JoystickButton getButton5() {
        return new JoystickButton(joystick, 5);
    }

    public JoystickButton getButton6() {
        return new JoystickButton(joystick, 6);
    }

    public JoystickButton getButton7() {
        return new JoystickButton(joystick, 7);
    }

    public JoystickButton getButton8() {
        return new JoystickButton(joystick, 8);
    }

    public JoystickButton getButton9() {
        return new JoystickButton(joystick, 9);
    }

    public JoystickButton getButton10() {
        return new JoystickButton(joystick, 10);
    }

    public JoystickButton getButton11() {
        return new JoystickButton(joystick, 11);
    }

    public JoystickButton getButton12() {
        return new JoystickButton(joystick, 12);
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