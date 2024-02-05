// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;

/** Add your docs here. */
public class Thrustmaster implements IJoystick {
   
    private Joystick joystick;

    public Thrustmaster(int port) {
        joystick = new Joystick(port);
    }



    @Override
    public double getXAxis() {
        return joystick.getRawAxis(Joystick.kDefaultXChannel);
    }

    @Override
    public double getYAxis() {
        return joystick.getRawAxis(Joystick.kDefaultYChannel);
    }

    @Override
    public double getTwistAxis() {
        return joystick.getRawAxis(Joystick.kDefaultZChannel);
    }

    @Override
    public double getThrottle() {
        return joystick.getRawAxis(Joystick.kDefaultThrottleChannel);
    }

    @Override
    public boolean getTrigger() {
        return joystick.getRawButton(1);
    }

    @Override
    public JoystickButton getThumbButton() {
        return null;
    }

    @Override
    public JoystickButton getTopLeftButton() {
        return new JoystickButton(joystick, 0);
    }

    @Override
    public JoystickButton getTopRightButton() {
        return new JoystickButton(joystick, 1);
    }

    @Override
    public JoystickButton getBottomLeftButton() {
        return new JoystickButton(joystick, 2);
    }

    @Override
    public JoystickButton getBottomRightButton() {
        return new JoystickButton(joystick, 3);
    }

    @Override
    public JoystickButton getButton1() {
        return new JoystickButton(joystick, 4);
    }

    @Override
    public JoystickButton getButton2() {
        return new JoystickButton(joystick, 5);
    }

    @Override
    public JoystickButton getButton3() {
        return new JoystickButton(joystick, 6);
    }

    @Override
    public JoystickButton getButton4() {
        return new JoystickButton(joystick, 7);
    }

    @Override
    public JoystickButton getButton5() {
        return new JoystickButton(joystick, 8);
    }

    @Override
    public JoystickButton getButton6() {
        return new JoystickButton(joystick, 9);
    }

    @Override
    public JoystickButton getButton7() {
        return new JoystickButton(joystick, 10);
    }

    @Override
    public JoystickButton getButton8() {
        return new JoystickButton(joystick, 11);
    }
    
    @Override
    public JoystickButton getButton9() {
        return new JoystickButton(joystick, 12);
    }

    @Override
    public JoystickButton getButton10() {
        return new JoystickButton(joystick, 13);
    }

    @Override
    public JoystickButton getButton11() {
        return new JoystickButton(joystick, 14);
    }

    @Override
    public JoystickButton getButton12() {
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

    @Override
    public POVState getPovState() {
        int pov = joystick.getPOV();
        switch(pov) {
            case 0:
                return POVState.UP;
            case 90:
                return POVState.RIGHT;
            case 180:
                return POVState.DOWN;
            case 270:
                return POVState.LEFT;
            default:
                return POVState.CENTER;
        }
    }
}