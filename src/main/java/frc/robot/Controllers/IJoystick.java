package frc.robot.Controllers;

import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;

public interface IJoystick{
    public double getXAxis();
    public double getYAxis();
    public double getTwistAxis();
    public double getThrottle();
    
    public boolean getTrigger();

    /**
     * Returns the state of the thumb button. Not all joysticks have a thumb button and will return false.
     */
    public JoystickButton getThumbButton();    //Not included on all joysticks.
    public JoystickButton getTopLeftButton();
    public JoystickButton getTopRightButton();
    public JoystickButton getBottomLeftButton();
    public JoystickButton getBottomRightButton();

    public JoystickButton getButton1();
    public JoystickButton getButton2();
    public JoystickButton getButton3();
    public JoystickButton getButton4();
    public JoystickButton getButton5();
    public JoystickButton getButton6();
    public JoystickButton getButton7();
    public JoystickButton getButton8();
    public JoystickButton getButton9();
    public JoystickButton getButton10();
    public JoystickButton getButton11();
    public JoystickButton getButton12();

    public POVButton getUpPovButton();
    public POVButton getDownPovButton();
    public POVButton getLeftPovButton();
    public POVButton getRightPovButton();

    public POVState getPovState();

    public static enum POVState
    {
        CENTER(-1),
        LEFT(270),
        RIGHT(90),
        UP(0),
        DOWN(180);

        private final int value;

        POVState(final int value){
            this.value = value;
        }

        public int value() {return value;}
    }
}