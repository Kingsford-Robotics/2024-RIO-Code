public interface IJoystick{
    public double getXAxis();
    public double getYAxis();
    public double getTwistAxis();
    public double getThrottle();
    
    public boolean getTrigger();
    public boolean getThumbButton();    //Not included on all joysticks.

    public boolean getTopLeftButton();
    public boolean getTopRightButton();
    public boolean getBottomLeftButton();
    public boolean getBottomRightButton();

    public boolean getButton1();
    public boolean getButton2();
    public boolean getButton3();
    public boolean getButton4();
    public boolean getButton5();
    public boolean getButton6();
    public boolean getButton7();
    public boolean getButton8();
    public boolean getButton9();
    public boolean getButton10();
    public boolean getButton11();
    public boolean getButton12();

    public POVState getPOV();

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