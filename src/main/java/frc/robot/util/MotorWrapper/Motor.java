package frc.robot.util.MotorWrapper;

public abstract class Motor {
    protected int ID;
    protected boolean inverted;
    protected boolean brushless;
    protected boolean brake;

    protected Motor(int ID, boolean inverted, boolean brushless, boolean brake) {
        this.ID = ID;
        this.inverted = inverted;
        this.brushless = brushless;
        this.brake = brake;
        initialize();
    }

    public abstract void initialize();
}
