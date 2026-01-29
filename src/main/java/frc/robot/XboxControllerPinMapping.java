package frc.robot;

public class XboxControllerPinMapping implements InputPinMapping {
    @Override
    public int buttonIntake() {
        return 1;
    }

    @Override
    public int buttonShoot() {
        return 2;
    }

    @Override
    public int buttonAim() {
        return 3;
    }

    @Override
    public int buttonClimb() {
        return -1;
    }

    @Override
    public int buttonDescend() {
        return -1;
    }

    @Override
    public int axisDriveX() {
        return -1;
    }

    @Override
    public int axisDriveY() {
        return -1;
    }

    @Override
    public int axisSteerClockwise() {
        return -1;
    }

    @Override
    public int axisSteerAnticlockwise() {
        return -1;
    }
}
