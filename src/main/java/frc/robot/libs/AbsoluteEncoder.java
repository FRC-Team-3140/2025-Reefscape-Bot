package frc.robot.libs;

import edu.wpi.first.wpilibj.AnalogEncoder;

public class AbsoluteEncoder extends AnalogEncoder {
    public AbsoluteEncoder(int analogID) {
        super(analogID);
    }

    public double getAbsolutePosition() {
        // TODO: return (super.getAbsolutePosition() - super.getPositionOffset()) * 360;
        return 0;
    }

    public void setPositionOffset(double offset) {
        // super.setPositionOffset(offset / 360);
    }

}