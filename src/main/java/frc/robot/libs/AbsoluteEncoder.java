package frc.robot.libs;

import edu.wpi.first.wpilibj.AnalogEncoder;

public class AbsoluteEncoder extends AnalogEncoder {
    private final double zeroOffset;
    
    public AbsoluteEncoder(int analogID, double zeroOffset) {
        // Initializes an encoder on a DIO port and uses 360 as full range and 180 as half of the range
        super(analogID, 360.0, 0.0);

        this.zeroOffset = zeroOffset;
    }

    public double getAbsolutePosition() {
        return super.get() - zeroOffset;
    }
}