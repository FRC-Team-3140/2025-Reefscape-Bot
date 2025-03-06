package frc.robot.libs;

import edu.wpi.first.wpilibj.DutyCycleEncoder;

public class OffsetDutyCycleEncoder extends DutyCycleEncoder {

    private final double zeroOffset;

    public OffsetDutyCycleEncoder(int analogID, double zeroOffset) {
        // Initializes an encoder on a DIO port and uses 360 as full range and 180 as
        // half of the range
        super(analogID);

        this.zeroOffset = zeroOffset;
    }

    public double getAbsolutePosition() {
        return super.get() - zeroOffset;
    }
}