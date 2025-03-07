package frc.robot.libs;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorOffsetDutyCycleEncoder extends SubsystemBase {

    private final double zeroOffset;
    private final DutyCycleEncoder encoder;

    private double lastreading = 0;
    private int rotations = 0;

    public ElevatorOffsetDutyCycleEncoder(int analogID, boolean inverted) {
        encoder = new DutyCycleEncoder(analogID);

        encoder.setDutyCycleRange(0, 1);

        encoder.setInverted(inverted);

        this.zeroOffset = encoder.get();
    }

    @Override
    public void periodic() {
        double currentReading = encoder.get();
        rotations += (currentReading < 0.1 && lastreading > 0.9) ? 1 : 0;
        rotations -= (currentReading > 0.9 && lastreading < 0.1) ? 1 : 0;
        lastreading = currentReading;
    }

    public double getAbsolutePosition() {
        return encoder.get() - zeroOffset + rotations;
    }
}