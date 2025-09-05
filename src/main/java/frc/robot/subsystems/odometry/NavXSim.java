package frc.robot.subsystems.odometry;

import edu.wpi.first.math.geometry.Rotation2d;

public class NavXSim {
    private static NavXSim instance;
    private double yawRadians = 0.0;

    private NavXSim() {
    }

    public static NavXSim getInstance() {
        if (instance == null) {
            instance = new NavXSim();
        }
        return instance;
    }

    public void reset() {
        yawRadians = 0.0;
    }

    public Rotation2d getRotation2d() {
        return new Rotation2d(yawRadians);
    }

    /**
     * Integrate chassis angular velocity (rad/s) over dt seconds
     */
    public void update(double omegaRadiansPerSec, double dt) {
        yawRadians += omegaRadiansPerSec * dt;
    }

    public boolean isMoving() {
        // Assuming isMoving checks if yawRadians is changing
        // TODO: Implement
        return yawRadians != 0.0;
    }
}
