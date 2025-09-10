package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.sim.SparkAbsoluteEncoderSim;
import com.revrobotics.sim.SparkFlexSim;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.sim.SparkRelativeEncoderSim;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.libs.AbsoluteEncoder;

public class SwerveModule extends SubsystemBase {
    public String moduleID;
    public int driveMotorID;
    public int turnMotorID;
    public double baseAngle;
    public SparkMax turnMotor;
    public SparkMaxSim simTurnMotor;
    public SparkFlex driveMotor;
    public SparkFlexSim simDriveMotor;
    public PIDController turnPID;
    public ProfiledPIDController drivePID;
    public AbsoluteEncoder turnEncoder;
    public SparkAbsoluteEncoderSim simTurnEncoder;
    public RelativeEncoder driveEncoder;
    public SparkRelativeEncoderSim simDriveEncoder;

    public double botMass = 24.4;

    public double turnP = .01;

    public double driveSetpointTolerance = .5;
    public double turnSetpointTolerance = 5;
    public double turnVelocityTolerance = 1;

    private SimpleMotorFeedforward driveFeedforward = new SimpleMotorFeedforward(0.084706 * .712, 2.4433 * .712,
            0.10133 * .712);
    // realised the feedforward was off by a factor of .712, corrected it
    private TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(Constants.Bot.maxChassisSpeed,
            Constants.Bot.maxAcceleration);

    // private State initialState = new TrapezoidProfile.State(0, 0);
    // private TrapezoidProfile trapezoidProfile;

    // Conversion Factor for the motor encoder output to wheel output
    // (Circumference / Gear Ratio) * Inches to meters conversion

    public SwerveModule(String moduleID, int analogID, int driveMotorID, int turnMotorID, double baseAngle,
            boolean driveInverted) {
        this.moduleID = moduleID;
        this.baseAngle = baseAngle;
        this.turnMotorID = turnMotorID;
        this.driveMotorID = driveMotorID;

        SparkMaxConfig motorConfig = new SparkMaxConfig();

        motorConfig.idleMode(IdleMode.kBrake).inverted(driveInverted).smartCurrentLimit(40);

        driveMotor = new SparkFlex(driveMotorID, MotorType.kBrushless);

        driveMotor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        motorConfig.idleMode(IdleMode.kBrake).inverted(false).smartCurrentLimit(30);

        turnMotor = new SparkMax(turnMotorID, MotorType.kBrushless);

        turnMotor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        turnEncoder = new AbsoluteEncoder(analogID, baseAngle);

        driveEncoder = driveMotor.getEncoder();

        turnPID = new PIDController(turnP, 0, 0);

        // we don't use I or D since P works well enough
        turnPID.enableContinuousInput(0, 360);
        turnPID.setTolerance(turnSetpointTolerance, turnVelocityTolerance);

        // determined from a SYSID scan
        drivePID = new ProfiledPIDController(0.005, 0, 0.0005, constraints);
        drivePID.setConstraints(constraints);
        drivePID.setTolerance(driveSetpointTolerance);

        if (RobotBase.isSimulation()) {
            simDriveEncoder = new SparkRelativeEncoderSim(driveMotor);
            simDriveMotor = new SparkFlexSim(driveMotor, DCMotor.getNeoVortex(1));

            simTurnEncoder = new SparkAbsoluteEncoderSim(turnMotor);
            simTurnMotor = new SparkMaxSim(turnMotor, DCMotor.getNEO(1));
        }
    }

    // scales the maximum acceleration of the robot based on the height of the
    // elevator to prevent toppling
    private double CalculateAccelScale(double elevHeight) {
        double minHeight = Constants.ElevatorHeights.minimum;
        double maxHeight = Constants.ElevatorHeights.maximum;
        double minAccelScale = 0.35;
        double height = Math.min(Math.max(elevHeight, minHeight), maxHeight);
        double mappedHeight = (height - minHeight) / (maxHeight - minHeight); // the height remapped from 0-1 (min to
                                                                              // max)
        double scale = 1 - (mappedHeight) * (1 - minAccelScale);
        return scale;
    }

    // runs while the bot is running
    @Override
    public void periodic() {
        // setAngle(0);
        // turnPID.calculate(getTurnEncoder().getAbsolutePosition());
        if (Elevator.getInstance().isMoving()) {
            double scale = CalculateAccelScale(Elevator.getInstance().getHeight());
            accelerationLimiter = new SlewRateLimiter(Constants.Bot.maxAcceleration * scale,
                    -Constants.Bot.maxAcceleration * scale, 0);
            constraints = new TrapezoidProfile.Constraints(Constants.Bot.maxChassisSpeed,
                    Constants.Bot.maxAcceleration * scale);
            drivePID.setConstraints(constraints);
        }
        NetworkTableInstance.getDefault().getTable("Angle").getEntry(moduleID)
                .setDouble(turnEncoder.getAbsolutePosition());
    }

    SlewRateLimiter accelerationLimiter = new SlewRateLimiter(Constants.Bot.maxAcceleration,
            -Constants.Bot.maxAcceleration, 0);

    public void setStates(SwerveModuleState state) {
        double currentAngle = turnEncoder.getAbsolutePosition();

        NetworkTableInstance.getDefault().getTable("ModuleDebug").getEntry(moduleID + "/CurrentAngle")
                .setDouble(currentAngle);
        NetworkTableInstance.getDefault().getTable("ModuleDebug").getEntry(moduleID + "/DesiredAngleBeforeOptimize")
                .setDouble(state.angle.getDegrees());
        NetworkTableInstance.getDefault().getTable("ModuleDebug").getEntry(moduleID + "/DesiredSpeedBeforeOptimize")
                .setDouble(state.speedMetersPerSecond);

        double angle;
        if (RobotBase.isSimulation()) {
            angle = (currentAngle + 180) % 360;
            if (angle < 0) {
                angle += 360;
            }
        } else {
            angle = currentAngle;
        }

        state.optimize(new Rotation2d(Units.degreesToRadians(angle)));

        NetworkTableInstance.getDefault().getTable("ModuleDebug").getEntry(moduleID + "/DesiredAngleAfterOptimize")
                .setDouble(state.angle.getDegrees());
        NetworkTableInstance.getDefault().getTable("ModuleDebug").getEntry(moduleID + "/DesiredSpeedAfterOptimize")
                .setDouble(state.speedMetersPerSecond);

        setAngle(state.angle.getDegrees());
        setDriveSpeed(accelerationLimiter.calculate(state.speedMetersPerSecond));

        NetworkTableInstance.getDefault().getTable("Speed").getEntry(moduleID).setDouble(state.speedMetersPerSecond);

        if (RobotBase.isSimulation()) {
            NetworkTableInstance.getDefault().getTable("ModuleDebug").getEntry(moduleID + "/TurnMotorPos")
                    .setDouble(simTurnMotor.getPosition());
            NetworkTableInstance.getDefault().getTable("ModuleDebug").getEntry(moduleID + "/DriveMotorRPM")
                    .setDouble(simDriveMotor.getVelocity());
        }
    }

    public void setAngle(double angle) {
        turnPID.setSetpoint(angle);
        turnMotor.set(-turnPID.calculate(turnEncoder.getAbsolutePosition()));
    }

    public void setDriveSpeed(double velocity) {
        // velocity is desired wheel speed in meters/second
        drivePID.setGoal(new State(velocity, 0));

        // driveEncoder.getVelocity() returns RPM -> convert to meters/sec:
        // measuredVelocity = (RPM / 60) * metersPerWheelRotation
        double measuredVelocity = driveEncoder.getVelocity() * Constants.Bot.encoderRotationToMeters / 60.0;

        // Feedforward (expects m/s) + PID (measurement in m/s)
        double voltage = driveFeedforward.calculate(velocity) + drivePID.calculate(measuredVelocity);

        driveMotor.setVoltage(voltage);

        NetworkTableInstance.getDefault().getTable(moduleID).getEntry("Set Speed").setDouble(velocity);
        NetworkTableInstance.getDefault().getTable(moduleID).getEntry("Actual Speed").setDouble(measuredVelocity);
    }

    public void setTurnSpeed(double speed) {
        speed = Math.max(Math.min(speed, Constants.Bot.maxTurnSpeed), -Constants.Bot.maxTurnSpeed);
        turnMotor.set(speed);
    }

    public SwerveModulePosition getSwerveModulePosition() {
        double angle = turnEncoder.getAbsolutePosition();
        // driveEncoder.getPosition() is in motor rotations -> encoderRotationToMeters
        // is meters per wheel rotation.
        // If encoderRotationToMeters is meters per wheel rotation, ensure
        // encoder.getPosition() is wheel rotations
        // (if not, make sure your constant matches motor->wheel ratio). This keeps
        // behavior identical to real robot.
        double distance = driveEncoder.getPosition() * Constants.Bot.encoderRotationToMeters;
        return new SwerveModulePosition(distance, new Rotation2d(Units.degreesToRadians(angle)));
    }

    public double getVelocity() {
        // Use driveEncoder (RPM) -> convert to meters/sec
        double measuredVelocity = driveEncoder.getVelocity() * Constants.Bot.encoderRotationToMeters / 60.0;
        return measuredVelocity;
    }

    public AbsoluteEncoder getTurnEncoder() {
        return this.turnEncoder;
    }

    public String getModuleID() {
        return this.moduleID;
    }

    public SwerveModuleState getState() {
        // Return module state with wheel speed in meters/sec and module angle in
        // radians
        double speedMetersPerSecond = driveEncoder.getVelocity() * Constants.Bot.encoderRotationToMeters / 60.0;
        return new SwerveModuleState(speedMetersPerSecond,
                Rotation2d.fromDegrees(turnEncoder.getAbsolutePosition()));
    }
}