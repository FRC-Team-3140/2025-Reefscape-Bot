package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
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
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.libs.AbsoluteEncoder;

public class SwerveModule extends SubsystemBase {
    public String moduleID;
    public int pwmID;
    public int driveMotorID;
    public int turnMotorID;
    public double baseAngle;
    public SparkMax turnMotor;
    public SparkFlex driveMotor;
    public PIDController turnPID;
    public ProfiledPIDController drivePID;
    public AbsoluteEncoder turnEncoder;
    public RelativeEncoder driveEncoder;

    public double botMass = 24.4;

    public double turnP = .01;

    public double driveSetpointTolerance = .5;
    public double turnSetpointTolerance;
    public double turnVelocityTolerance;

    private SimpleMotorFeedforward driveFeedforward = new SimpleMotorFeedforward(0.084706 * .712, 2.4433 * .712,
            0.10133 * .712);
    // realised the feedforward was off by a factor of .712, corrected it
    private TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(Constants.Bot.maxChassisSpeed,
            Constants.Bot.maxAcceleration);

    // private State initialState = new TrapezoidProfile.State(0, 0);
    // private TrapezoidProfile trapezoidProfile;

    // Conversion Factor for the motor encoder output to wheel output
    // (Circumference / Gear Ratio) * Inches to meters conversion

    public SwerveModule(String moduleID, int analogID, int driveMotorID, int turnMotorID, double baseAngle) {
        this.moduleID = moduleID;
        this.baseAngle = baseAngle;
        this.turnMotorID = turnMotorID;
        this.driveMotorID = driveMotorID;

        SparkMaxConfig motorConfig = new SparkMaxConfig();

        motorConfig.idleMode(IdleMode.kBrake).inverted(false).smartCurrentLimit(40);

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
        drivePID.setTolerance(driveSetpointTolerance);
    }
    
    // runs while the bot is running
    @Override
    public void periodic() {
        setAngle(0);
        turnPID.calculate(getTurnEncoder().getAbsolutePosition());
        NetworkTableInstance.getDefault().getTable("Angle").getEntry(moduleID)
                .setDouble(turnEncoder.getAbsolutePosition());
    }

    SlewRateLimiter accelerationLimiter = new SlewRateLimiter(30.0, -Constants.Bot.maxAcceleration, 0);

    public void setStates(SwerveModuleState state, boolean locked) {
        state.optimize(Rotation2d.fromDegrees(turnEncoder.getAbsolutePosition()));
        setAngle(state.angle.getDegrees());
        setDriveSpeed(accelerationLimiter.calculate(state.speedMetersPerSecond));
        NetworkTableInstance.getDefault().getTable("Speed").getEntry(moduleID).setDouble(state.speedMetersPerSecond);
    }

    public void setAngle(double angle) {
        turnPID.setSetpoint(angle);
        turnMotor.set(-turnPID.calculate(turnEncoder.getAbsolutePosition()));
    }

    public void setDriveSpeed(double velocity) {
        drivePID.setGoal(new State(velocity, 0));
        driveMotor.setVoltage(driveFeedforward.calculate(velocity)
                + drivePID.calculate(driveEncoder.getVelocity() * Constants.Bot.encoderRotationToMeters));
        NetworkTableInstance.getDefault().getTable(moduleID).getEntry("Set Speed").setDouble(velocity);
        NetworkTableInstance.getDefault().getTable(moduleID).getEntry("Actual Speed")
                .setDouble(driveEncoder.getVelocity() * Constants.Bot.encoderRotationToMeters);
        // drivePID.calculate(driveEncoder.getVelocity()*Constants.Bot.encoderRotationToMeters));
        // ///drivePID added too much
        // instability
    }

    public void setTurnSpeed(double speed) {
        speed = Math.max(Math.min(speed, Constants.Bot.maxTurnSpeed), -Constants.Bot.maxTurnSpeed);
        turnMotor.set(speed);
    }

    public SwerveModulePosition getSwerveModulePosition() {
        double angle = turnEncoder.getAbsolutePosition();
        double distance = driveEncoder.getPosition() * Constants.Bot.encoderRotationToMeters;
        return new SwerveModulePosition(distance, new Rotation2d(3.14 * angle / 180));
    }

    // public RelativeEncoder getDriveEncoder() {
    // return this.driveEncoder;
    // }
    // ^ Dangerous since values need to be manually multiplied by
    // Constants.Bot.encoderRotationToMeters

    public AbsoluteEncoder getTurnEncoder() {
        return this.turnEncoder;
    }

    public String getModuleID() {
        return this.moduleID;
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(driveEncoder.getVelocity() * Constants.Bot.encoderRotationToMeters,
                Rotation2d.fromDegrees(turnEncoder.getAbsolutePosition()));
    }
}