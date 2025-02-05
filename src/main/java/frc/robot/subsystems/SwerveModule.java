package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

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
import frc.robot.RobotContainer;

public class SwerveModule extends SubsystemBase {

  // Zero : 0.697578
  // One : 0.701239
  // Two: 0.467096
  // Three : 0.207867
  public String moduleID;
  public int pwmID;
  public int driveMotorID;
  public int turnMotorID;
  public SparkMax turnMotor;
  public SparkFlex driveMotor;
  public PIDController turnPID;
  public ProfiledPIDController drivePID;
  public CANcoder turnEncoder;
  public RelativeEncoder driveEncoder;

  public double botMass = 24.4;

  public double turnP = .001;

  public double driveSetpointTolerance = .5;
  public double turnSetpointTolerance = 10;
  public double turnVelocityTolerance = .1;
  // ks: voltage the motors start to move at
  // kv: velocity / voltage
  // ka: velocity / (voltage^2) 
  private SimpleMotorFeedforward driveFeedforward = new SimpleMotorFeedforward(0.1, 0.45, 0.075);
      
  private TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(Constants.Bot.maxChassisSpeed,
      Constants.Bot.maxAcceleration);

  // private State initialState = new TrapezoidProfile.State(0, 0);
  // private TrapezoidProfile trapezoidProfile;

  // Conversion Factor for the motor encoder output to wheel output
  // (Circumference / Gear Ratio) * Inches to meters conversion

  public SwerveModule(String moduleID, int analogID, int driveMotorID, int turnMotorID) {
    this.moduleID = moduleID;
    this.turnMotorID = turnMotorID;
    this.driveMotorID = driveMotorID;

    SparkMaxConfig motorConfig = new SparkMaxConfig();

    motorConfig.idleMode(IdleMode.kBrake).inverted(false).smartCurrentLimit(40);

    driveMotor = new SparkFlex(driveMotorID, MotorType.kBrushless);
    
    driveMotor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    motorConfig.idleMode(IdleMode.kBrake).inverted(false).smartCurrentLimit(30);

    turnMotor = new SparkMax(turnMotorID, MotorType.kBrushless);
    
    turnMotor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    turnEncoder = new CANcoder(analogID);
    // TO CALIBRATE ENCODERS: Uncomment this line, put the wheels 
    // turnEncoder.setPosition(baseAngle);
    
    driveEncoder = driveMotor.getEncoder();

    turnPID = new PIDController(turnP, 0, 0);
    // we don't use I or D since P works well enough
    turnPID.enableContinuousInput(0, 360);
    turnPID.setTolerance(turnSetpointTolerance, turnVelocityTolerance);
    // determined from a SYSID scan
    drivePID = new ProfiledPIDController(.1, 0, 0, constraints);
    drivePID.setTolerance(driveSetpointTolerance);

    setAngle(0);
  }

  // runs while the bot is running
  @Override
  public void periodic() {
    // turnMotor.set(-turnPID.calculate(getTurnEncoderAngle()));
    //driveMotor.setVoltage(driveFeedforward.calculate(driveEncoder.getVelocity()));
    driveMotor.setVoltage(Math.max(RobotContainer.m_driverController.getLeftX()*12, -RobotContainer.m_driverController.getLeftY()*12));
    NetworkTableInstance.getDefault().getTable(moduleID).getEntry("Raw Angle").setDouble(turnEncoder.getAbsolutePosition().getValueAsDouble());
    NetworkTableInstance.getDefault().getTable(moduleID).getEntry("Actual Angle").setDouble(getTurnEncoderAngle());
    NetworkTableInstance.getDefault().getTable(moduleID).getEntry("Angle Setpoint").setDouble(turnPID.getSetpoint());

    NetworkTableInstance.getDefault().getTable(moduleID).getEntry("Actual Velo").setDouble(getDriveVelo() / 60);
    NetworkTableInstance.getDefault().getTable(moduleID).getEntry("Actual Volt").setDouble(RobotContainer.m_driverController.getLeftX()*12);
  }

  SlewRateLimiter accelerationLimiter = new SlewRateLimiter(30.0, -Constants.Bot.maxAcceleration, 0);

  public void setStates(SwerveModuleState state, boolean locked) {
    //state.optimize(Rotation2d.fromDegrees(getTurnEncoderAngle()));
    setAngle(state.angle.getDegrees());
    setDriveSpeed(accelerationLimiter.calculate(state.speedMetersPerSecond));
    NetworkTableInstance.getDefault().getTable("Speed").getEntry(moduleID).setDouble(state.speedMetersPerSecond);
  }

  public void setAngle(double angle) {
    turnPID.setSetpoint(angle);
    turnMotor.set(-turnPID.calculate(turnEncoder.getAbsolutePosition().getValueAsDouble()));
  }

  public void setDriveSpeed(double velocity) {
    drivePID.setGoal(new State(velocity*Constants.Bot.maxChassisSpeed, 0));
    NetworkTableInstance.getDefault().getTable(moduleID).getEntry("Set Speed").setDouble(velocity);
    NetworkTableInstance.getDefault().getTable(moduleID).getEntry("Actual Speed")
        .setDouble(getDriveVelo());
    // drivePID.calculate(getDriveVelo())); ///drivePID added too much
    // instability
  }

  // public void setTurnSpeed(double speed) {
  //   speed = Math.max(Math.min(speed, Constants.Bot.maxTurnSpeed), -Constants.Bot.maxTurnSpeed);
  //   turnMotor.set(speed);
  // }

  public SwerveModulePosition getSwerveModulePosition() {
    double angle = getTurnEncoderAngle();
    double distance = driveEncoder.getPosition()*Constants.Bot.encoderRotationToMeters;
    return new SwerveModulePosition(distance, new Rotation2d(3.14 * angle / 180));
  }

  // public RelativeEncoder getDriveEncoder() {
  //   return this.driveEncoder;
  // }
  // ^ Dangerous since values need to be manually multiplied by Constants.Bot.encoderRotationToMeters

  public double getDriveVelo() {
    double readVelo = driveMotor.getEncoder().getVelocity();

    if (Math.abs(readVelo) <= Constants.Limits.DriveVeloEncoderThreshold) {
      return 0;
    }

    return readVelo*Constants.Bot.encoderRotationToMeters;
  }

  public CANcoder getTurnEncoder() {
    return this.turnEncoder;
  }

  public double getTurnEncoderAngle() {
    double revolutions = turnEncoder.getAbsolutePosition().getValueAsDouble();
    int sign = revolutions < 0 ? -1 : 1;
    double encoderAngle = (Math.abs(revolutions * 360) % 360 - 180) * sign;

    return encoderAngle;
  }

  public String getModuleID() {
    return this.moduleID;
  }

  public SwerveModuleState getState() {
    return new SwerveModuleState(getDriveVelo(),
        Rotation2d.fromDegrees(getTurnEncoderAngle()));
  }
}