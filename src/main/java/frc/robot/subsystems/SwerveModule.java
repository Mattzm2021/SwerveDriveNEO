package frc.robot.subsystems;

import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.helpers.IDashboardProvider;

public class SwerveModule implements IDashboardProvider {
    private static final double WHEEL_RADIUS = 0.0508; // meters // 0.3192
    private static final double DRIVE_GEAR_RATIO = 57.0 / 7.0;
    private static final double MAX_ANGULAR_VELOCITY = SwerveSubsystem.MAX_ANGULAR_SPEED;
    private static final double MAX_ANGULAR_ACCELERATION = 2.0 * Math.PI; // radians per second squared

    private final CANSparkMax driveMotor;
    private final CANSparkMax turningMotor;
    private final RelativeEncoder driveEncoder;
    private final CANCoder turningEncoder;
    private final PIDController drivePIDController = new PIDController(1.0, 0.0, 0.0);
    private final ProfiledPIDController turningPIDController = new ProfiledPIDController(1.0, 0.0, 0.0,
            new TrapezoidProfile.Constraints(MAX_ANGULAR_VELOCITY, MAX_ANGULAR_ACCELERATION));
    private final SimpleMotorFeedforward driveFeedforward = new SimpleMotorFeedforward(1.0, 3.0);
    private final SimpleMotorFeedforward turnFeedforward = new SimpleMotorFeedforward(1.0, 0.5);
    private final String name;

    public SwerveModule(String name, int driveMotorChannel, int turningMotorChannel,
                        boolean driveMotorInverted, boolean turningMotorInverted, int absoluteEncoderId) {
        this.registerDashboard();
        this.name = name;

        this.driveMotor = new CANSparkMax(driveMotorChannel, CANSparkMaxLowLevel.MotorType.kBrushless);
        this.driveMotor.setSmartCurrentLimit(30);
        this.driveMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
        this.driveMotor.setInverted(driveMotorInverted);

        this.turningMotor = new CANSparkMax(turningMotorChannel, CANSparkMaxLowLevel.MotorType.kBrushless);
        this.turningMotor.setSmartCurrentLimit(30);
        this.turningMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
        this.turningMotor.setInverted(turningMotorInverted);

        this.driveEncoder = this.driveMotor.getEncoder();
        this.driveEncoder.setPositionConversionFactor(2.0 * Math.PI * WHEEL_RADIUS / DRIVE_GEAR_RATIO);
        this.driveEncoder.setVelocityConversionFactor(2.0 * Math.PI * WHEEL_RADIUS / DRIVE_GEAR_RATIO / 60.0);

        this.turningEncoder = new CANCoder(absoluteEncoderId);
        this.turningPIDController.enableContinuousInput(-Math.PI, Math.PI);
    }

    @SuppressWarnings("unused")
    public SwerveModuleState getState() {
        return new SwerveModuleState(this.driveEncoder.getVelocity(),
                new Rotation2d(this.turningEncoder.getPosition()));
    }

    public void setDesiredState(SwerveModuleState desiredState) {
        SwerveModuleState state = SwerveModuleState.optimize(desiredState,
                new Rotation2d(this.turningEncoder.getPosition()));

        final double driveOutput = this.drivePIDController.calculate(
                this.driveEncoder.getVelocity(), state.speedMetersPerSecond);

        final double driveFeedforward = this.driveFeedforward.calculate(state.speedMetersPerSecond);

        final double turnOutput = this.turningPIDController.calculate(
                this.turningEncoder.getPosition(), state.angle.getRadians());

        final double turnFeedforward = this.turnFeedforward.calculate(
                this.turningPIDController.getSetpoint().velocity);

        this.driveMotor.setVoltage(driveOutput + driveFeedforward);
        this.turningMotor.setVoltage(turnOutput + turnFeedforward);
    }

    @Override
    public void putDashboard() {
        SmartDashboard.putNumber(this.name + " DrivePosition", this.driveEncoder.getPosition());
        SmartDashboard.putNumber(this.name + " DriveVelocity", this.driveEncoder.getVelocity());
        SmartDashboard.putNumber(this.name + " TurnPosition", this.turningEncoder.getPosition());
        SmartDashboard.putNumber(this.name + " TurnVelocity", this.turningEncoder.getVelocity());
    }
}
