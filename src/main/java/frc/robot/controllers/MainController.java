package frc.robot.controllers;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.helpers.IDashboardProvider;
import frc.robot.subsystems.SwerveSubsystem;

public class MainController extends XboxController implements IDashboardProvider {
    public static final int COMPUTER_PORT = 0;
    private static final double DEADBAND = 0.05;
    private final SlewRateLimiter xSpeedLimiter = new SlewRateLimiter(3.0);
    private final SlewRateLimiter ySpeedLimiter = new SlewRateLimiter(3.0);
    private final SlewRateLimiter rotateLimiter = new SlewRateLimiter(3.0);

    public MainController(final int port) {
        super(port);
        this.registerDashboard();
    }

    public double getRobotXSpeed() {
        double rawValue = MathUtil.applyDeadband(-this.getLeftY(), DEADBAND);
        return this.xSpeedLimiter.calculate(rawValue) * SwerveSubsystem.MAX_SPEED;
    }

    public double getRobotYSpeed() {
        double rawValue = MathUtil.applyDeadband(-this.getLeftX(), DEADBAND);
        return this.ySpeedLimiter.calculate(rawValue) * SwerveSubsystem.MAX_SPEED;
    }

    public double getRobotRotation() {
        double rawValue = MathUtil.applyDeadband(-this.getRightX(), DEADBAND);
        return this.rotateLimiter.calculate(rawValue) * SwerveSubsystem.MAX_ANGULAR_SPEED;
    }

    @Override
    public void putDashboard() {
        SmartDashboard.putNumber("RobotXSpeed", this.getRobotXSpeed());
        SmartDashboard.putNumber("RobotYSpeed", this.getRobotYSpeed());
        SmartDashboard.putNumber("RobotRotateSpeed", this.getRobotRotation());
    }
}
