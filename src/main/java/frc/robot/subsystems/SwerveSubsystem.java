package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.helpers.SwerveHelper;

public class SwerveSubsystem extends SubsystemBase {
    public static final double MAX_SPEED = 0.3; // meters per second
    public static final double MAX_ANGULAR_SPEED = Math.PI / 2.0; // radians per second
    private static final double TRACK_WIDTH = 0.62; // meters TODO update value
    private static final double WHEEL_BASE = 0.62; // meters TODO update value

    private final SwerveModule frontLeft = new SwerveModule("FrontLeft",
            2, 1, false, true, 9);
    private final SwerveModule frontRight = new SwerveModule("FrontRight",
            4, 3, true, true, 10);
    private final SwerveModule backLeft = new SwerveModule("BackLeft",
            5, 6, true, true, 11);
    private final SwerveModule backRight = new SwerveModule("BackRight",
            7, 8, false, true, 12);
    private final AHRS gyro = new AHRS(SPI.Port.kMXP);
    private final SwerveDriveKinematics kinematics = SwerveHelper.constructKinematics(TRACK_WIDTH, WHEEL_BASE);


    public SwerveSubsystem() {
        this.gyro.reset();
    }

    public void drive(double xSpeed, double ySpeed, double rotation, boolean fieldRelative) {
        SwerveModuleState[] swerveModuleStates = this.kinematics.toSwerveModuleStates(fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rotation, this.gyro.getRotation2d())
                : new ChassisSpeeds(xSpeed, ySpeed, rotation));

        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, MAX_SPEED);

        this.frontLeft.setDesiredState(swerveModuleStates[SwerveHelper.ModuleIds.FRONT_LEFT.get()]);
        this.frontRight.setDesiredState(swerveModuleStates[SwerveHelper.ModuleIds.FRONT_RIGHT.get()]);
        this.backLeft.setDesiredState(swerveModuleStates[SwerveHelper.ModuleIds.BACK_LEFT.get()]);
        this.backRight.setDesiredState(swerveModuleStates[SwerveHelper.ModuleIds.BACK_RIGHT.get()]);
    }
}
