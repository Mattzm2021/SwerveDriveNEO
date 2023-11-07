package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.controllers.MainController;
import frc.robot.subsystems.SwerveSubsystem;

public class RobotContainer {
    private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
    private final MainController mainController = new MainController(MainController.COMPUTER_PORT);


    public RobotContainer() {
        this.swerveSubsystem.setDefaultCommand(Commands.run(this::driveRobot, this.swerveSubsystem));
    }

    private void driveRobot() {
        final double xSpeed = this.mainController.getRobotXSpeed();
        final double ySpeed = this.mainController.getRobotYSpeed();
        final double rotation = this.mainController.getRobotRotation();
        this.swerveSubsystem.drive(xSpeed, ySpeed, rotation, false);
    }

    public Command getAutonomousCommand() {
        // TODO: Implement properly
        return null;
    }
}
