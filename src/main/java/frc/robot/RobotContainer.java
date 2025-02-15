// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.DriveCommands;
import frc.robot.constants.Camera;
import frc.robot.constants.Constants;
import frc.robot.constants.SweveConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.imu.GyroIO;
import frc.robot.subsystems.drive.imu.GyroIOPigeon2;
import frc.robot.subsystems.drive.module.ModuleIO;
import frc.robot.subsystems.drive.module.ModuleIOSim;
import frc.robot.subsystems.drive.module.ModuleIOTalonFX;
import frc.robot.subsystems.vision.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  public Drive drive;
  public Vision vision;
  // public  Elevator elevator;
  // private final AutoFactory autoFactory;

  // Controller
  private final CommandXboxController controller1 = new CommandXboxController(0);
  private final CommandXboxController controller2 = new CommandXboxController(1);

  // Dashboard inputs
  // private final LoggedDashboardChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOTalonFX(SweveConstants.FrontLeft),
                new ModuleIOTalonFX(SweveConstants.FrontRight),
                new ModuleIOTalonFX(SweveConstants.BackLeft),
                new ModuleIOTalonFX(SweveConstants.BackRight));
        vision =
        new Vision(
            drive::addVisionMeasurement,
            new VisionIOPhotonVision(Camera.BackCamera.name, Camera.BackCamera.robotToCam)
                 /* ,new VisionIOPhotonVision(
                Camera.FrontRightCamera.name, Camera.BackCamera.robotToCam),
            new VisionIOPhotonVision(
                Camera.FrontLeftCamera.name, Camera.BackCamera.robotToCam)*/

            );
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(SweveConstants.FrontLeft),
                new ModuleIOSim(SweveConstants.FrontRight),
                new ModuleIOSim(SweveConstants.BackLeft),
                new ModuleIOSim(SweveConstants.BackRight));

        vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIOPhotonVisionSim(
                    Camera.BackCamera.name, Camera.BackCamera.robotToCam, drive::getPose),
                new VisionIOPhotonVisionSim(
                    Camera.FrontRightCamera.name, Camera.BackCamera.robotToCam, drive::getPose),
                new VisionIOPhotonVisionSim(
                    Camera.FrontLeftCamera.name, Camera.BackCamera.robotToCam, drive::getPose));

        break;

      default:
        // Replayed robot, disable IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});

        vision = new Vision(drive::addVisionMeasurement, new VisionIO() {}, new VisionIO() {});

        break;
    }

    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Default command, normal field-relative drive
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -controller1.getLeftY(),
            () -> -controller1.getLeftX(),
            () -> -controller1.getRightX()));

    // Lock to 0° when A button is held
    controller1
        .a()
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                drive,
                () -> -controller1.getLeftY(),
                () -> -controller1.getLeftX(),
                () -> new Rotation2d()));

    // Switch to X pattern when X button is pressed
    controller1.x().onTrue(Commands.runOnce(drive::stopWithX, drive));

    // Reset gyro to 0° when B button is pressed
    controller1
        .b()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), new Rotation2d())),
                    drive)
                .ignoringDisable(true));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return null;
  }
}
