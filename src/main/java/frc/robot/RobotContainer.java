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

import static frc.robot.DriveCommands.*;
import static frc.robot.constants.VisionConstants.*;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.constants.Constants;
import frc.robot.constants.EndEffectorConstants;
import frc.robot.constants.SwerveConstants;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.climber.ClimberIOReal;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.imu.GyroIO;
import frc.robot.subsystems.drive.imu.GyroIOPigeon2;
import frc.robot.subsystems.drive.module.ModuleIO;
import frc.robot.subsystems.drive.module.ModuleIOTalonFX;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorIOKraken;
import frc.robot.subsystems.endEffector.EndEffector;
import frc.robot.subsystems.endEffector.EndEffectorIOReal;
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
  public Elevator elevator;
  // public Intake intake;
  public Climber climber;
  public EndEffector endEffector;
  public AutoFactory autoFactory;
  public AutoChooser autoChooser;

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
                new ModuleIOTalonFX(SwerveConstants.FrontLeft),
                new ModuleIOTalonFX(SwerveConstants.FrontRight),
                new ModuleIOTalonFX(SwerveConstants.BackLeft),
                new ModuleIOTalonFX(SwerveConstants.BackRight));
        vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIOPhotonVision(camera0Name, robotToCamera0),
                new VisionIOPhotonVision(camera1Name, robotToCamera1)
                /*ew VisionIOPhotonVision(
                Camera.FrontLeftCamera.name, Camera.BackCamera.robotToCam)*/

                );
        elevator = new Elevator(new ElevatorIOKraken());
        // intake = new Intake(new IntakeIOReal());
        climber = new Climber(new ClimberIOReal());
        endEffector = new EndEffector(new EndEffectorIOReal());
        autoFactory =
            new AutoFactory(drive::getPose, drive::setPose, drive::followTrajectory, true, drive);
        autoChooser = new AutoChooser();

        autoChooser.addRoutine("L3 MAIN ONE!!! ", this::L3Main);
        autoChooser.addRoutine("L2 MAIN ONE!!!", this::L2Main);
        autoChooser.addRoutine("L4 Cycle", this::L4Cycle);
        autoChooser.addRoutine("L4 MAIN ONE!!!", this::L4Main);

        autoChooser.addRoutine("L2 Cycle", this::L2Cycle);

        SmartDashboard.putData("AutoChooser", autoChooser);
        RobotModeTriggers.autonomous().whileTrue(autoChooser.selectedCommandScheduler());
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementationss

        vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIOPhotonVisionSim(camera0Name, robotToCamera0, drive::getPose)
                //    ,new VisionIOPhotonVisionSim(camera1Name, robotToCamera1, drive::getPose)

                );

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

  /*private void restartRoboToCam(){
      vision =
      new Vision(
          drive::addVisionMeasurement,
          new VisionIOPhotonVisionSim(camera0Name, robotToCamera0, Pose2d())
          //    ,new VisionIOPhotonVisionSim(camera1Name, robotToCamera1, drive::getPose)

          );
  }*/

  private void configureButtonBindings() {

    // Default command, normal field-relative drive
    drive.setDefaultCommand(
        joystickDrive(
            drive,
            () -> -controller1.getLeftY() * elevator.getDriveWeight(),
            () -> -controller1.getLeftX() * elevator.getDriveWeight(),
            () -> -controller1.getRightX() * elevator.getDriveWeight()));

    /*   climber.setDefaultCommand(
    new RunCommand(
        () -> {
          climber.setSpeed(controller2.getLeftY());
        },
        climber));*/

    // Intake
    controller2
        .leftTrigger()
        .onTrue(
            elevator
                .goToCoralPickup()
                .andThen(
                    endEffector
                        .setShooter(EndEffectorConstants.IntakeSpeed)
                        .andThen(endEffector.RotateCoralPickup())))
        .onFalse(endEffector.setShooter(0));

    // Place Coral
    controller2
        .rightTrigger()
        .onTrue(endEffector.setShooter(EndEffectorConstants.PlacementSpeed))
        .onFalse(endEffector.setShooter(0));

    // Climber Climb
    controller1.leftTrigger().whileTrue(climber.setSpeed(1.0)).onFalse(climber.setSpeed(0));

    // Climber UnClimb
    controller1.rightTrigger().whileTrue(climber.setSpeed(-1.0)).onFalse(climber.setSpeed(0));

    // ----------------------------------------------------------------------------------------------------------------------------------------------------
    // Level One Elevator
    controller2.a().onTrue(elevator.goToGroundLevel().andThen(endEffector.RotateCoralPickup()));

    // Level One Elevator auto align left
    controller2
        .a()
        .and(controller2.leftBumper())
        .whileTrue(
            elevator
                .goToGroundLevel()
                .alongWith(endEffector.RotateCoralPickup())
                .alongWith(drive.alignToReef(1))
                .andThen(Commands.waitUntil(elevator::atTargetPosition))
                .andThen(
                    endEffector.setShooterTimed(
                        EndEffectorConstants.PlacementSpeed, EndEffectorConstants.PlacementTime))
                .andThen(endEffector.RotateCoralPickup())
                .andThen(elevator.goToCoralPickup()));

    // Level One Elevator auto align right
    controller2
        .a()
        .and(controller2.rightBumper())
        .whileTrue(
            elevator
                .goToGroundLevel()
                .alongWith(endEffector.RotateCoralPickup())
                .alongWith(drive.alignToReef(2))
                .andThen(Commands.waitUntil(elevator::atTargetPosition))
                .andThen(
                    endEffector.setShooterTimed(
                        EndEffectorConstants.PlacementSpeed, EndEffectorConstants.PlacementTime))
                .andThen(endEffector.RotateCoralPickup())
                .andThen(elevator.goToCoralPickup()));

    // ----------------------------------------------------------------------------------------------------------------------------------------------------
    // Level Two Elevator
    controller2.x().onTrue(elevator.goToLevelTwo().andThen(endEffector.RotateCoralPlacement()));

    // Level two Elevator auto align left
    controller2
        .x()
        .and(controller2.leftBumper())
        .whileTrue(
            elevator
                .goToLevelTwo()
                .alongWith(endEffector.RotateCoralPlacement())
                .alongWith(drive.alignToReef(1))
                .andThen(Commands.waitUntil(elevator::atTargetPosition))
                .andThen(
                    endEffector.setShooterTimed(
                        EndEffectorConstants.PlacementSpeed, EndEffectorConstants.PlacementTime))
                .andThen(endEffector.RotateCoralPickup())
                .andThen(elevator.goToCoralPickup()));

    // Level two Elevator auto align right
    controller2
        .x()
        .and(controller2.rightBumper())
        .whileTrue(
            elevator
                .goToLevelTwo()
                .alongWith(endEffector.RotateCoralPlacement())
                .alongWith(drive.alignToReef(2))
                .andThen(Commands.waitUntil(elevator::atTargetPosition))
                .andThen(
                    endEffector.setShooterTimed(
                        EndEffectorConstants.PlacementSpeed, EndEffectorConstants.PlacementTime))
                .andThen(endEffector.RotateCoralPickup())
                .andThen(elevator.goToCoralPickup()));

    // Level two Elevator algae
    controller2
        .x()
        .and(controller2.povDown())
        .whileTrue(
            drive
                .alignToReef(3)
                .alongWith(Commands.waitUntil(drive::atTargetPositionL3))
                .andThen(elevator.goToLevelTwo())
                .alongWith(endEffector.RotateCoralPlacement())
                .andThen(Commands.waitUntil(elevator::atTargetPosition))
                .andThen(
                    endEffector.setShooterTimed(
                        EndEffectorConstants.PlacementSpeed, EndEffectorConstants.KnockTime))
                .andThen(endEffector.RotateCoralPickup())
                .andThen(elevator.goToCoralPickup()));
    // ----------------------------------------------------------------------------------------------------------------------------------------------------
    // Level Three Elevator
    controller2.b().onTrue(elevator.goToLevelThree().andThen(endEffector.RotateCoralPlacement()));

    // Level three Elevator auto align left
    controller2
        .b()
        .and(controller2.leftBumper())
        .whileTrue(
            drive
                .alignToReef(1)
                .alongWith(Commands.waitUntil(drive::atTargetPositionL3))
                .andThen(elevator.goToLevelThree())
                .alongWith(endEffector.RotateCoralPlacement())
                .andThen(Commands.waitUntil(elevator::atTargetPosition))
                .andThen(
                    endEffector.setShooterTimed(
                        EndEffectorConstants.PlacementSpeed, EndEffectorConstants.PlacementTime))
                .andThen(endEffector.RotateCoralPickup())
                .andThen(elevator.goToCoralPickup()));

    // Level three Elevator auto align right
    controller2
        .b()
        .and(controller2.rightBumper())
        .whileTrue(
            drive
                .alignToReef(2)
                .alongWith(Commands.waitUntil(drive::atTargetPositionL3))
                .andThen(elevator.goToLevelThree())
                .alongWith(endEffector.RotateCoralPlacement())
                .andThen(Commands.waitUntil(elevator::atTargetPosition))
                .andThen(
                    endEffector.setShooterTimed(
                        EndEffectorConstants.PlacementSpeed, EndEffectorConstants.PlacementTime))
                .andThen(endEffector.RotateCoralPickup())
                .andThen(elevator.goToCoralPickup()));

    // Level three Elevator algae
    controller2
        .b()
        .and(controller2.povDown())
        .whileTrue(
            drive
                .alignToReef(3)
                .alongWith(Commands.waitUntil(drive::atTargetPositionL3))
                .andThen(elevator.goToLevelThree())
                .alongWith(endEffector.RotateCoralPlacement())
                .andThen(Commands.waitUntil(elevator::atTargetPosition))
                .andThen(
                    endEffector.setShooterTimed(
                        EndEffectorConstants.PlacementSpeed, EndEffectorConstants.KnockTime))
                .andThen(endEffector.RotateCoralPickup())
                .andThen(elevator.goToCoralPickup()));
    // ----------------------------------------------------------------------------------------------------------------------------------------------------
    // Level Four Elevator
    controller2.y().onTrue(elevator.goToLevelFour().andThen(endEffector.RotateCoralL4Placement()));

    // Level four Elevator auto align left
    controller2
        .y()
        .and(controller2.leftBumper())
        .whileTrue(
            drive
                .alignToReefStart(1)
                .alongWith(elevator.goToLevelFour())
                .alongWith(endEffector.RotateCoralPlacement())
                .andThen(Commands.waitUntil(elevator::atTargetPositionL4))
                .andThen(drive.alignToReefL4(1))
                .andThen(
                    endEffector.setShooterTimed(
                        EndEffectorConstants.PlacementSpeed, EndEffectorConstants.PlacementTime))
                .andThen(endEffector.RotateCoralPickup())
                .andThen(elevator.goToCoralPickup()));

    // Level four Elevator auto align right
    controller2
        .y()
        .and(controller2.rightBumper())
        .whileTrue(
            drive
                .alignToReefStart(2)
                .alongWith(elevator.goToLevelFour())
                .alongWith(endEffector.RotateCoralPlacement())
                .andThen(Commands.waitUntil(elevator::atTargetPositionL4))
                .andThen(drive.alignToReefL4(2))
                .andThen(
                    endEffector.setShooterTimed(
                        EndEffectorConstants.PlacementSpeed, EndEffectorConstants.PlacementTime))
                .andThen(endEffector.RotateCoralPickup())
                .andThen(elevator.goToCoralPickup()));
    // ----------------------------------------------------------------------------------------------------------------------------------------------------

    // Ground Level
    controller2.povDown().whileTrue(drive.alignToReef(3));

    // Barge
    controller2
        .povUp()
        .onTrue(elevator.goToLevelFour().andThen(endEffector.RotateBargePlacement()));

    // Reef 1
    controller2.leftBumper().whileTrue((drive.alignToReef(1)));

    // Reef 2
    controller2.rightBumper().whileTrue((drive.alignToReef(2)));

    // Coral Station 1
    controller2
        .povLeft()
        .whileTrue(
            (drive
                .alignToCoralStation(1)
                .alongWith(
                    elevator
                        .goToCoralPickup()
                        .alongWith(
                            endEffector
                                .RotateCoralPickup()
                                .alongWith(
                                    endEffector.setShooter(EndEffectorConstants.IntakeSpeed))))))
        .onFalse(endEffector.setShooter(0));

    // Coral Station 2
    controller2
        .povRight()
        .whileTrue(
            (drive
                .alignToCoralStation(2)
                .alongWith(
                    elevator
                        .goToCoralPickup()
                        .alongWith(
                            endEffector
                                .RotateCoralPickup()
                                .alongWith(
                                    endEffector.setShooter(EndEffectorConstants.IntakeSpeed))))))
        .onFalse(endEffector.setShooter(0));

    controller1.x().onTrue(Commands.runOnce(drive::stopWithX, drive));
    // controller1.b().onTrue(drive.offCam(new VisionIOPhotonVision(camera0Name, robotToCamera0)));
    // controller1.b().onTrue(Commands.runOnce());

    // Elevator
    // Ground Intake
    /*    controller2
        .rightTrigger()
        .onTrue(
            elevator
                .goToGroundLevel()
                .andThen(endEffector.rotateDown())
                .alongWith(endEffector.setShooter(EndEffectorConstants.IntakeSpeed)))
        .onFalse(endEffector.setShooter(0.0).alongWith(endEffector.rotateUp()));

    controller2.leftBumper().onTrue(elevator.goToLevelFour().andThen(endEffector.rotateUp()));*/

    // controller2.a().whileTrue(PlaceOnReef(elevator, drive, endEffector, false));
    /*
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
    */
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  // AUTO--------------------------------------------------------------------------------------------------------------------------
  private AutoRoutine L3Main() {
    AutoRoutine routine = autoFactory.newRoutine("L3 Main");

    routine
        .active()
        .onTrue(
            drive
                .alignToReef(1)
                .alongWith(elevator.goToLevelThree())
                .alongWith(endEffector.RotateCoralPlacement())
                .andThen(Commands.waitUntil(elevator::atTargetPosition))
                .andThen(
                    endEffector.setShooterTimed(
                        EndEffectorConstants.PlacementSpeed, EndEffectorConstants.PlacementTime))
                .andThen(elevator.goToLevelTwo())
                .andThen(drive.alignToReef(3))
                .andThen(
                    endEffector.setShooterTimed(
                        EndEffectorConstants.PlacementSpeed, EndEffectorConstants.KnockTime))
                .andThen(endEffector.RotateCoralPickup())
                .andThen(elevator.goToCoralPickup()));

    return routine;
  }

  private AutoRoutine L4Main() {
    AutoRoutine routine = autoFactory.newRoutine("L4 Main");

    routine
        .active()
        .onTrue(
            drive
                .alignToReefStart(1)
                .alongWith(elevator.goToLevelFour())
                .alongWith(endEffector.RotateCoralPlacement())
                .andThen(Commands.waitUntil(elevator::atTargetPositionL4))
                .andThen(drive.alignToReefL4(1))
                .andThen(
                    endEffector.setShooterTimed(
                        EndEffectorConstants.PlacementSpeed, EndEffectorConstants.PlacementTime))
                .andThen(elevator.goToLevelTwo())
                .andThen(drive.alignToReef(3))
                .andThen(
                    endEffector.setShooterTimed(
                        EndEffectorConstants.PlacementSpeed, EndEffectorConstants.KnockTime))
                .andThen(endEffector.RotateCoralPickup())
                .andThen(elevator.goToCoralPickup()));

    return routine;
  }

  private AutoRoutine L2Main() {
    AutoRoutine routine = autoFactory.newRoutine("L2 Main");
    routine
        .active()
        .onTrue(
            drive
                .alignToReef(1)
                .andThen(elevator.goToLevelTwo())
                .andThen(endEffector.RotateCoralPlacement())
                .andThen(Commands.waitUntil(elevator::atTargetPosition))
                .andThen(
                    endEffector.setShooterTimed(
                        EndEffectorConstants.PlacementSpeed, EndEffectorConstants.PlacementTime))
                .andThen(drive.alignToReef(3))
                .andThen(elevator.goToLevelThree())
                .andThen(
                    endEffector.setShooterTimed(
                        EndEffectorConstants.PlacementSpeed, EndEffectorConstants.KnockTime))
                .andThen(endEffector.RotateCoralPickup())
                .andThen(elevator.goToCoralPickup()));

    return routine;
  }

  private AutoRoutine L2Cycle() {
    AutoRoutine routine = autoFactory.newRoutine("L2 Cycle");
    routine
        .active()
        .onTrue(
            drive
                .alignToReef(1)
                .andThen(elevator.goToLevelTwo())
                .andThen(endEffector.RotateCoralPlacement())
                .andThen(Commands.waitUntil(elevator::atTargetPosition))
                .andThen(
                    endEffector.setShooterTimed(
                        EndEffectorConstants.PlacementSpeed, EndEffectorConstants.PlacementTime))
                .andThen(drive.alignToReef(3))
                .andThen(elevator.goToLevelThree())
                .andThen(
                    endEffector.setShooterTimed(
                        EndEffectorConstants.PlacementSpeed, EndEffectorConstants.KnockTime))
                .andThen(endEffector.RotateCoralPickup())
                .andThen(elevator.goToCoralPickup())
                .andThen(drive.alignToCoralStation(1))
                .andThen(
                    endEffector.setShooterTimed(
                        EndEffectorConstants.IntakeSpeed, EndEffectorConstants.IntakeTime))
                .andThen(drive.alignToReef(1))
                .andThen(elevator.goToLevelThree())
                .andThen(endEffector.RotateCoralPlacement())
                .andThen(Commands.waitUntil(elevator::atTargetPosition))
                .andThen(
                    endEffector.setShooterTimed(
                        EndEffectorConstants.PlacementSpeed, EndEffectorConstants.PlacementTime))
                .andThen(elevator.goToLevelTwo())
                .andThen(drive.alignToReef(3))
                .andThen(
                    endEffector.setShooterTimed(
                        EndEffectorConstants.PlacementSpeed, EndEffectorConstants.KnockTime))
                .andThen(endEffector.RotateCoralPickup())
                .andThen(elevator.goToGroundLevel()));
    return routine;
  }

  private AutoRoutine L4Cycle() {
    AutoRoutine routine = autoFactory.newRoutine("L4 Cycle");
    routine
        .active()
        .onTrue(
            drive
                .alignToReefStart(1)
                .alongWith(elevator.goToLevelFour())
                .alongWith(endEffector.RotateCoralPlacement())
                .andThen(Commands.waitUntil(elevator::atTargetPositionL4))
                .andThen(drive.alignToReefL4(1))
                .andThen(
                    endEffector.setShooterTimed(
                        EndEffectorConstants.PlacementSpeed, EndEffectorConstants.PlacementTime))
                .andThen(drive.alignToReef(3))
                .andThen(elevator.goToLevelThree())
                .andThen(
                    endEffector.setShooterTimed(
                        EndEffectorConstants.PlacementSpeed, EndEffectorConstants.KnockTime))
                .andThen(endEffector.RotateCoralPickup())
                .andThen(elevator.goToCoralPickup())
                .andThen(drive.alignToCoralStation(1))
                .andThen(
                    endEffector.setShooterTimed(
                        EndEffectorConstants.IntakeSpeed, EndEffectorConstants.IntakeTime))
                .andThen(drive.alignToReefStart(1))
                .alongWith(elevator.goToLevelFour())
                .andThen(endEffector.RotateCoralPlacement())
                .andThen(Commands.waitUntil(elevator::atTargetPositionL4))
                .andThen(drive.alignToReefL4(1))
                .andThen(
                    endEffector.setShooterTimed(
                        EndEffectorConstants.PlacementSpeed, EndEffectorConstants.PlacementTime))
                .andThen(elevator.goToLevelTwo())
                .andThen(drive.alignToReef(3))
                .andThen(
                    endEffector.setShooterTimed(
                        EndEffectorConstants.PlacementSpeed, EndEffectorConstants.KnockTime))
                .andThen(endEffector.RotateCoralPickup())
                .andThen(elevator.goToGroundLevel()));
    return routine;
  }
}
