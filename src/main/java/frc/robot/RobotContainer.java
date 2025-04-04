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
import choreo.auto.AutoTrajectory;
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
        autoChooser.addRoutine("L2 Knock Off", this::L2Knock);
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
            () -> -controller1.getLeftY(),
            () -> -controller1.getLeftX(),
            () -> -controller1.getRightX()));

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
    controller1.povLeft().whileTrue(climber.setSpeed(1.0)).onFalse(climber.setSpeed(0));

    // Climber UnClimb
    controller1.povRight().whileTrue(climber.setSpeed(-0.5)).onFalse(climber.setSpeed(0));

    // Level One Elevator
    controller2.a().onTrue(elevator.goToLevelOne().andThen(endEffector.RotateTroftPlacement()));
    //    // Level Two Elevator
    controller2.x().onTrue(elevator.goToLevelTwo().andThen(endEffector.RotateCoralPlacement()));
    // Level Three Elevator
    controller2.b().onTrue(elevator.goToLevelThree().andThen(endEffector.RotateCoralPlacement()));
    // Level Four Elevator
    controller2.y().onTrue(elevator.goToLevelFour().andThen(endEffector.RotateCoralL4Placement()));
    // Ground Level
    controller2
        .povDown()
        .onTrue(elevator.goToGroundLevel().andThen(endEffector.RotateCoralPickup()));
    // Barge
    controller2
        .povUp()
        .onTrue(elevator.goToLevelFour().andThen(endEffector.RotateBargePlacement()));

    // Reef 1
    controller2.leftBumper().whileTrue((drive.alignToReef(1)));

    // Reef 2
    controller2.rightBumper().whileTrue((drive.alignToReef(2)));

    // Coral Station 1
    controller2.povLeft().whileTrue((drive.alignToCoralStation(1)));

    // Coral Station 2
    controller2.povRight().whileTrue((drive.alignToCoralStation(2)));

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
                .andThen(elevator.goToLevelThree())
                .andThen(endEffector.RotateCoralPlacement())
                .andThen(Commands.waitUntil(elevator::atTargetPosition))
                .andThen(endEffector.setShooterTimed(EndEffectorConstants.PlacementSpeed, 1))
                .andThen(endEffector.RotateCoralPickup())
                .andThen(elevator.goToGroundLevel()));

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
                .andThen(endEffector.setShooterTimed(EndEffectorConstants.PlacementSpeed, 1))
                .andThen(endEffector.RotateCoralPickup())
                .andThen(elevator.goToGroundLevel()));

    return routine;
  }

  private AutoRoutine L2Knock() {
    AutoRoutine routine = autoFactory.newRoutine("L2 Knock Off");
    routine
        .active()
        .onTrue(
            drive
                .alignToReef(1)
                .andThen(elevator.goToLevelTwo())
                .andThen(endEffector.RotateCoralPlacement())
                .andThen(Commands.waitUntil(elevator::atTargetPosition))
                .andThen(endEffector.setShooterTimed(EndEffectorConstants.PlacementSpeed, 1))
                .andThen(elevator.goToLevelThree())
                .andThen(endEffector.setShooterTimed(EndEffectorConstants.PlacementSpeed, 2))
                .andThen(endEffector.RotateCoralPickup())
                .andThen(elevator.goToGroundLevel()));

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
                .andThen(endEffector.setShooterTimed(EndEffectorConstants.PlacementSpeed, 1))
                .andThen(drive.alignToReef(3))
                .andThen(elevator.goToLevelThree())
                .andThen(endEffector.setShooterTimed(EndEffectorConstants.PlacementSpeed, 1.5))
                .andThen(endEffector.RotateCoralPickup())
                .andThen(elevator.goToCoralPickup())
                .andThen(drive.alignToCoralStation(1))
                .andThen(endEffector.setShooterTimed(EndEffectorConstants.IntakeSpeed, 5)));

    return routine;
  }

  private AutoRoutine RedRight() {
    AutoRoutine basic_right = autoFactory.newRoutine("red right");
    AutoTrajectory moveOut = basic_right.trajectory("red_right");
    //  AutoTrajectory moveReturn = basic_right.trajectory("basic_left_leave"); // basic left leave

    // When the routine begins, reset odometry and start the first trajectory
    basic_right.active().onTrue(Commands.sequence(moveOut.resetOdometry(), moveOut.cmd()));

    // Starting at the event marker named "intake", run the intake
    // pickupTraj.atTime("intake").onTrue(intakeSubsystem.intake());
    moveOut
        .done()
        .onTrue(
            endEffector
                .RotateCoralPlacement()
                .alongWith(endEffector.setShooter(EndEffectorConstants.PlacementSpeed)));
    // When the trajectory is done, start the next trajectory
    // moveOut.done().onTrue(moveAgain.cmd());

    // While the trajectory is active, prepare the scoring subsystem
    // scoreTraj.active().whileTrue(scoringSubsystem.getReady());

    // When the trajectory is done, score
    // scoreTraj.done().onTrue(scoringSubsystem.score());

    return basic_right;
  }

  private AutoRoutine Contingency() {
    AutoRoutine contingency = autoFactory.newRoutine("Contingency");
    AutoTrajectory moveOut = contingency.trajectory("testing-new");
    //  AutoTrajectory moveReturn = basic_right.trajectory("basic_left_leave"); // basic left leave

    // When the routine begins, reset odometry and start the first trajectory
    contingency.active().onTrue(Commands.sequence(moveOut.resetOdometry(), moveOut.cmd()));

    // Starting at the event marker named "intake", run the intake
    // pickupTraj.atTime("intake").onTrue(intakeSubsystem.intake());
    /*  moveOut
    .done()
    .onTrue(
        endEffector
            .RotateCoralPlacement()
            .alongWith(endEffector.setShooter(EndEffectorConstants.PlacementSpeed)));*/
    // moveOut.done().onTrue(moveAgain.cmd());

    // While the trajectory is active, prepare the scoring subsystem
    // scoreTraj.active().whileTrue(scoringSubsystem.getReady());

    // When the trajectory is done, score
    // scoreTraj.done().onTrue(scoringSubsystem.score());

    return contingency;
  }
}
