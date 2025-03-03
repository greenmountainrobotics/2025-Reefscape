package frc.robot.subsystems.intake;

import static edu.wpi.first.math.MathUtil.angleModulus;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.constants.IntakeConstants.*;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.constants.TunableConstants;
import frc.robot.util.RunMode;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();
  private final IntakeIO io;

  private final ProfiledPIDController articulationPID;

  private SysIdRoutineLog.State sysIdState = SysIdRoutineLog.State.kNone;
  private final SysIdRoutine articulationSysId;

  private double prevArticulation = 0.0;
  private double prevTimestamp = 0.0;

  private Rotation2d articulationSetpoint = new Rotation2d();

  public Intake(IntakeIO io) {
    this.io = io;

    switch (RunMode.getMode()) {
      case REAL, REPLAY -> {
        articulationPID =
            new ProfiledPIDController(
                TunableConstants.KpIntakeArticulation,
                TunableConstants.KiIntakeArticulation,
                TunableConstants.KdIntakeArticulation,
                new TrapezoidProfile.Constraints(200, 100));
      }
      default -> {
        articulationPID =
            new ProfiledPIDController(1, 0, 0.2, new TrapezoidProfile.Constraints(1, 1));
      }
    }
    setArticulation(UpRotation);

    articulationPID.setGoal(angleModulus(articulationSetpoint.getRadians()));

    articulationSysId =
        new SysIdRoutine(
            new SysIdRoutine.Config(null, null, null, state -> sysIdState = state),
            new SysIdRoutine.Mechanism(
                voltageMeasure -> {
                  io.articulationRunVoltage(voltageMeasure.in(Volts));
                },
                null,
                this));
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);
    Logger.recordOutput("Intake/Target Position Rotations", articulationSetpoint);
    Logger.recordOutput("Intake/Target Position Real Radians", articulationSetpoint.getRadians());

    Logger.recordOutput("Intake/SysIdState", sysIdState.toString());
    Logger.recordOutput("Intake/ArticulationPositionRad", inputs.articulationPosition.getRadians());
    Logger.recordOutput(
        "Intake/ArticulatinonVelocity",
        (inputs.articulationPosition.getRadians() - prevArticulation)
            / (Timer.getFPGATimestamp() - prevTimestamp));
    prevArticulation = inputs.articulationPosition.getRadians();
    prevTimestamp = Timer.getFPGATimestamp();
    if (sysIdState != SysIdRoutineLog.State.kNone) return;

    io.articulationRunVoltage(articulationPID.calculate(inputs.articulationPosition.getRadians()));
  }

  public void setIntakeSpeed(double speed) {
    io.spinRunVoltage(12.0 * speed);
  }

  public void setArticulation(Rotation2d rotation) {
    /*double adjustedRadians =
        angleModulus(rotation.getRadians() + IntakeConstants.AbsoluteEncoderOffsetRads);
    articulationPID.setGoal(adjustedRadians);*/
    articulationSetpoint = rotation;
  }

  @AutoLogOutput
  public boolean articulationIsAtSetpoint() {
    return Math.abs(getArticulation().minus(articulationSetpoint).getRadians())
        < ArticulationToleranceRad;
  }

  public Rotation2d getArticulation() {
    return inputs.articulationPosition;
  }

  /** Returns a command to run a quasistatic test in the specified direction. */
  public Command articulationSysIdQuasistatic(SysIdRoutine.Direction direction) {
    return articulationSysId.quasistatic(direction);
  }

  /** Returns a command to run a dynamic test in the specified direction. */
  public Command articulationSysIdDynamic(SysIdRoutine.Direction direction) {
    return articulationSysId.dynamic(direction);
  }

  public Command setShooter(double speed) {
    return new InstantCommand(() -> setIntakeSpeed(speed));
  }

  public Command rotateDown() {
    return new InstantCommand(
        () -> {
          setArticulation(DownRotation);
        },
        this);
  }

  public Command rotateUp() {
    return new InstantCommand(
        () -> {
          setArticulation(UpRotation);
        },
        this);
  }
}
