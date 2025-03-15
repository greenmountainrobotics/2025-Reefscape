package frc.robot.subsystems.endEffector;

import static edu.wpi.first.math.MathUtil.angleModulus;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.constants.EndEffectorConstants.*;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.constants.ElevatorConstants;
import frc.robot.constants.EndEffectorConstants;
import frc.robot.constants.TunableConstants;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class EndEffector extends SubsystemBase {
  private final EndEffectorIOInputsAutoLogged inputs = new EndEffectorIOInputsAutoLogged();
  private final EndEffectorIO io;
  boolean isEnabled = false;
  boolean wasEnabled = false;
  boolean pidResetDone = false;

  private final PIDController articulationPID;

  private SysIdRoutineLog.State sysIdState = SysIdRoutineLog.State.kNone;
  private final SysIdRoutine articulationSysId;

  private double prevArticulation = 0.0;
  private double prevTimestamp = 0.0;

  private double appliedVolts = 0.0;
  private double adjustedPos = 0.0;

  private Rotation2d articulationSetpoint;

  public EndEffector(EndEffectorIO io) {
    this.io = io;

    articulationPID =
        new PIDController(
            TunableConstants.KpEndEffectorArticulation,
            TunableConstants.KiEndEffectorArticulation,
            TunableConstants.KdEndEffectorArticulation);
    articulationPID.setTolerance(0.02);

    adjustedPos =
        angleModulus((inputs.articulationPosition.minus(AbsoluteEncoderOffset)).getRadians());

    adjustedPos = 90 - adjustedPos * (180 / Math.PI);

    setArticulation(Rotation2d.fromDegrees(adjustedPos));

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

    isEnabled = RobotState.isEnabled();

    // Pid reset
    if (isEnabled && !wasEnabled && !pidResetDone) {
      setArticulation(EndEffectorConstants.CoralPickupRotation);
      // articulationPID.reset(articulationSetpoint.getDegrees(), 0.0);
      pidResetDone = true;
    }

    if (!isEnabled) {
      pidResetDone = false; // Clear the PID reset flag when disabled
    }

    wasEnabled = isEnabled;

    adjustedPos =
        angleModulus((inputs.articulationPosition.minus(AbsoluteEncoderOffset)).getRadians());

    adjustedPos = 90 - adjustedPos * (180 / Math.PI);

    appliedVolts =
        Math.max(
                -1.0,
                Math.min(
                    1.0,
                    (ElevatorConstants.voltageMultiplier
                        * articulationPID.calculate(
                            adjustedPos, articulationSetpoint.getDegrees()))))
            * TunableConstants.KpEndEffectorSpeed;

    // appliedVolts = (Math.cos(adjustedPos * (Math.PI / 180)) *
    // TunableConstants.KgEndEffectorArticulation);

    /*appliedVolts =
    12
        * Math.max(
            -1.0,
            Math.min(
                1.0,
                (articulationPID.calculate(adjustedPos, articulationSetpoint.getDegrees())
                    + (Math.cos(adjustedPos * (Math.PI / 180))
                        * TunableConstants.KgEndEffectorArticulation))));*/
    io.updateInputs(inputs);
    Logger.processInputs("EndEffector", inputs);
    Logger.recordOutput("EndEffector/P", SmartDashboard.getNumber("End P", 0.0));
    Logger.recordOutput("EndEffector/I", SmartDashboard.getNumber("End I", 0.0));

    Logger.recordOutput("EndEffector/D", SmartDashboard.getNumber("End D", 0.0));
    Logger.recordOutput("EndEffector/End Voltage", SmartDashboard.getNumber("End Voltage", 0.0));

    Logger.recordOutput("EndEffector/Target Position Rotations", articulationSetpoint);
    Logger.recordOutput("EndEffector/PID Integral", articulationPID.getAccumulatedError());

    Logger.recordOutput("EndEffector/Reset PID", pidResetDone);

    Logger.recordOutput("EndEffector/Target Position Degrees", articulationSetpoint.getDegrees());

    Logger.recordOutput("EndEffector/SysIdState", sysIdState.toString());

    Logger.recordOutput("EndEffector/Articulation Volts", appliedVolts);

    Logger.recordOutput(
        "EndEffector/Articulation Cosine", (Math.cos(adjustedPos * (Math.PI / 180))));

    Logger.recordOutput(
        "EndEffector/ArticulationPositionRot", inputs.articulationPosition.getRotations());

    Logger.recordOutput(
        "EndEffector/ArticulationPositionRaw", inputs.articulationPosition.getRotations());

    Logger.recordOutput(
        "EndEffector/ArticulationPositionRad", inputs.articulationPosition.getRadians());
    Logger.recordOutput(
        "EndEffector/AdjustedArticulationPositionRot",
        (inputs.articulationPosition.minus(AbsoluteEncoderOffset)).getRotations());
    Logger.recordOutput("EndEffector/AdjustedArticulationPositionRad", adjustedPos);
    Logger.recordOutput("EndEffector/AdjustedArticulationPositionDeg", adjustedPos);
    Logger.recordOutput(
        "EndEffector/ArticulatinonVelocity",
        (inputs.articulationPosition.getRadians() - prevArticulation)
            / (Timer.getFPGATimestamp() - prevTimestamp));
    prevArticulation = inputs.articulationPosition.getRadians();
    prevTimestamp = Timer.getFPGATimestamp();
    if (sysIdState != SysIdRoutineLog.State.kNone) return;
    io.articulationRunVoltage(appliedVolts);
    /*if (!articulationIsAtSetpoint()) {


    } else {
      io.articulationRunVoltage(0.0);
    }*/

  }

  public void setIntakeSpeed(double speed) {
    io.spinRunVoltage(12.0 * speed);
  }

  public void setArticulation(Rotation2d rotation) {
    articulationSetpoint = rotation;
    // articulationPID.reset(articulationSetpoint.getDegrees(), 0.0);
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

  public Command RotateCoralPickup() {
    return new InstantCommand(
        () -> {
          setArticulation(CoralPickupRotation);
        },
        this);
  }

  public Command RotateCoralPlacement() {
    return new InstantCommand(
        () -> {
          setArticulation(CoralPlacementPosition);
        },
        this);
  }

  public Command RotateCoralL4Placement() {
    return new InstantCommand(
        () -> {
          setArticulation(CoralL4PlacementPosition);
        },
        this);
  }

  public Command RotateBargePlacement() {
    return new InstantCommand(
        () -> {
          setArticulation(BargePlacementRotation);
        },
        this);
  }

  public Command RotateTroftPlacement() {
    return new InstantCommand(
        () -> {
          setArticulation(TroftPlacementRotation);
        },
        this);
  }
}
