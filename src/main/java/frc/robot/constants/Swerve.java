package frc.robot.constants;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import frc.robot.subsystems.drive.Drive;
import edu.wpi.first.units.measure.*;
import static edu.wpi.first.units.Units.*;
import frc.robot.constants.*;
import frc.robot.constants.IdConstants;



public enum Swerve {
  LeftFront(IdConstants.CANId.FrontLeftTurnId, IdConstants.CANId.FrontLeftDriveId, IdConstants.CANId.FrontLeftEncoderId, Rotations.of(0.15234375), Inches.of(10), Inches.of(10), true, false), 
  RightFront(IdConstants.CANId.FrontRightTurnId, IdConstants.CANId.FrontRigh, IdConstants.CANId.FrontLeftEncoderId, Rotations.of(0.15234375), Inches.of(10), Inches.of(10), true, false);

  private static final int kFrontRightDriveMotorId = 1;
  private static final int kFrontRightSteerMotorId = 0;
  private static final int kFrontRightEncoderId = 0;
  private static final Angle kFrontRightEncoderOffset = Rotations.of(-0.4873046875);
  private static final boolean kFrontRightSteerMotorInverted = true;
  private static final boolean kFrontRightEncoderInverted = false;

  private static final Distance kFrontRightXPos = Inches.of(10);
  private static final Distance kFrontRightYPos = Inches.of(-10);

  private static final int kFrontRightDriveMotorId = 1;
  private static final int kFrontRightSteerMotorId = 0;
  private static final int kFrontRightEncoderId = 0;
  private static final Angle kFrontRightEncoderOffset = Rotations.of(-0.4873046875);
  private static final boolean kFrontRightSteerMotorInverted = true;
  private static final boolean kFrontRightEncoderInverted = false;

  private static final Distance kFrontRightXPos = Inches.of(10);
  private static final Distance kFrontRightYPos = Inches.of(-10);





  public final int SteerMotorId;
  public final int DriveMotorId;
  public final int EncoderId;
  public final Angle EncoderOffset;
  public final Distance XPos;
  public final Distance YPos;
  public final boolean SteerMotorInverted;
  public final boolean EncoderInverted;

  Swerve(
      int SteerMotorId,
      int DriveMotorId,
      int EncoderId,
      Angle EncoderOffset,
      Distance XPos,
      Distance YPos,
      boolean SteerMotorInverted,
      boolean EncoderInverted) {
    this.SteerMotorId = SteerMotorId;
    this.DriveMotorId = DriveMotorId;
    this.EncoderId = EncoderId;
    this.EncoderOffset = EncoderOffset;
    this.XPos = XPos;
    this.YPos = YPos;
    this.SteerMotorInverted = SteerMotorInverted;
    this.EncoderInverted = EncoderInverted;
  }
}
