package frc.robot.constants;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Pose3d;

public class AprilTagConstants {
    public static final Pose3d[] TAGS = {
        new Pose3d(new Translation3d(530.49, 130.17, 12.13), new Rotation3d(300,0,0)),  // id6
        new Pose3d(new Translation3d(530.49, 130.17, 12.13), new Rotation3d(0,0,0)),    // id7
        new Pose3d(new Translation3d(530.49, 186.83, 12.13), new Rotation3d(60,0,0)),   // id8
        new Pose3d(new Translation3d(497.77, 186.83, 12.13), new Rotation3d(120,0,0)),  // id9
        new Pose3d(new Translation3d(481.39, 158.50, 12.13), new Rotation3d(180,0,0)),  // id10
        new Pose3d(new Translation3d(497.77, 130.17, 12.13), new Rotation3d(240,0,0)),  // id11
        new Pose3d(new Translation3d(160.39, 130.17, 12.13), new Rotation3d(240,0,0)),  // id17
        new Pose3d(new Translation3d(144.00, 158.50, 12.13), new Rotation3d(180,0,0)),  // id18
        new Pose3d(new Translation3d(160.39, 186.83, 12.13), new Rotation3d(120,0,0)),  // id19
        new Pose3d(new Translation3d(193.10, 186.83, 12.13), new Rotation3d(60,0,0)),   // id20
        new Pose3d(new Translation3d(209.49, 158.50, 12.13), new Rotation3d(0,0,0)),    // id21
        new Pose3d(new Translation3d(193.10, 130.17, 12.13), new Rotation3d(300,0,0))   // id22
    };
}
