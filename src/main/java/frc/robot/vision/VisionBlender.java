package frc.robot.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class VisionBlender {
    public AprilTagLimelight visionA;
    public AprilTagLimelight visionB;

    public VisionBlender(AprilTagLimelight visionA, AprilTagLimelight visionB) {
        this.visionA = visionA;
        this.visionB = visionB;
    }

    public Pose2d getPose() {
        if (visionA.hasTarget() && !visionB.hasTarget()) return visionA.getPose();
        else if (visionB.hasTarget() && !visionA.hasTarget()) return visionA.getPose();
        else if (visionB.hasTarget() && visionA.hasTarget()) {
            Translation2d translation = visionA.getPose().getTranslation()
                .plus(visionB.getPose().getTranslation())
                .div(2);
            
            Rotation2d rotation = visionA.getPose().getRotation()
                .plus(visionB.getPose().getRotation())
                .div(2);

            return new Pose2d(translation, rotation);
        }

        return visionA.getPose();
    }

    public void updateValues() {
        if (visionA.hasTarget()) visionA.updateValues();
        if (visionB.hasTarget()) visionB.updateValues();
    }
}
