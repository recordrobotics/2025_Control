package frc.robot.utils.modifiers;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import org.littletonrobotics.junction.Logger;

public class AutoControlModifier extends OneshotControlModifier {

    private ChassisSpeeds speeds;

    public void drive(ChassisSpeeds speeds) {
        this.speeds = speeds;
        this.setEnabled(true);
    }

    @Override
    protected boolean perform(DrivetrainControl control) {
        Logger.recordOutput("AutoControlModifier/Speeds", speeds);

        control.applyWeightedVelocity(
                new Transform2d(
                        speeds.vxMetersPerSecond,
                        speeds.vyMetersPerSecond,
                        new Rotation2d(speeds.omegaRadiansPerSecond)),
                1);
        return true;
    }
}
