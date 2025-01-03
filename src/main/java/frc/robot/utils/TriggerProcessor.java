package frc.robot.utils;
import java.util.Optional;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.FieldPosition;
import frc.robot.subsystems.Drivetrain;

public class TriggerProcessor {

    public @interface TriggerDistance{
        public FieldPosition position();
        public double distance();
    }
    
    public static <A> Optional<Boolean> isWithinDistance(Class<A> clas){
        if(!clas.isAnnotationPresent(TriggerDistance.class))
            return Optional.empty();
        
        TriggerDistance annotation = clas.getAnnotation(TriggerDistance.class);
        // read the docs
        Translation2d swervePosition = Drivetrain.poseFilter.getEstimatedPosition().getTranslation();
        Translation2d annotationPosition = annotation.position().getPose();
        return Optional.of(swervePosition.getDistance(annotationPosition) <= annotation.distance());
    }
    
    public static <A> Trigger withAnnotation(Trigger trigger, Class<A> clas){
        return trigger.and(()->{
            return isWithinDistance(clas).orElse(true);
        });
    }
}