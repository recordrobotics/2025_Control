package frc.robot.subsystems.io.sim;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.RobotMap;
import frc.robot.commands.simulation.CoralIntakeToElevator;
import frc.robot.subsystems.io.CoralIntakeIO;
import frc.robot.utils.DCMotors;
import frc.robot.utils.IntakeSimulationUtils;
import frc.robot.utils.ProjectileSimulationUtils;
import org.ironmaple.simulation.IntakeSimulation;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.AbstractDriveTrainSimulation;
import org.ironmaple.simulation.gamepieces.GamePieceProjectile;
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeCoralOnField;

public class CoralIntakeSim implements CoralIntakeIO {

  private final double periodicDt;

  private final SparkMax wheel;
  private final TalonFX arm;

  private final SparkMaxSim wheelSim;
  private final TalonFXSimState armSim;

  private final DCMotor wheelMotor = DCMotor.getNeo550(1);
  private final DCMotor armMotor = DCMotors.getKrakenX44(1);

  private final DCMotorSim wheelSimModel =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(
              Constants.CoralIntake.wheel_kV, Constants.CoralIntake.wheel_kA),
          wheelMotor,
          0.01,
          0.01);

  private final SingleJointedArmSim armSimModel =
      new SingleJointedArmSim(
          LinearSystemId.createSingleJointedArmSystem(
              armMotor, 0.179, Constants.CoralIntake.ARM_GEAR_RATIO),
          armMotor,
          Constants.CoralIntake.ARM_GEAR_RATIO,
          Units.inchesToMeters(17.02),
          -0.95,
          Math.PI / 2,
          true,
          Constants.CoralIntake.ARM_START_POS,
          0.001,
          0.001);

  private final AbstractDriveTrainSimulation drivetrainSim;
  private final IntakeSimulation intakeSimulation;

  public CoralIntakeSim(double periodicDt, AbstractDriveTrainSimulation drivetrainSim) {
    this.periodicDt = periodicDt;
    this.drivetrainSim = drivetrainSim;

    wheel = new SparkMax(RobotMap.CoralIntake.WHEEL_ID, MotorType.kBrushless);
    arm = new TalonFX(RobotMap.CoralIntake.ARM_ID);
    wheelSim = new SparkMaxSim(wheel, wheelMotor);
    armSim = arm.getSimState();

    var width = Inches.of(21.25);
    var lengthExtended = Inches.of(8.419554);
    var intakeRect =
        IntakeSimulationUtils.getIntakeRectangle(
            drivetrainSim,
            width.in(Meters),
            lengthExtended.in(Meters),
            IntakeSimulation.IntakeSide.LEFT); // apparently it's on the left in maplesim
    intakeRect.translate(Inches.of(-3.3999).in(Meters), 0);

    intakeSimulation = new IntakeSimulation("Coral", drivetrainSim, intakeRect, 1);
  }

  public IntakeSimulation getIntakeSimulation() {
    return intakeSimulation;
  }

  @Override
  public void applyArmTalonFXConfig(TalonFXConfiguration configuration) {
    arm.getConfigurator().apply(configuration);
  }

  @Override
  public void setWheelVoltage(double outputVolts) {
    wheel.setVoltage(outputVolts);
  }

  @Override
  public void setArmVoltage(double outputVolts) {
    arm.setVoltage(outputVolts);
  }

  @Override
  public void setWheelPosition(double newValue) {
    wheel.getEncoder().setPosition(newValue);
  }

  @Override
  public void setArmPosition(double newValue) {
    arm.setPosition(newValue);
  }

  @Override
  public void setArmMotionMagic(MotionMagicVoltage request) {
    arm.setControl(request);
  }

  @Override
  public double getWheelPosition() {
    return wheel.getEncoder().getPosition();
  }

  @Override
  public double getWheelVelocity() {
    return wheel.getEncoder().getVelocity();
  }

  @Override
  public double getWheelVoltage() {
    return wheel.getAppliedOutput() * wheel.getBusVoltage();
  }

  @Override
  public double getArmVoltage() {
    return arm.getMotorVoltage().getValueAsDouble();
  }

  @Override
  public double getArmPosition() {
    return arm.getPosition().getValueAsDouble();
  }

  @Override
  public double getArmVelocity() {
    return arm.getVelocity().getValueAsDouble();
  }

  @Override
  public void setWheelPercent(double newValue) {
    wheel.set(newValue);
  }

  @Override
  public void setArmPercent(double newValue) {
    arm.set(newValue);
  }

  @Override
  public double getWheelPercent() {
    return wheel.get();
  }

  @Override
  public double getArmPercent() {
    return arm.get();
  }

  @Override
  public double getWheelCurrentDrawAmps() {
    return wheelSimModel.getCurrentDrawAmps();
  }

  @Override
  public double getArmCurrentDrawAmps() {
    return arm.getSupplyCurrent().getValueAsDouble();
  }

  @Override
  public void close() throws Exception {
    wheel.close();
    arm.close();
  }

  private boolean hadGamePieces = false;

  @Override
  public void simulationPeriodic() {
    armSim.setSupplyVoltage(RobotController.getBatteryVoltage());

    var wheelVoltage = wheelSim.getAppliedOutput() * wheelSim.getBusVoltage();
    var armVoltage = armSim.getMotorVoltage();

    wheelSimModel.setInputVoltage(wheelVoltage);
    wheelSimModel.update(periodicDt);

    armSimModel.setInputVoltage(armVoltage);
    armSimModel.update(periodicDt);

    wheelSim.iterate(
        Constants.CoralIntake.WHEEL_GEAR_RATIO
            * Units.radiansToRotations(wheelSimModel.getAngularVelocityRadPerSec())
            * 60.0,
        RobotController.getBatteryVoltage(),
        periodicDt);

    armSim.setRawRotorPosition(
        Constants.CoralIntake.ARM_GEAR_RATIO
            * Units.radiansToRotations(
                armSimModel.getAngleRads() - Constants.CoralIntake.ARM_START_POS));
    armSim.setRotorVelocity(
        Constants.CoralIntake.ARM_GEAR_RATIO
            * Units.radiansToRotations(armSimModel.getVelocityRadPerSec()));

    if (RobotContainer.coralIntake.getWheelVelocity() < -1) {
      intakeSimulation.startIntake();
    } else if (RobotContainer.coralIntake.getWheelVelocity() > 1) {
      if (intakeSimulation.obtainGamePieceFromIntake()
          && RobotContainer.coralIntake.getArmAngle() < Units.degreesToRadians(70)) {
        var ejectPose =
            RobotContainer.model
                .coralIntake
                .getCoralTargetPose()
                .relativeTo(new Pose3d(RobotContainer.model.getRobot()));
        RobotContainer.model.getRobotCoral().poseSupplier = () -> null;

        var rot = drivetrainSim.getSimulatedDriveTrainPose().getRotation();
        double launchingSpeed = 2.0;

        SimulatedArena.getInstance()
            .addGamePieceProjectile(
                new GamePieceProjectile(
                        ReefscapeCoralOnField.REEFSCAPE_CORAL_INFO,
                        // Obtain robot position from drive simulation
                        drivetrainSim
                            .getSimulatedDriveTrainPose()
                            .getTranslation()
                            .plus(ejectPose.toPose2d().getTranslation().rotateBy(rot)),
                        ProjectileSimulationUtils.calculateInitialProjectileVelocityMPS(
                            ejectPose.toPose2d().getTranslation(),
                            drivetrainSim.getDriveTrainSimulatedChassisSpeedsFieldRelative(),
                            rot.plus(Rotation2d.kCCW_90deg),
                            launchingSpeed * Math.cos(RobotContainer.coralIntake.getArmAngle())),
                        ejectPose.getZ(),
                        launchingSpeed * Math.sin(RobotContainer.coralIntake.getArmAngle()),
                        RobotContainer.model.coralIntake.getCoralTargetPose().getRotation())
                    .withTouchGroundHeight(0.2)
                    .enableBecomesGamePieceOnFieldAfterTouchGround());
      }
      intakeSimulation.stopIntake();
    } else {
      intakeSimulation.stopIntake();
    }

    if (intakeSimulation.getGamePiecesAmount() != 0) {
      if (!hadGamePieces) {
        RobotContainer.model.getRobotCoral().poseSupplier =
            () -> RobotContainer.model.coralIntake.getCoralTargetPose();
        hadGamePieces = true;
      }
    } else {
      hadGamePieces = false;
    }

    // source intake simulation
    if (RobotContainer.elevatorHead.getVelocity() > 0.6
        && RobotContainer.model.getRobotCoral().poseSupplier.get() == null) {
      Pose3d sourceTarget = RobotContainer.model.elevator.getCoralIntakeEjectFinalPose();

      var projectiles = SimulatedArena.getInstance().gamePieceLaunched();
      for (var projectile : projectiles) {
        if (projectile.gamePieceType.equals("Coral")) {
          if (projectile.getPose3d().getTranslation().getDistance(sourceTarget.getTranslation())
                  < 0.2
              && // position matches
              projectile
                      .getPose3d()
                      .getRotation()
                      .toVector()
                      .unit()
                      .dot(sourceTarget.getRotation().toVector().unit())
                  < -0.9
              && // TODO: rotation matches (always -1 idk why but works anyways)
              projectile
                      .getVelocity3dMPS()
                      .toVector()
                      .unit()
                      .dot(sourceTarget.getRotation().toVector().unit())
                  < -0.2 // velocity is in direction of target
          ) {
            SimulatedArena.getInstance().removeProjectile(projectile);
            var pose = projectile.getPose3d();
            RobotContainer.model.getRobotCoral().poseSupplier = () -> pose;
            new CoralIntakeToElevator().schedule();
            break;
          }
        }
      }
    }
  }
}
