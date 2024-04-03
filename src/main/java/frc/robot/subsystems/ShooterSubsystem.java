package frc.robot.subsystems;

import java.util.Random;
import java.util.TreeMap;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.util.CanIDs;
import frc.robot.util.LookupTable;
import frc.robot.util.PID;
 
public class ShooterSubsystem extends SubsystemBase {

    public static class Constants {
        public static PID pivotPID = new PID(1.0, 0, 0.01); // TODO tune this
        public static PID flywheelPID = new PID(1.0, 0, 0.01); //TODO: tune this

        public static double armStage1Angle = 53.92;
        public static double armStage2Angle = 4.6;
        public static double maxAngle = 10;
        public static double demoAngle = 10;
        public static double stowAngle = 10;
        public static double minAngle = 10;

        public static double manualClose = 30.7;

        public static double maximumRPMError = 10.0; //TODO: Check if this value is even realistic

        public static LookupTable angleLookup;
        public static LookupTable powerLookup;

        static {
            TreeMap<Double, Double> angleMap = new TreeMap<Double, Double>();
            TreeMap<Double, Double> powerMap = new TreeMap<Double, Double>();

            // -- Angle lookup table values
            angleMap.put(0.20, 33.0);
            angleMap.put(1.39, 33.0);
            angleMap.put(1.72, 40.0);
            angleMap.put(2.00, 44.3);
            angleMap.put(2.40, 50.0);
            angleMap.put(2.88, 54.8);
            angleMap.put(3.33, 59.0);
            angleMap.put(3.67, 60.8);
            angleMap.put(3.67, 60.8);
            angleMap.put(3.89, 62.9);
            angleMap.put(4.27, 63.5);
            angleMap.put(5.00, 64.0);

            // -- Power lookup table values
            powerMap.put(0.00, 0.55);
            powerMap.put(1.40, 0.55);
            powerMap.put(2.00, 0.7);
            powerMap.put(5.00, 0.7);

            angleLookup = new LookupTable(angleMap);
            powerLookup = new LookupTable(powerMap);
        }
    }

    private TalonFX topMotor;
    private TalonFX bottomMotor;
    private CANSparkMax pivotMotor;

    private AbsoluteEncoder pivotEncoder;
    
    private double pivotTarget;
    private boolean running;

    private double targetRPM;

    public ShooterSubsystem() {
        super();

        pivotTarget = Constants.minAngle;

        topMotor = new TalonFX(CanIDs.get("shooter-top"), RobotContainer.getRioCanBusName());
        topMotor.stopMotor();
        
        bottomMotor = new TalonFX(CanIDs.get("shooter-bottom"), RobotContainer.getRioCanBusName());
        bottomMotor.stopMotor();

        pivotMotor = new CANSparkMax(CanIDs.get("shooter-pivot"), MotorType.kBrushless);
        pivotMotor.stopMotor();
        pivotMotor.setIdleMode(IdleMode.kBrake);

        pivotEncoder = pivotMotor.getAbsoluteEncoder(Type.kDutyCycle);
        pivotEncoder.setPositionConversionFactor(360);
        pivotEncoder.setZeroOffset(146.6); // 4 degrees from zero for wrapping issues
    }

    /**
     * Stops the shooter if it's running and starts it
     * if it's not, at the specified speed
     * @param speed the power to set the wheels to
     * @return the command
     */
    public Command toggleShooter(double speed) {
        return runOnce(() -> {
            if (!running) CommandScheduler.getInstance().schedule(startShooter(speed));
            else CommandScheduler.getInstance().schedule(stopShooter());
        });
    }

    public Command startAndRun(double speed, double time) {
        return run(() -> setPowerRaw(speed)).withTimeout(time);
    }

    /**
     * Starts the shooter wheels
     * @param speed the power to set the wheels to
     * @return the command
     */
    public Command startShooter(double speed) {
        return runOnce(() -> {
            topMotor.set(speed);
            bottomMotor.set(speed);
        }).andThen(() -> running = true);
    }

    public Command startShooterRPM(double rpm) {
        return runOnce(() -> {
            // topMotor.setControl(null);
            // bottomMotor.setControl(null);
            targetRPM = rpm;
        });
    }

    public Command startSeperateShooters(double speedTop, double speedBottom) {
        return runOnce(() -> {
            topMotor.set(speedTop);
            bottomMotor.set(speedBottom);
        }).andThen(() -> running = true);
    }

    /**
     * Stops the shooter wheel motors
     * @return the command
     */
    public Command stopShooter() {
        return runOnce(() -> {
            bottomMotor.stopMotor();
            topMotor.stopMotor();
        }).andThen(() -> running = false);
    }

    /**
     * @return true if the shooter wheels are running
     */
    public boolean isRunning() {
        return running;
    }

    /**
     * Set the position of the shooter pivot
     * @param target The target angle in degrees
     * @return The command that moves the pivot
     */
    public Command setPivotTarget(double target) {
        return runOnce(() -> pivotTarget = target);
    }

    /**
     * Non-command version of setPivotTarget()
     * @param target The target angle in degrees
     */
    public void setPivotTargetRaw(double target) {
        pivotTarget = target;
    }

    public void changePivotTarget(double amount) {
        pivotTarget += amount;
    }

    /**
     * @return the current pivot target
     */
    public double getPivotTarget() {
        return pivotTarget;
    }

    /**
     * Sets the pivot to target its min angle
     * @return the command
     */
    public Command stowPivot() {
        return setPivotTarget(Constants.minAngle);
    }

    /**
     * Gets whether the flywheels are close enough to the desired speed
     * @return Boolean determining if they are up to speed
     */
    public boolean upToSpeed() {
        double topDeviation = Math.abs(targetRPM - topMotor.getVelocity().getValueAsDouble());
        double bottomDeviation = Math.abs(targetRPM - bottomMotor.getVelocity().getValueAsDouble());

        return topDeviation <= Constants.maximumRPMError && bottomDeviation <= Constants.maximumRPMError;
    }

    @Override
    public void periodic() {
        super.periodic();

        // Calculate the target and current rotations
        double encoderPos = pivotEncoder.getPosition();
        Rotation2d target = Rotation2d.fromDegrees(pivotTarget);
        Rotation2d current = Rotation2d.fromDegrees(encoderPos);

        double error = Math.max(Math.min(target.minus(current).getDegrees() / 160.0, 0.2), -0.2);
        pivotMotor.set(error - 0.005);

        // Display values
        SmartDashboard.putNumber("Shooter Position", pivotEncoder.getPosition());
        SmartDashboard.updateValues();
    }

    // Please don't use this
    public Command setRandomPivotTarget() {
        return runOnce(() -> setPivotTargetRaw(new Random().nextInt(32, 50)));
    }

    public void setPowerRaw(double speed) {
        bottomMotor.set(speed);
        topMotor.set(speed);
    }

    public void setFromDistance(double distance) {
        setPivotTargetRaw(Constants.angleLookup.getInterpolated(distance));
        setPowerRaw(Constants.powerLookup.getInterpolated(distance));
    }
}