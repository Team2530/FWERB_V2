package frc.robot.subsystems;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkLimitSwitch;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.PWMSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import frc.robot.Constants;
import frc.robot.Robot;

public class ElevatorSubsystem extends ProfiledPIDSubsystem {
    // NOTE: Elevator motor one has both the encoder used for positioning, and the
    // limit switch used for zeroing
    private final CANSparkFlex elevatorMotorOne = new CANSparkFlex(Constants.Elevator.elevatorOnePort,
            MotorType.kBrushless);
    private final CANSparkFlex elevatorMotorTwo = new CANSparkFlex(Constants.Elevator.elevatorTwoPort,
            MotorType.kBrushless);
    private final ElevatorFeedforward feedForward = new ElevatorFeedforward(
            Constants.Elevator.Feedforward.Ks,
            Constants.Elevator.Feedforward.Kg,
            Constants.Elevator.Feedforward.Kv,
            Constants.Elevator.Feedforward.Ka

    );

    /*
     * DCMotor gearbox,
     * double gearing,
     * double carriageMassKg,
     * double drumRadiusMeters,
     * double minHeightMeters,
     * double maxHeightMeters,
     * boolean simulateGravity,
     * double startingHeightMeters,
     * Matrix<N1, N1> measurementStdDevs
     */
    private boolean isZeroed = false; // TODO: CHENAGE
    private final RelativeEncoder elevatorEncoder;
    private final SparkLimitSwitch bottomLimit;

    private final ElevatorSim simulation = new ElevatorSim(
            Constants.Elevator.PhysicalParameters.simMotor,
            Constants.Elevator.PhysicalParameters.gearReduction,
            Constants.Elevator.PhysicalParameters.carriageMassKg,
            Constants.Elevator.PhysicalParameters.driveRadiusMeters,
            0.0,
            Constants.Elevator.PhysicalParameters.elevatorHeightMeters,
            true, 0.0,
            VecBuilder.fill(0.01));

    public ElevatorSubsystem() {
        super(
                new ProfiledPIDController(
                        Constants.Elevator.PID.kP,
                        Constants.Elevator.PID.kI,
                        Constants.Elevator.PID.kD,
                        new TrapezoidProfile.Constraints(
                                Constants.Elevator.PID.MAX_VELOCITY,
                                Constants.Elevator.PID.MAX_ACCELERATION)),
                0.0);

        // TODO: Check that this is an acceptable tolerance!
        this.getController().setTolerance(Units.inchesToMeters(0.5));

        // isZeroed = false;

        elevatorMotorOne.setIdleMode(CANSparkFlex.IdleMode.kBrake);
        elevatorMotorTwo.setIdleMode(CANSparkFlex.IdleMode.kBrake);

        elevatorMotorOne.getEncoder().setPositionConversionFactor(1.0 / Constants.Elevator.motorTurnsPerMeter);
        elevatorMotorOne.getEncoder().setVelocityConversionFactor(1.0 / Constants.Elevator.motorTurnsPerMeter);

        elevatorMotorTwo.getEncoder().setPositionConversionFactor(1.0 / Constants.Elevator.motorTurnsPerMeter);
        elevatorMotorTwo.getEncoder().setVelocityConversionFactor(1.0 / Constants.Elevator.motorTurnsPerMeter);

        elevatorEncoder = elevatorMotorOne.getEncoder();

        // elevatorEncoder.setInverted(Constants.Elevator.elevatorEncoderInverted);

        elevatorMotorOne.setInverted(Constants.Elevator.elevatorOneInverted);
        elevatorMotorTwo.setInverted(Constants.Elevator.elevatorTwoInverted);

        elevatorMotorOne.getEncoder().setPosition(0);
        elevatorMotorTwo.getEncoder().setPosition(0);

        bottomLimit = elevatorMotorOne.getReverseLimitSwitch(Constants.Elevator.bottomLimitMode);

        this.enable();
        // todo check if offset encoders
        // todo check if motors inverted
        // todo set elevator tolerance
    }

    @Override
    public void useOutput(double output, TrapezoidProfile.State setpoint) {
        double ff = feedForward.calculate(setpoint.velocity, 0.0);

        if (isZeroed || Robot.isSimulation()) {
            double op = ff + output;
            // if (bottomLimit.isPressed() && (op < 0.0)) {
            // elevatorMotorOne.setVoltage(0.0);
            // elevatorMotorTwo.setVoltage(0.0);
            // } else {
            elevatorMotorOne.set(op / 12.0);
            elevatorMotorTwo.set(op / 12.0);
            // }

            // elevatorMotorOne.setVoltage(2);
            // elevatorMotorTwo.setVoltage(2);

            SmartDashboard.putNumber("Elevator Total Output", op / 12);
            SmartDashboard.putNumber("Elevator PID Output", output / 12);
            SmartDashboard.putNumber("Elevator FF Output", ff / 12);
            SmartDashboard.putNumber("Current Elevator Target (profiled)", setpoint.position);
            if (Robot.isSimulation()) {
                simulation.setInputVoltage(MathUtil.clamp((output + ff) * 12.0, -12.0, 12.0));
            }
        } else {
            // Move down a little bit to zero
            elevatorMotorOne.set(-0.05);
            elevatorMotorTwo.set(-0.05);
        }
    }

    @Override
    public double getMeasurement() {
        return Robot.isSimulation() ? simulation.getPositionMeters() : elevatorEncoder.getPosition();
    }

    @Override
    public void periodic() {
        // TODO Auto-generated method stub

        if ((bottomLimit.isPressed()) && (!isZeroed)) {
            elevatorEncoder.setPosition(0.0);
            setGoal(0.0);
            isZeroed = true;
        }

        super.periodic();

        SmartDashboard.putNumber("Elevator One Output", elevatorMotorOne.getAppliedOutput());
        SmartDashboard.putNumber("Elevator One GET Output", elevatorMotorOne.get());

        SmartDashboard.putNumber("Elevator Two Output", elevatorMotorTwo.getAppliedOutput());
        SmartDashboard.putNumber("Elevator Two GET Output", elevatorMotorTwo.get());

        SmartDashboard.putNumber("Current Elevator Position", getMeasurement());
        SmartDashboard.putNumber("Goal Elevator Position", this.getController().getGoal().position);
        SmartDashboard.putBoolean("Elevator Zeroed", isZeroed);
        SmartDashboard.putBoolean("Elevator Bottom Limit", bottomLimit.isPressed());

        SmartDashboard.putNumber("Elevator motor 1 encoder", elevatorMotorOne.getEncoder().getPosition());
        SmartDashboard.putNumber("Elevator motor 2 encoder", elevatorMotorTwo.getEncoder().getPosition());

        SmartDashboard.putNumber("Elevator motor 1 current", elevatorMotorOne.getOutputCurrent());
        SmartDashboard.putNumber("Elevator motor 1 output voltage", elevatorMotorOne.getBusVoltage());
    }

    @Override
    public void simulationPeriodic() {
        // TODO Auto-generated method stub
        super.simulationPeriodic();

        // Next, we update it. The standard loop time is 20ms.

        simulation.update(0.020);

        // Finally, we set our simulated encoder's readings and simulated battery
        // voltage

        // SimBattery estimates loaded battery voltages

        RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(simulation.getCurrentDrawAmps()));
    }

    public boolean isInPosition() {
        return this.getController().atGoal();
    }
}