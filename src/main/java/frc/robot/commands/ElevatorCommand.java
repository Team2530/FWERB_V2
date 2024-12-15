package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.ElevatorSubsystem;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class ElevatorCommand extends Command {
    private final ElevatorSubsystem elevatorSub;

    public enum ElevatorPresets {
        STOW(0.0),
        MIDDLE(Constants.Elevator.PhysicalParameters.elevatorHeightMeters / 2),
        TOP(Constants.Elevator.PhysicalParameters.elevatorHeightMeters);

        private ElevatorPresets(double pos_meters) {
            this.position_m = pos_meters;
        }

        private double position_m;
    }

    private ElevatorPresets target = ElevatorPresets.STOW;

    public ElevatorCommand(ElevatorSubsystem elevatorSub, ElevatorPresets targetPosition) {
        this.elevatorSub = elevatorSub;
        this.target = targetPosition;
        addRequirements(elevatorSub);
    }

    @Override
    public void initialize() {
        elevatorSub.setGoal(target.position_m);
        SmartDashboard.putString("Elevator Command", target.toString());
    }

    @Override
    public boolean isFinished() {
        return elevatorSub.isInPosition();
    }

    @Override
    public void end(boolean interrupted) {
        // NOTE: Don't disable (so it position-holds with feedforward)
        // elevatorSub.disable();
    }
}
