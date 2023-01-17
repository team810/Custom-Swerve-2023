package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class subsystemSubsystem extends SubsystemBase {
    private final static subsystemSubsystem INSTANCE = new subsystemSubsystem();

    @SuppressWarnings("WeakerAccess")
    public static subsystemSubsystem getInstance() {
        return INSTANCE;
    }

    private subsystemSubsystem() {

    }
}

