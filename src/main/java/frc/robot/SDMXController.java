package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;

public class SDMXController extends GenericHID {
    /**
     * Construct an instance of a device.
     *
     * @param port The port index on the Driver Station that the device is plugged into.
     */
    public SDMXController(int port) {
        super(port);
    }
}
