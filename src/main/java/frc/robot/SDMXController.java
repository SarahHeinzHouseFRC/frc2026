package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import org.reflections.Reflections;

import java.lang.reflect.InvocationTargetException;
import java.lang.reflect.Method;
import java.util.HashMap;
import java.util.Map;

public class SDMXController {
    private final GenericHID hid;
    private final Map<Integer, Method> digitalInputEventHandlers;

    public SDMXController(GenericHID hid) {
        this.hid = hid;

        digitalInputEventHandlers = new HashMap<>();
    }

    public void registerEventHandlers() {
        Reflections reflections = new Reflections("frc.robot", org.reflections.scanners.Scanners.MethodsAnnotated);
        var digitalInputEventHandlerMethods = reflections.getMethodsAnnotatedWith(SDMXDigitalInputEventHandler.class);
        for (Method method : digitalInputEventHandlerMethods) {
            SDMXDigitalInputEventHandler annotation = method.getAnnotation(SDMXDigitalInputEventHandler.class);
            digitalInputEventHandlers.put(annotation.value(), method);
        }
    }

    public void writeDigital(int sdmxChannel, boolean value) {
        // TODO
    }

    public void writeAnalog(int sdmxChannel, byte value) {
        // TODO
    }

    protected void periodic() throws InvocationTargetException, IllegalAccessException {
        for (Integer key : digitalInputEventHandlers.keySet()) {
            if (hid.getRawButtonPressed(key) || hid.getRawButtonReleased(key)) {
                digitalInputEventHandlers.get(key).invoke(null, hid.getRawButton(key));
            }
        }
    }
}
