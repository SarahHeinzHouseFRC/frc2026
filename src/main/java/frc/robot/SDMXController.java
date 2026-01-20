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
    private final Map<Integer, Method> analogInputEventHandlers;

    public SDMXController(GenericHID hid) {
        this.hid = hid;

        digitalInputEventHandlers = new HashMap<>();
        analogInputEventHandlers = new HashMap<>();
    }

    public void registerEventHandlers() {
        Reflections reflections = new Reflections("frc.robot", org.reflections.scanners.Scanners.MethodsAnnotated);
        var digitalInputEventHandlerMethods = reflections.getMethodsAnnotatedWith(SDMXDigitalInputEventHandler.class);
        for (Method method : digitalInputEventHandlerMethods) {
            SDMXDigitalInputEventHandler annotation = method.getAnnotation(SDMXDigitalInputEventHandler.class);
            digitalInputEventHandlers.put(annotation.value(), method);
        }
        var analogInputEventHandlerMethods = reflections.getMethodsAnnotatedWith(SDMXAnalogInputEventHandler.class);
        for (Method method : analogInputEventHandlerMethods) {
            SDMXAnalogInputEventHandler annotation = method.getAnnotation(SDMXAnalogInputEventHandler.class);
            analogInputEventHandlers.put(annotation.value(), method);
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
        for (Integer key : analogInputEventHandlers.keySet()) {
            analogInputEventHandlers.get(key).invoke(null, (byte) Math.round(Math.max(0.0, Math.min(1.0, hid.getRawAxis(key))) * 255.0));
        }
    }
}
