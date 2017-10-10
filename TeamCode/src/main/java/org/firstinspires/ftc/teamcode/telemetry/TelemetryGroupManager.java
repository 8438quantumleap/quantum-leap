package org.firstinspires.ftc.teamcode.telemetry;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Iterator;
import java.util.Map;

/**
 * Manages telemetry groups and updating telemetry
 */

public class TelemetryGroupManager {

    private ArrayList<TelemetryGroup> telemetryGroups = new ArrayList<TelemetryGroup>();
    private Telemetry telemetry;

    public TelemetryGroupManager(Telemetry telemetry){
        this.telemetry = telemetry;
        telemetry.setAutoClear(false);
    }

    public void addTelemetryGroup(TelemetryGroup telemetryGroup){
        telemetryGroups.add(telemetryGroup);
    }

    public void update(){
        for(TelemetryGroup group : telemetryGroups){
            telemetry.addData(group.getName(), "");
            for (Map.Entry<String, String> pair : group.getDataPairs().entrySet()) {
                telemetry.addData("\t" + pair.getKey(), pair.getValue());
            }
        }
        telemetry.update();
    }
}
