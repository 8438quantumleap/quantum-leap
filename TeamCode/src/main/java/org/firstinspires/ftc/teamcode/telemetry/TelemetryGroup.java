package org.firstinspires.ftc.teamcode.telemetry;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map;

/**
 * This class allows grouping of telemetry data
 */

public class TelemetryGroup {

    private String name;
    //private ArrayList<String> items = new ArrayList<String>();
    //Format: Caption, value
    private HashMap<String, String> dataPairs = new HashMap<String, String>();
    private ArrayList<TelemetryGroup> subgroups = new ArrayList<TelemetryGroup>();

    public TelemetryGroup(String groupName){
        this.name = groupName;
    }

    public void addSubgroup(TelemetryGroup subgroup){
        subgroups.add(subgroup);
    }

    public void setData(String name, String value){
        dataPairs.put(name, value);
    }

    String getName() {
        return name;
    }

    HashMap<String, String> getDataPairs(){
        if(!subgroups.isEmpty()){
            for(TelemetryGroup group : subgroups){
                this.dataPairs.put(group.getName(), "");
                for(Map.Entry<String, String> entry: group.getDataPairs().entrySet()){
                    this.dataPairs.put("\t" + entry.getKey(), entry.getValue());
                }
            }
        }
        return this.dataPairs;
    }
}
