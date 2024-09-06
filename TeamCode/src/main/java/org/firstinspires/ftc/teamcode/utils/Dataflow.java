package org.firstinspires.ftc.teamcode.utils;

import com.acmerobotics.dashboard.FtcDashboard;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Dataflow {
    FtcDashboard dashboard = FtcDashboard.getInstance(); //create objects for ftcdashboard usage and telemetry
    Telemetry dashboardTelemetry = dashboard.getTelemetry();
    Telemetry telemetry; 

    public Dataflow(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    public void addDatasToDashBoard(String[] contexts, Object... values) { //add telemetry datas to send to ftcdashboard
        contexts = (contexts == null || contexts.length == 0) ? new String[]{" "} : contexts; //create an array for telemetry datas
        for (int i = 0; i < values.length; i++) {
            String context = (i < contexts.length && contexts[i] != null && !contexts[i].isEmpty()) ? contexts[i] : "Unknown param";
            dashboardTelemetry.addData(context, values[i]);  //add datas to ftc dashboard
        }
    }

    public void addDatasToTelemetry(String[] contexts, Object... values) { //send telemetry to the driver hub
        contexts = (contexts == null || contexts.length == 0) ? new String[]{" "} : contexts;
        for (int i = 0; i < values.length; i++) {
            String context = (i < contexts.length && contexts[i] != null && !contexts[i].isEmpty()) ? contexts[i] : "Unknown param";
            telemetry.addData(context, values[i]);
        }
    }

    public void addToAll(String[] contexts, Object... values) { //add datas to both driver hub and ftcdashboard
        addDatasToTelemetry(contexts, values);
        addDatasToDashBoard(contexts, values);
    }
    public void sendDatas() { //update the telemetry datas
        telemetry.update();
        dashboardTelemetry.update();
    }
}
