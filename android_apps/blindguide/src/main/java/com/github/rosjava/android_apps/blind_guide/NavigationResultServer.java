package com.github.rosjava.android_apps.blind_guide;

import android.util.Log;

import com.github.rosjava.android_apps.blind_guide.slamdog_navi.arrivalType;
import com.github.rosjava.android_apps.blind_guide.slamdog_navi.arrivalTypeRequest;
import com.github.rosjava.android_apps.blind_guide.slamdog_navi.arrivalTypeResponse;

import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.service.ServiceResponseBuilder;

public class NavigationResultServer extends AbstractNodeMain {

    private static long res = -1;
    private static boolean isAlive;

    @Override
    public GraphName getDefaultNodeName() {
        return null;
    }

    @Override
    public void onStart(final ConnectedNode connectedNode) {
        isAlive = true;

        connectedNode.newServiceServer("slamdog_navi_result", arrivalType._TYPE,
                new ServiceResponseBuilder<arrivalTypeRequest, arrivalTypeResponse>() {
                    @Override
                    public void build(arrivalTypeRequest request, arrivalTypeResponse response) {
                        res = request.getType();
                        response.setResult(res);

                        connectedNode.getLog().info("Received :  " + res);
                        Log.e("service result", String.valueOf(res));
                    }
                });
    }

    public static long getResponse(){
//        Log.d("return response", String.valueOf(res));
        return res;
    }

    public static void initResponse(){
        res = -1;
    }

    public static boolean isAlive(){
        return isAlive;
    }
}
