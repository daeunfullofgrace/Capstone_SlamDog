package com.github.rosjava.android_apps.make_a_map;

import android.content.Context;
import android.widget.Toast;

import com.github.rosjava.android_apps.make_a_map.ros_msgs_tutorial.srvTutorial;
import com.github.rosjava.android_apps.make_a_map.ros_msgs_tutorial.srvTutorialRequest;
import com.github.rosjava.android_apps.make_a_map.ros_msgs_tutorial.srvTutorialResponse;

import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.service.ServiceResponseBuilder;

public class Server extends AbstractNodeMain {
    Context context;

    @Override
    public GraphName getDefaultNodeName() {
        return GraphName.of("ros_tutorial_srv/server");
    }

    @Override
    public void onStart(final ConnectedNode connectedNode) {
        connectedNode.newServiceServer("ros_tutorial_srv", srvTutorial._TYPE,
                new ServiceResponseBuilder<srvTutorialRequest, srvTutorialResponse>() {

                    @Override
                    public void
                    build(srvTutorialRequest request, srvTutorialResponse response) {
                        response.setSum(request.getA() + request.getB());
                        connectedNode.getLog().info(
                                String.format("send srv, srv.Request.a and b : %1d, %1d", request.getA(), request.getB()));
                    }
                });
    }

    public void msg(String str){
        Toast.makeText(context, str, Toast.LENGTH_SHORT).show();
    }
}
