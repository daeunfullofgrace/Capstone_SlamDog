package com.github.rosjava.android_apps.make_a_map;

import android.content.Context;
import android.util.Log;

import com.github.rosjava.android_apps.make_a_map.ros_msgs_tutorial.srvTutorial;
import com.github.rosjava.android_apps.make_a_map.ros_msgs_tutorial.srvTutorialRequest;
import com.github.rosjava.android_apps.make_a_map.ros_msgs_tutorial.srvTutorialResponse;

import org.ros.exception.RemoteException;
import org.ros.exception.RosRuntimeException;
import org.ros.exception.ServiceNotFoundException;
import org.ros.namespace.GraphName;
import org.ros.namespace.NameResolver;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.service.ServiceClient;
import org.ros.node.service.ServiceResponseListener;

public class Client extends AbstractNodeMain {

    Context context;
    private ConnectedNode connectedNode;
    private NameResolver nameResolver;
    private boolean isNameResolverSet = false;

    public void setNameResolver(NameResolver newNameResolver) {
        nameResolver = newNameResolver;
        isNameResolverSet = true;
    }

    @Override
    public void onStart(final ConnectedNode connectedNode){
        this.connectedNode = connectedNode;

        //test code
        ServiceClient<srvTutorialRequest, srvTutorialResponse> serviceClient;

        try {
            serviceClient = connectedNode.newServiceClient("ros_tutorial_srv", srvTutorial._TYPE);
        } catch (ServiceNotFoundException e) {
            throw new RosRuntimeException(e);
        }

        final srvTutorialRequest request = serviceClient.newMessage();
        request.setA(2);
        request.setB(2);
        serviceClient.call(request, new ServiceResponseListener<srvTutorialResponse>() {

            @Override
            public void onSuccess(srvTutorialResponse response) {
                connectedNode.getLog().info(
                        String.format("%1d, %1d", request.getA(), request.getB()));

                Log.d("client","succeed");
            }

            @Override
            public void onFailure(RemoteException e) {
                throw new RosRuntimeException(e);
            }
        });
    }

    @Override
    public GraphName getDefaultNodeName() {
        return null;
    }
}

