package com.github.rosjava.android_apps.blind_guide;

import android.annotation.SuppressLint;
import android.util.Log;

import com.github.rosjava.android_apps.blind_guide.data_manager.*;

import org.ros.exception.RemoteException;
import org.ros.exception.RosRuntimeException;
import org.ros.exception.ServiceNotFoundException;
import org.ros.namespace.GraphName;
import org.ros.namespace.NameResolver;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.service.ServiceClient;
import org.ros.node.service.ServiceResponseListener;

class SetPoseClient extends AbstractNodeMain {

    private boolean isSuccessed;

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
        this.isSuccessed = false;

        ServiceClient<setPoseRequest, setPoseResponse> serviceClient;

        try {
            serviceClient = connectedNode.newServiceClient("pose_initialization", setPose._TYPE);
        } catch (ServiceNotFoundException e) {
            throw new RosRuntimeException(e);
        }

        final setPoseRequest request = serviceClient.newMessage();
        request.setA(Integer.toString(0));
        serviceClient.call(request, new ServiceResponseListener<setPoseResponse>() {
            @SuppressLint("DefaultLocale")
            @Override
            public void onSuccess(setPoseResponse response) {
                Log.d("input : ", request.getA());
                connectedNode.getLog().info(
                        String.format("%s", request.getA()));

                Log.d("pose service result", response.getResult());
                isSuccessed = true;
            }

            @Override
            public void onFailure(RemoteException e) {
                throw new RosRuntimeException(e);
            }
        });
    }

    @Override
    public GraphName getDefaultNodeName() {
        return GraphName.of("robot_setting_service/client");
    }

    public boolean isSuccessed(){
        return isSuccessed;
    }
}
