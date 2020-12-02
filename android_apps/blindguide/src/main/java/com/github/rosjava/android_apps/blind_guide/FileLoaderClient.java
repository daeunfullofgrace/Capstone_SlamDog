package com.github.rosjava.android_apps.blind_guide;

import android.util.Log;

import com.github.rosjava.android_apps.blind_guide.data_manager.dataLoader;
import com.github.rosjava.android_apps.blind_guide.data_manager.dataLoaderRequest;
import com.github.rosjava.android_apps.blind_guide.data_manager.dataLoaderResponse;

import org.ros.exception.RemoteException;
import org.ros.exception.RosRuntimeException;
import org.ros.exception.ServiceNotFoundException;
import org.ros.namespace.GraphName;
import org.ros.namespace.NameResolver;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.service.ServiceClient;
import org.ros.node.service.ServiceResponseListener;

class FileLoaderClient extends AbstractNodeMain {

    private String result;
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

        ServiceClient<dataLoaderRequest, dataLoaderResponse> serviceClient;

        try {
            serviceClient = connectedNode.newServiceClient("location_data", dataLoader._TYPE);
        } catch (ServiceNotFoundException e) {
            throw new RosRuntimeException(e);
        }

        final dataLoaderRequest request = serviceClient.newMessage();
        request.setA("client");
        serviceClient.call(request, new ServiceResponseListener<dataLoaderResponse>() {

            @Override
            public void onSuccess(dataLoaderResponse response) {
                connectedNode.getLog().info(
                        String.format("%s", request.getA()));

                Log.d("service result",response.getResult());
                result = response.getResult();
            }

            @Override
            public void onFailure(RemoteException e) {
                throw new RosRuntimeException(e);
            }
        });
    }

    @Override
    public GraphName getDefaultNodeName() {
        return GraphName.of("file_loader_service/client");
    }

    public String getDestinationList(){
//        String[] splitResult = result.split("[%\\n]");
//        StringBuilder summed = new StringBuilder();
//        int idx=0;

//        for (String s : splitResult) {
//            System.out.println(idx++);
//            Log.w("destination", s);
//        }
//
//        for(int i=0; i<splitResult.length; i+=2){
//            if(i == 2) --i;
//            summed.append(splitResult[i]);
//
//            if(i>0 && i<splitResult.length-1) summed.append(", ");
//        }
//
//        Log.d("summed", summed.toString());
//        return summed.toString();

        String str = result.replace("%", "").replace(System.getProperty("line.separator"), ",,,,,,,");

        return str;
    }

    public String getDestinationId(String target){
        String[] splitResult = result.split("[\\n%]");

        for(int i=0; i<splitResult.length; i++){
            if(target.equals(splitResult[i])){
                return Integer.toString(i/2);
            }
        }
        return Integer.toString(-1);
    }
}
