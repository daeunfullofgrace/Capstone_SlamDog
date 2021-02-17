package com.github.rosjava.android_apps.make_a_map.ros_custom_service;

public interface SaveServiceResponseListener<MessageType> {

    void onSuccess(MessageType messageType);

    void onFailure(org.ros.exception.RemoteException e);
}
