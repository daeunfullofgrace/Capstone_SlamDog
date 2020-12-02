package com.github.rosjava.android_apps.blind_guide.data_manager;

public interface setPoseResponse extends org.ros.internal.message.Message {
    java.lang.String _TYPE = "com.github.rosjava.android_apps.blind_guide/data_manager/setPoseResponse";
    java.lang.String _DEFINITION = "string result";

    String getResult();

    void setResult(String s);
}
