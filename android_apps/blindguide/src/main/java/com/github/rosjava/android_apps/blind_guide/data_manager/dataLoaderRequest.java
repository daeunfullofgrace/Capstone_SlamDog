package com.github.rosjava.android_apps.blind_guide.data_manager;

public interface dataLoaderRequest extends org.ros.internal.message.Message {
    java.lang.String _TYPE = "com.github.rosjava.android_apps.blind_guide/data_manager/dataLoaderRequest";
    java.lang.String _DEFINITION = "string a\n";

    String getA();

    void setA(String l);
}
