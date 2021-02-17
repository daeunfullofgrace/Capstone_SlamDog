package com.github.rosjava.android_apps.blind_guide.slamdog_navi;

public interface arrivalTypeRequest extends org.ros.internal.message.Message {
    java.lang.String _TYPE = "com.github.rosjava.android_apps.blind_guide/slamdog_navi/arrivalTypeRequest";
    java.lang.String _DEFINITION = "int64 type\n";

    long getType();

    void setType(long l);
}
