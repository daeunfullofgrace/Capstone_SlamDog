package com.github.rosjava.android_apps.blind_guide.slamdog_navi;

public interface arrivalTypeResponse extends org.ros.internal.message.Message {
    java.lang.String _TYPE = "com.github.rosjava.android_apps.blind_guide/slamdog_navi/arrivalTypeResponse";
    java.lang.String _DEFINITION = "int64 result";

    long getResult();

    void setResult(long l);
}
