package com.github.rosjava.android_apps.blind_guide;

import android.content.Intent;
import android.os.Build;
import android.os.Bundle;
import android.os.Handler;
import android.support.annotation.RequiresApi;
import android.util.Log;
import android.view.View;
import android.widget.Button;
import android.widget.EditText;
import android.widget.LinearLayout;
import android.widget.RelativeLayout;
import android.widget.Toast;

import com.github.rosjava.android_apps.teleop.R;
import com.github.rosjava.android_remocons.common_tools.apps.RosAppActivity;

import org.ros.address.InetAddressFactory;
import org.ros.node.NodeConfiguration;
import org.ros.node.NodeMainExecutor;

public class SelectActivity extends RosAppActivity {

    Button micBtn;
    LinearLayout submitBtn;
    RelativeLayout listLayout;
    VoiceAssi voiceAssi;
    EditText targetTxt;

    Intent intent;
    FileLoaderClient fileLoaderClient;
    SetPoseClient poseClient;
    NodeMainExecutor nodeMainExecutor;
    NodeConfiguration nodeConfiguration;

    String destinationList;
    LinearLayout backLayout;

    private long backBtnTime = 0;

    public SelectActivity(){
        super("길안내 어플", "종료 버튼");
    }

    @Override
    public void onCreate(Bundle savedInstanceState) {
        setMainWindowResource(R.layout.select_view);
        setDashboardResource(R.id.top_bar);
        super.onCreate(savedInstanceState);

        targetTxt = findViewById(R.id.target_txt);

        voiceAssi = new VoiceAssi(intent, SelectActivity.this, targetTxt);
        voiceAssi.startReading(",,,,");

        micBtn = findViewById(R.id.mic_btn);
        micBtn.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View view) {
                voiceAssi.startListening();
            }
        });

        fileLoaderClient = new FileLoaderClient();
        poseClient = new SetPoseClient();

        new Handler().postDelayed(new Runnable() {
            @Override
            public void run() {
                nodeMainExecutor.execute(fileLoaderClient, nodeConfiguration.setNodeName("android/data_loader_client"));
//                nodeMainExecutor.execute(poseClient, nodeConfiguration.setNodeName("android/pose_initialization"));
            }
        }, 800);

        listLayout = findViewById(R.id.list_layout);
        listLayout.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View view) {
//                voiceAssi.destroyTTS();
//                nodeMainExecutor.execute(fileLoaderClient, nodeConfiguration.setNodeName("android/data_loader_client"));

                new Handler().postDelayed(new Runnable() {
                    @Override
                    public void run() {
//                        fileLoaderClient = new FileLoaderClient();
                        destinationList = fileLoaderClient.getDestinationList();
                        Toast.makeText(SelectActivity.this, "목적지 목록 : " + destinationList, Toast.LENGTH_SHORT).show();
                        voiceAssi.startReading("목적지 목록 : " + destinationList);
                    }
                }, 800);
            }
        });

        submitBtn = findViewById(R.id.submit_btn);
        submitBtn.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View view) {
                if(targetTxt.getText().toString().equals("")){
                    voiceAssi.startReading("설정한 목적지 없음");
                    Toast.makeText(SelectActivity.this, "설정한 목적지 없음", Toast.LENGTH_SHORT).show();
                } else {
                    if(destinationList == null){
                        destinationList = fileLoaderClient.getDestinationList();
                    }
                    if(destinationList.contains(targetTxt.getText())){
                        voiceAssi.destroyTTS();
                        Log.d("destination", targetTxt.getText().toString());

                        Intent intent1 = new Intent(SelectActivity.this, NavigationActivity.class);
                        intent1.putExtra("destination", targetTxt.getText().toString());
                        intent1.putExtra("destinationId", fileLoaderClient.getDestinationId(targetTxt.getText().toString()));
                        SelectActivity.this.startActivity(intent1);
                        finish();
                    } else {
                        voiceAssi.startReading("일치하는 목적지 없음");
                        Toast.makeText(SelectActivity.this, "일치하는 목적지 없음", Toast.LENGTH_SHORT).show();
                        targetTxt.setText("");
                    }
                }
            }
        });

        backLayout = findViewById(R.id.back);
        backLayout.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View view) {
                voiceAssi.destroyTTS();

                Intent intent1 = new Intent(SelectActivity.this, MainActivity.class);
                SelectActivity.this.startActivity(intent1);
                finish();
            }
        });

        new Handler().postDelayed(new Runnable() {
            @Override
            public void run() {
                try {
                    voiceAssi.startReading("목적지 선택 화면입니다.");
                } catch (Exception e) {
                    e.printStackTrace();
                }
            }
        }, 100);
    }

    @Override
    public void init(NodeMainExecutor nodeMainExecutor){
        this.nodeMainExecutor = nodeMainExecutor;
        this.nodeConfiguration = NodeConfiguration.newPublic(InetAddressFactory
                .newNonLoopback().getHostAddress(), getMasterUri());
    }

    @RequiresApi(api = Build.VERSION_CODES.LOLLIPOP)
    @Override
    public void onBackPressed() {
        //super.onBackPressed();

        long curTime = System.currentTimeMillis();
        long gapTime = curTime - backBtnTime;

        if(0 <= gapTime && 2000 >= gapTime) {
            moveTaskToBack(true);
            finishAndRemoveTask();
            android.os.Process.killProcess(android.os.Process.myPid());
        }
        else {
            backBtnTime = curTime;
            voiceAssi.startReading("한번 더 누르면 종료");
        }
    }
}