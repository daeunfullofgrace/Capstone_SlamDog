package com.github.rosjava.android_apps.blind_guide;

import android.annotation.SuppressLint;
import android.content.Intent;
import android.media.AudioManager;
import android.media.ToneGenerator;
import android.os.Build;
import android.os.Bundle;
import android.os.Handler;
import android.support.annotation.RequiresApi;
import android.util.Log;
import android.view.View;
import android.widget.LinearLayout;
import android.widget.TextView;

import com.github.rosjava.android_apps.teleop.R;
import com.github.rosjava.android_remocons.common_tools.apps.RosAppActivity;

import org.ros.address.InetAddressFactory;
import org.ros.node.NodeConfiguration;
import org.ros.node.NodeMainExecutor;

public class NavigationActivity extends RosAppActivity {

    NodeMainExecutor nodeMainExecutor;
    NodeConfiguration nodeConfiguration;

    TextView statusTxt;
    TextView backTxt;

    String destination;
    String destinationId;
    LinearLayout submitBtn;
    LinearLayout backLayout;

    VoiceAssi voiceAssi;
    Intent intent;

    RobotSettingClient client;
    NavigationResultServer resultServer;
    WaitingThread waitingThread;

    int delay = 800;
    private long backBtnTime = 0;

    public NavigationActivity() {
        super("길안내 어플", "종료 버튼");
    }

    @SuppressLint("SetTextI18n")
    @Override
    public void onCreate(Bundle savedInstanceState) {
        setMainWindowResource(R.layout.navigation_view);
        setDashboardResource(R.id.top_bar);
        super.onCreate(savedInstanceState);

        statusTxt = findViewById(R.id.status_txt);
        backTxt = findViewById(R.id.back_txt);
        submitBtn = findViewById(R.id.submit_btn);

        voiceAssi = new VoiceAssi(intent, NavigationActivity.this);
        voiceAssi.startReading(",,,,");

        destination = getIntent().getStringExtra("destination");
        destinationId = getIntent().getStringExtra("destinationId");

        waitingThread = new WaitingThread();

        if(!NavigationResultServer.isAlive()){
            resultServer = new NavigationResultServer();
            new Handler().postDelayed(new Runnable() {
                @Override
                public void run() {
                    try {
                        nodeMainExecutor.execute(resultServer, nodeConfiguration.setNodeName("android/slamdog_navi_result_server"));
                        voiceAssi.startReading("안내 시작");
                    } catch (Exception e) {
                        Log.d("error", String.valueOf(e));
                    }
                }
            }, 50);
        }

        if(destination != null){
            statusTxt.setText("목적지 : " + destination + "\n아이디 : " + destinationId);
        }

        submitBtn.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View view) {
                if(destination != null){
                    client = new RobotSettingClient(Integer.parseInt(destinationId));
                    voiceAssi.destroyTTS();
                    NavigationResultServer.initResponse();

                    statusTxt.setText("이동중");
                    backLayout.setClickable(false);
                    backLayout.setVisibility(View.INVISIBLE);
                    submitBtn.setVisibility(View.INVISIBLE);

                    new Handler().postDelayed(new Runnable() {
                        @Override
                        public void run() {
                            try{
                                nodeMainExecutor.execute(client, nodeConfiguration.setNodeName("android/data_loader_client"));
                                waitingThread.run();
                            } catch (Exception e){
                                Log.d("error", String.valueOf(e));
                            }
                        }
                    }, 800);
                }
            }
        });

        backLayout = findViewById(R.id.back);
        backLayout.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View view) {
                voiceAssi.destroyTTS();

                Intent intent1 = new Intent(NavigationActivity.this, MainActivity.class);
                intent1.putExtra("destination", destination);
                intent1.putExtra("destinationId", destinationId);
                NavigationActivity.this.startActivity(intent1);
                finish();
            }
        });

        new Handler().postDelayed(new Runnable() {
            @Override
            public void run() {
                try {
                    if(destination != null) {
                        client = new RobotSettingClient(Integer.parseInt(destinationId));
                        voiceAssi.destroyTTS();
                        NavigationResultServer.initResponse();

                        nodeMainExecutor.execute(client, nodeConfiguration.setNodeName("android/navigation_ctrl"));
                        waitingThread.run();
                    }
                } catch (Exception e) {
                    e.printStackTrace();
                }
            }
        }, 2000);

        backLayout.setClickable(false);
        backLayout.setVisibility(View.INVISIBLE);
        submitBtn.setVisibility(View.INVISIBLE);
    }

    @Override
    public void onStart() {
        voiceAssi.startReading(statusTxt.getText().toString());
        super.onStart();
    }

    @Override
    public void init(NodeMainExecutor nodeMainExecutor){
        this.nodeMainExecutor = nodeMainExecutor;
        this.nodeConfiguration = NodeConfiguration.newPublic(InetAddressFactory
                .newNonLoopback().getHostAddress(), getMasterUri());
    }

    private class WaitingThread implements Runnable {
        ToneGenerator tone;

        public WaitingThread(){
            tone = new ToneGenerator(AudioManager.STREAM_MUSIC, ToneGenerator.MAX_VOLUME);
        }

        @SuppressLint("SetTextI18n")
        @Override
        public void run() {
            try {
                while (NavigationResultServer.getResponse() < 0) {
//                    Log.d("Result", Long.toString(NavigationResultServer.getResponse()));
                    Thread.sleep(delay);

                    for(int i=0; i<5; i++){
                        tone.startTone(ToneGenerator.TONE_DTMF_0, 50);
                    }
                }
            } catch (Exception e) {
                Log.d("Result", "Error");
                statusTxt.setText("에러 발생");
            } finally {
                Log.d("Result", "Dead");
                Log.d("Result", Long.toString(NavigationResultServer.getResponse()));
                voiceAssi.initReading();

                backLayout.setClickable(true);
                backLayout.setVisibility(View.VISIBLE);

                switch ((int) NavigationResultServer.getResponse()) {
                    case 0:
                        statusTxt.setText("도착");
                        destination = null;
                        destinationId = null;
                        break;
                    case 1:
                        statusTxt.setText("초기화 실패\n" + "목적지 : " + destination);
                        submitBtn.setVisibility(View.VISIBLE);
                        break;
                    case 2:
                        statusTxt.setText("주행 실패\n" + "목적지 : " + destination);
                        submitBtn.setVisibility(View.VISIBLE);
                        break;
                }

                new Handler().postDelayed(new Runnable() {
                    @Override
                    public void run() {
                        try {
                            tone.startTone(ToneGenerator.TONE_DTMF_2, 50);
                            voiceAssi.startReading(statusTxt.getText().toString());
                        } catch (Exception e) {
                            Log.d("error", String.valueOf(e));
                        }
                    }
                }, 200);
            }
        }
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
