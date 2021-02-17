package com.github.rosjava.android_apps.blind_guide;

import android.Manifest;
import android.annotation.SuppressLint;
import android.content.Intent;
import android.content.SharedPreferences;
import android.os.Build;
import android.os.Bundle;
import android.os.Handler;
import android.preference.PreferenceManager;
import android.support.annotation.RequiresApi;
import android.support.v4.app.ActivityCompat;
import android.util.Log;
import android.view.View;
import android.widget.Button;
import android.widget.RelativeLayout;
import android.widget.TextView;
import android.widget.Toast;

import com.github.rosjava.android_apps.teleop.R;
import com.github.rosjava.android_remocons.common_tools.apps.RosAppActivity;

import org.ros.node.NodeMainExecutor;

import java.net.URISyntaxException;

public class MainActivity extends RosAppActivity {

	Intent intent;

	RelativeLayout listLayout;
	private Button startBtn;
	private Button selectBtn;

	private final int PERMISSION = 1;
	private final String MASTER_URI = "http://192.168.0.23:11311";

	private VoiceAssi voiceAssi;
	private TextView statusTxt;
	private String destination;
	private String destinationId;
	private String navigationFlag;
	private NodeMainExecutor nodeMainExecutor;

	private long backBtnTime = 0;

	public MainActivity() throws URISyntaxException {
		super("길안내 어플", "종료");
	}

	@SuppressLint("ClickableViewAccessibility")
	@Override
	public void onCreate(Bundle savedInstanceState) {
		setDashboardResource(R.id.top_bar);
		setMainWindowResource(R.layout.main);
		super.onCreate(savedInstanceState);

		voiceAssi = new VoiceAssi(intent, MainActivity.this);
		voiceAssi.startReading(",,,,");
		statusTxt = findViewById(R.id.status);

		destination = getIntent().getStringExtra("destination");
		destinationId = getIntent().getStringExtra("destinationId");
		navigationFlag = getIntent().getStringExtra("navigation");

		if(destination != null){
			statusTxt.setText("목적지 : " + destination);
		} else if (navigationFlag != null) {
			statusTxt.setText("네비게이션 시작");
		}

		startBtn = (Button) findViewById(R.id.start_btn);
		selectBtn = (Button) findViewById(R.id.select_btn);
		listLayout = (RelativeLayout) findViewById(R.id.list_layout);

		if(Build.VERSION.SDK_INT >= 23){
			// 퍼미션 체크
			ActivityCompat.requestPermissions(this, new String[]{Manifest.permission.INTERNET,
					Manifest.permission.RECORD_AUDIO},PERMISSION);
		}

		selectBtn.setOnClickListener(new View.OnClickListener() {
			@Override
			public void onClick(View view) {
				voiceAssi.destroyTTS();
				Intent intent1 = new Intent(MainActivity.this, SelectActivity.class);
				MainActivity.this.startActivity(intent1);
				finish();
			}
		});

		startBtn.setOnClickListener(new View.OnClickListener() {
			@Override
			public void onClick(View view) {
				if(statusTxt.getText().toString().equals("목적지 선택 필요")) {
					voiceAssi.startReading("설정된 목적지가 없습니다");
					Toast.makeText(MainActivity.this, "설정된 목적지가 없습니다",Toast.LENGTH_SHORT).show();
				} else {
					voiceAssi.destroyTTS();
					Intent intent1 = new Intent(MainActivity.this, NavigationActivity.class);
					if(destination != null){
						intent1.putExtra("destination", destination);
						intent1.putExtra("destinationId", destinationId);
					}
					MainActivity.this.startActivity(intent1);
					finish();
				}
			}
		});

		new Handler().postDelayed(new Runnable() {
			@Override
			public void run() {
				try {
					voiceAssi.startReading("메인 화면입니다." +statusTxt.getText().toString());
				} catch (Exception e) {
					e.printStackTrace();
				}
			}
		}, 100);
	}

	@Override
	public void onStart() {
		voiceAssi.startReading("메인 화면입니다." + statusTxt.getText().toString());
		super.onStart();
	}

	@Override
	public void onDestroy() {
		super.onDestroy();
		this.voiceAssi = null;
	}

	@Override
	public void startMasterChooser(){
		try{
			SharedPreferences prefs = PreferenceManager.getDefaultSharedPreferences(getBaseContext());
			String masterURI = prefs.getString("masterURI", MASTER_URI);

			Intent data = new Intent();
			data.putExtra("ROS_MASTER_URI", masterURI);
			onActivityResult(0, RESULT_OK, data);

			new Handler().postDelayed(new Runnable() {
				@Override
				public void run() {
					try {
						voiceAssi.startReading("메인 화면입니다." +statusTxt.getText().toString());
					} catch (Exception e) {
						e.printStackTrace();
					}
				}
			}, 50);

		} catch (Exception e){
			Toast.makeText(MainActivity.this, "서버와 연결 실패", Toast.LENGTH_SHORT).show();
			Log.e("Connection Error", "Master can't reachable");
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