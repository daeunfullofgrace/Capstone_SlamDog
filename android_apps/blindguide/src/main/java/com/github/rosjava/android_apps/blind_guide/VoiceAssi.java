package com.github.rosjava.android_apps.blind_guide;

import android.app.Activity;
import android.content.Context;
import android.content.Intent;
import android.os.Bundle;
import android.speech.RecognitionListener;
import android.speech.RecognizerIntent;
import android.speech.SpeechRecognizer;
import android.speech.tts.TextToSpeech;
import android.util.Log;
import android.widget.EditText;
import android.widget.Toast;

import java.util.ArrayList;
import java.util.Locale;


public class VoiceAssi extends Activity {

    Intent intent;
    static SpeechRecognizer mRecognizer;
    static TextToSpeech mGenerator;
    Context mContext;
    
    EditText editText;

    final int PERMISSION = 1;

    public VoiceAssi(Intent intent, Context mContext) {
        this.intent = intent;
        this.mContext = mContext;

        this.initReading();
        this.initListening();
    }

    public VoiceAssi(Intent intent, Context mContext, EditText editText) {
        this.intent = intent;
        this.mContext = mContext;
        this.editText = editText;

        this.initReading();
        this.initListening();
    }

    public void initListening(){
        intent = new Intent(RecognizerIntent.ACTION_RECOGNIZE_SPEECH);
        intent.putExtra(RecognizerIntent.EXTRA_CALLING_PACKAGE,getPackageName());
        intent.putExtra(RecognizerIntent.EXTRA_LANGUAGE, "ko-KR");

        mRecognizer = SpeechRecognizer.createSpeechRecognizer(mContext);
        mRecognizer.setRecognitionListener(listener);
    }

    public void startListening(){
        mRecognizer.startListening(intent);
    }

    public void initReading(){
        mGenerator = new TextToSpeech(mContext, new TextToSpeech.OnInitListener() {
            @Override
            public void onInit(int status) {
                if (status == TextToSpeech.SUCCESS) {
                    mGenerator.setLanguage(Locale.KOREAN);
                    mGenerator.setSpeechRate(0.9f);
                    mGenerator.setPitch(1f);
                }
            }
        });
    }

    public void startReadingAf(String msg){
        initReading();
        startReading(msg);
    }

    public void destroyTTS(){
        if(mGenerator != null){
            mGenerator.stop();
            mGenerator.shutdown();
            mGenerator = null;
        }
    }

    public void startReading(String msg){
        mGenerator.speak(msg,TextToSpeech.QUEUE_FLUSH, null);
    }

    @Override
    protected void onDestroy() {
        super.onDestroy();
        // TTS 객체가 남아있다면 실행을 중지하고 메모리에서 제거한다.
        if(mGenerator != null){
            mGenerator.stop();
            mGenerator.shutdown();
            mGenerator = null;
        }
    }

    @Override
    public void onPause() {
        super.onPause();
        if(mGenerator != null){
            mGenerator.stop();
            mGenerator.shutdown();
            mGenerator = null;
        }
    }

    private RecognitionListener listener = new RecognitionListener() {
        @Override
        public void onReadyForSpeech(Bundle params) {
            Toast.makeText(mContext, "음성인식을 시작합니다.", Toast.LENGTH_SHORT).show();
        }

        @Override
        public void onBeginningOfSpeech() {
        }

        @Override
        public void onRmsChanged(float rmsdB) {
        }

        @Override
        public void onBufferReceived(byte[] buffer) {
        }

        @Override
        public void onEndOfSpeech() {
        }

        @Override
        public void onError(int error) {
            String message;

            switch (error) {
                case SpeechRecognizer.ERROR_AUDIO:
                    message = "오디오 에러";
                    break;
                case SpeechRecognizer.ERROR_CLIENT:
                    message = "클라이언트 에러";
                    break;
                case SpeechRecognizer.ERROR_INSUFFICIENT_PERMISSIONS:
                    message = "퍼미션 없음";
                    break;
                case SpeechRecognizer.ERROR_NETWORK:
                    message = "네트워크 에러";
                    break;
                case SpeechRecognizer.ERROR_NETWORK_TIMEOUT:
                    message = "네트웍 타임아웃";
                    break;
                case SpeechRecognizer.ERROR_NO_MATCH:
                    message = "찾을 수 없음";
                    break;
                case SpeechRecognizer.ERROR_RECOGNIZER_BUSY:
                    message = "RECOGNIZER가 바쁨";
                    break;
                case SpeechRecognizer.ERROR_SERVER:
                    message = "서버가 이상함";
                    break;
                case SpeechRecognizer.ERROR_SPEECH_TIMEOUT:
                    message = "말하는 시간초과";
                    break;
                default:
                    message = "알 수 없는 오류임";
                    break;
            }

            Toast.makeText(mContext, "에러 발생 : " + message, Toast.LENGTH_SHORT).show();
        }

        @Override
        public void onResults(Bundle bundle) {
            
            ArrayList<String> matches =
                    bundle.getStringArrayList(SpeechRecognizer.RESULTS_RECOGNITION);

            String listeningResult = matches.toString();
            String[] splitedResult = listeningResult.split("\\[|]|,");

            if(editText != null){
                editText.setText(splitedResult[1]);
                Log.e("Set EditTxt", splitedResult[1]);
            }
        }

        @Override
        public void onPartialResults(Bundle bundle) {

        }

        @Override
        public void onEvent(int i, Bundle bundle) {

        }
    };
}
