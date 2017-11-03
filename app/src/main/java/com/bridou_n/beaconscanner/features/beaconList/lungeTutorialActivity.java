package com.bridou_n.beaconscanner.features.beaconList;

import android.content.Intent;
import android.media.MediaPlayer;
import android.net.Uri;
import android.os.Bundle;
import android.support.v7.app.AppCompatActivity;
import android.support.v7.widget.Toolbar;
import android.view.View;
import android.widget.Button;
import android.widget.Toast;
import android.widget.VideoView;

import com.bridou_n.beaconscanner.R;

//런지 튜토리얼 화면
public class lungeTutorialActivity extends AppCompatActivity {
    private VideoView videoView;

    @Override
    protected void onCreate(Bundle saveInstanceState) {
        super.onCreate(saveInstanceState);
        setContentView(R.layout.activity_lungevideo);

        Toolbar toolbar = (Toolbar) findViewById(R.id.toolbar);
        setSupportActionBar(toolbar);

        Button btnStart, btnStop;
        Button btnNext;
        btnNext = (Button) findViewById(R.id.btnNext);

        videoView = (VideoView) findViewById(R.id.tutorialVideoView);

        videoView.setVideoURI(Uri.parse("android.resource://" + getPackageName() + "/" + R.raw.crunch_tutorial)); //동영상 파일 주소

        videoView.setMediaController(new android.widget.MediaController(this));
        videoView.requestFocus();

        //동영상 재생 완료 나타내기
        videoView.setOnCompletionListener(new MediaPlayer.OnCompletionListener() {
            @Override
            public void onCompletion(MediaPlayer mp) {
                Toast.makeText(lungeTutorialActivity.this,
                        "동영상 재생이 완료되었습니다.", Toast.LENGTH_SHORT).show();
            }
        });
    }

    private void playVideo() { //동영상 재생
        videoView.seekTo(0);
        videoView.start();
    }

    private void stopVideo() { //동영상 정지
        videoView.pause();
        videoView.stopPlayback();
    }

    public void StartButton(View v) {
        playVideo();
    }

    public void StopButton(View v) {
        stopVideo();
    }

    public void NextButton(View v) { //다음 화면 이동
        Intent intent = new Intent(lungeTutorialActivity.this, lungeSetting.class);
        startActivity(intent);
    }
}
