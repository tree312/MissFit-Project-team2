package com.bridou_n.beaconscanner.features.beaconList;

import android.content.Intent;
import android.net.Uri;
import android.os.Bundle;
import android.support.v7.app.AppCompatActivity;
import android.support.v7.widget.Toolbar;
import android.view.View;
import android.widget.Button;
import android.widget.VideoView;

import com.bridou_n.beaconscanner.R;

public class ropeTutorialActivity extends AppCompatActivity {

    private VideoView videoView;

    @Override
    protected void onCreate(Bundle saveInstanceState) {
        super.onCreate(saveInstanceState);
        setContentView(R.layout.activity_ropevideo);

        Toolbar toolbar = (Toolbar) findViewById(R.id.toolbar);
        setSupportActionBar(toolbar);

        Button btnStart, btnStop;
        Button btnNext;
        btnNext = (Button) findViewById(R.id.btnNext);

        videoView = (VideoView) findViewById(R.id.tutorialVideoView);
        videoView.setVideoURI(Uri.parse("android.resource://" + getPackageName() + "/" + R.raw.crunch_tutorial)); //동영상 파일 주소
        videoView.setMediaController(new android.widget.MediaController(this));
        videoView.requestFocus();
    }

    private void stopVideo() { //동영상 정지
        videoView.pause();
        videoView.stopPlayback();
    }

    public void NextButton(View v) { //메인화면으로 이동
        Intent intent = new Intent(ropeTutorialActivity.this, MainActivity.class);
        startActivity(intent);
    }
}
