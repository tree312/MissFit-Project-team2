package com.bridou_n.beaconscanner.features.beaconList;

import android.content.Intent;
import android.os.Bundle;
import android.support.v7.app.AppCompatActivity;
import android.view.View;
import android.widget.ImageButton;

import com.bridou_n.beaconscanner.R;

//앱 첫 화면
public class IntroActivity extends AppCompatActivity {
    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_intro);

        ImageButton wholeButton = (ImageButton) findViewById(R.id.viewButton);
        ImageButton centerButton = (ImageButton) findViewById(R.id.centerButton);

    }

    public void introClick(View view) { // 캘린더 화면으로 이동
        Intent intent = new Intent(IntroActivity.this, CalendarActivity.class);
        startActivity(intent);
        // 뒤로가기 했을경우 안나오도록 없애주기 >> finish
        finish();
    }
}
