package com.bridou_n.beaconscanner.features.beaconList;

import android.content.Intent;
import android.os.Bundle;
import android.support.design.widget.Snackbar;
import android.support.v7.app.AppCompatActivity;
import android.support.v7.widget.Toolbar;
import android.view.View;
import android.widget.Button;
import android.widget.EditText;

import com.bridou_n.beaconscanner.R;

//스쿼트 설정 화면
public class squartSetting extends AppCompatActivity {

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_squart_setting);

        final EditText numtext3 = (EditText) findViewById(R.id.getNumText3); //운동 횟수
        final EditText timetext3 = (EditText) findViewById(R.id.getTimeText3); //운동 유지 시간

        Toolbar toolbar = (Toolbar) findViewById(R.id.toolbar);
        setSupportActionBar(toolbar);
        Button nextBtn = (Button) findViewById(R.id.nextButton999);

        nextBtn.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View view) {
                String E_NUM3, E_TIME3, P_TIME3;
                E_NUM3 = numtext3.getText().toString();
                E_TIME3 = timetext3.getText().toString();

                //설정값 다음 화면에 넘겨주기
                Intent intentSubActivity = new Intent(squartSetting.this, SquartActivity.class);
                intentSubActivity.putExtra("E_TIME3", E_TIME3);
                intentSubActivity.putExtra("E_NUM3", E_NUM3);
                startActivity(intentSubActivity);
            }
        });
    }
}