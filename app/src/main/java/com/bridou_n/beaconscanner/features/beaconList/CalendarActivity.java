package com.bridou_n.beaconscanner.features.beaconList;

import android.app.Activity;
import android.app.AlertDialog;
import android.content.DialogInterface;
import android.content.Intent;
import android.content.SharedPreferences;
import android.graphics.Color;
import android.graphics.drawable.Drawable;
import android.os.Bundle;
import android.preference.PreferenceManager;
import android.support.v4.content.ContextCompat;
import android.view.View;
import android.widget.Button;
import android.widget.GridView;
import android.widget.TextView;

import com.bridou_n.beaconscanner.R;

import java.util.ArrayList;
import java.text.SimpleDateFormat;
import java.util.Calendar;
import java.util.Date;
import java.util.List;
import java.util.Locale;

import android.content.Context;
import android.view.LayoutInflater;
import android.view.ViewGroup;
import android.widget.BaseAdapter;
import android.widget.Toast;

//앱 출석 날짜 학인 캘린더 화면
public class CalendarActivity extends Activity {
    private TextView tvDate;
    private GridAdapter gridAdapter;
    private ArrayList<String> dayList;
    private GridView gridView;
    private Calendar mCal;
    Button btnNext;

    boolean mboolean = false;

    @Override
    protected void onCreate(Bundle savedInstanceState) {

        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_calendar);

        tvDate = (TextView) findViewById(R.id.tv_date);
        gridView = (GridView) findViewById(R.id.gridview);
        btnNext = (Button) findViewById(R.id.btnNext);

        //처음 업데이트 시 preference 값 초기화
        SharedPreferences ratePrefs = getSharedPreferences("First Update", 0);
        if (!ratePrefs.getBoolean("FrstTime", false)) {
            setPreferences("Checkdate", "");
            SharedPreferences.Editor edit = ratePrefs.edit();
            edit.putBoolean("FrstTime", true);
            edit.commit();
        }

        // 오늘에 날짜를 세팅
        long now = System.currentTimeMillis();
        final Date date = new Date(now);

        //연,월,일을 따로 저장
        final SimpleDateFormat curYearFormat = new SimpleDateFormat("yyyy", Locale.KOREA);
        final SimpleDateFormat curMonthFormat = new SimpleDateFormat("MM", Locale.KOREA);
        final SimpleDateFormat curDayFormat = new SimpleDateFormat("dd", Locale.KOREA);

        //현재 날짜 텍스트뷰에 뿌려줌
        tvDate.setText(curYearFormat.format(date) + "/" + curMonthFormat.format(date));

        //요일 넣기
        dayList = new ArrayList<String>();
        dayList.add("일");
        dayList.add("월");
        dayList.add("화");
        dayList.add("수");
        dayList.add("목");
        dayList.add("금");
        dayList.add("토");

        mCal = Calendar.getInstance();

        //이번달 1일 무슨요일인지 판단 mCal.set(Year,Month,Day)
        mCal.set(Integer.parseInt(curYearFormat.format(date)), Integer.parseInt(curMonthFormat.format(date)) - 1, 1);

        int dayNum = mCal.get(Calendar.DAY_OF_WEEK);

        //1일 - 요일 매칭 시키기 위해 공백 add
        for (int i = 1; i < dayNum; i++) {
            dayList.add("");
        }

        setCalendarDate(mCal.get(Calendar.MONTH) + 1);

        //griedview에 요일 추가
        gridAdapter = new GridAdapter(getApplicationContext(), dayList);
        gridView.setAdapter(gridAdapter);
    }

    //값 가져오기
    private String getPreferences(String key) {
        SharedPreferences pref = PreferenceManager.getDefaultSharedPreferences(getApplicationContext());
        return pref.getString(key, "");
    }

    // 값 저장하기
    private void setPreferences(String key, String value) {
        SharedPreferences pref = PreferenceManager.getDefaultSharedPreferences(getApplicationContext());
        //    SharedPreferences pref = this.getSharedPreferences("com.bridou_n.beaconscanner.features.beaconList", Context.MODE_PRIVATE);
        SharedPreferences.Editor editor = pref.edit();
        editor.putString(key, value);
        editor.apply();
        editor.commit();
    }

    /**
     * 해당 월에 표시할 일 수 구함
     *
     * @param month
     */
    private void setCalendarDate(int month) {
        mCal.set(Calendar.MONTH, month - 1);
        for (int i = 0; i < mCal.getActualMaximum(Calendar.DAY_OF_MONTH); i++) {
            dayList.add("" + (i + 1));
        }
    }

    //그리드뷰 어댑터
    private class GridAdapter extends BaseAdapter {
        private final List<String> list;
        private final LayoutInflater inflater;

        /**
         * 생성자
         *
         * @param context
         * @param list
         */

        public GridAdapter(Context context, List<String> list) {
            this.list = list;
            this.inflater = (LayoutInflater) context.getSystemService(Context.LAYOUT_INFLATER_SERVICE);
        }

        @Override
        public int getCount() {
            return list.size();
        }

        @Override
        public String getItem(int position) {
            return list.get(position);
        }

        @Override
        public long getItemId(int position) {
            return position;
        }

        @Override
        public View getView(int position, View convertView, ViewGroup parent) {
            ViewHolder holder = null;

            if (convertView == null) {
                convertView = inflater.inflate(R.layout.item_calendar_gridview, parent, false);
                holder = new ViewHolder();
                holder.tvItemGridView = (TextView) convertView.findViewById(R.id.tv_item_gridview);
                convertView.setTag(holder);
            } else {
                holder = (ViewHolder) convertView.getTag();
            }
            holder.tvItemGridView.setText("" + getItem(position));

            //해당 날짜 텍스트 컬러,배경 변경
            mCal = Calendar.getInstance();

            Integer today = mCal.get(Calendar.DAY_OF_MONTH); //오늘 day 가져옴
            String sToday = String.valueOf(today);

            if (sToday.equals(getItem(position))) { //오늘 day 텍스트 컬러 변경
                holder.tvItemGridView.setTextColor(Color.BLUE);
                setPreferences("Checkdate", getPreferences("Checkdate") + sToday + " ");
            }

            String putDate[] = getPreferences("Checkdate").split(" ");
            for (String date : putDate) {
                if (date.equals(getItem(position)) && !date.equals("")) {//오늘 날짜와 앱 실행한 날짜가 같으면 체크 표시
                    holder.tvItemGridView.setBackground(getResources().getDrawable(R.drawable.checked));
                }
            }
            //매달 1일에 체크 표시 초기화
            if (today == 1) {
                setPreferences("Checkdate", "");
            }
            convertView.setOnClickListener(null);
            return convertView;
        }

    }

    //다음 화면 이동
    public void NextButton(View v) {
        Intent intent = new Intent(CalendarActivity.this, choice.class);
        startActivity(intent);
    }

    private class ViewHolder {
        TextView tvItemGridView;
    }
}

