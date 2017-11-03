package com.bridou_n.beaconscanner.features.beaconList;

import android.Manifest;
import android.bluetooth.BluetoothAdapter;
import android.content.Context;
import android.content.Intent;
import android.net.Uri;
import android.os.Bundle;
import android.os.Handler;
import android.os.Message;
import android.os.RemoteException;
import android.provider.Settings;
import android.speech.tts.TextToSpeech;
import android.support.design.widget.CoordinatorLayout;
import android.support.design.widget.FloatingActionButton;
import android.support.design.widget.Snackbar;
import android.support.v4.app.ActivityCompat;
import android.support.v4.content.ContextCompat;
import android.support.v7.app.AppCompatActivity;
import android.support.v7.widget.Toolbar;
import android.util.Log;
import android.view.Gravity;
import android.view.LayoutInflater;
import android.view.Menu;
import android.view.MenuItem;
import android.view.View;
import android.view.WindowManager;
import android.view.animation.Animation;
import android.widget.ImageButton;
import android.widget.LinearLayout;
import android.widget.PopupWindow;
import android.widget.ProgressBar;
import android.widget.TextView;
import android.widget.Toast;

import com.bridou_n.beaconscanner.AppSingleton;
import com.bridou_n.beaconscanner.R;
import com.bridou_n.beaconscanner.events.Events;
import com.bridou_n.beaconscanner.events.RxBus;
import com.bridou_n.beaconscanner.models.BeaconSaved;
import com.bridou_n.beaconscanner.utils.BluetoothManager;
import com.getkeepsafe.taptargetview.TapTarget;
import com.getkeepsafe.taptargetview.TapTargetView;

import org.altbeacon.beacon.Beacon;
import org.altbeacon.beacon.BeaconConsumer;
import org.altbeacon.beacon.BeaconManager;
import org.altbeacon.beacon.Region;

import java.util.Collection;
import java.util.List;
import java.util.Timer;
import java.util.TimerTask;

import javax.inject.Inject;
import javax.inject.Named;

import butterknife.BindView;
import butterknife.ButterKnife;
import butterknife.OnClick;
import io.realm.Realm;
import io.realm.RealmResults;
import io.realm.Sort;
import pub.devrel.easypermissions.EasyPermissions;
import rx.Observable;
import rx.android.schedulers.AndroidSchedulers;
import rx.subscriptions.CompositeSubscription;

//런지 운동 구현
public class LungeActivity extends AppCompatActivity implements BeaconConsumer, EasyPermissions.PermissionCallbacks, TextToSpeech.OnInitListener {
    protected final String TAG = "LUNGE_ACTIVITY";
    private final String[] perms = {Manifest.permission.ACCESS_COARSE_LOCATION};
    private final int RC_COARSE_LOCATION = 1;
    private final int RC_SETTINGS_SCREEN = 2;
    private final String PREF_TUTO_KEY = "PREF_TUTO_KEY";
    private final String STATE_SCANNING = "scanState";
    public String distance_value = "-1";
    public String uuid_value = "-1";
    public String major = "-1";
    public String minor = "-1";
    public String roll = "-1";
    public char roll0;
    public String one = "0";
    public int test;
    public int exnum;
    public int gaptime;
    public int timeCounter;
    public int numCounter = 0;
    public int flag_leg = 2; //왼쪽 다리 부터 시작
    boolean bRunning = true;
    boolean bPause = true;
    boolean bRelease = true;
    Message msg;

    TimerTask mTimerTask;
    TimerTask sTimerTask;
    Timer timer = new Timer();
    Timer timer2 = new Timer();
    TextToSpeech myTTS;
    private PopupWindow lungePopupWindow;

    private CompositeSubscription subs = new CompositeSubscription();

    @Inject
    @Named("fab_search")
    Animation rotate;
    @Inject
    BluetoothManager bluetooth;
    @Inject
    BeaconManager beaconManager;
    @Inject
    RxBus rxBus;
    @Inject
    Realm realm;
    Toast toast;

    @BindView(R.id.toolbar)
    Toolbar toolbar;
    @BindView(R.id.activity_lunge)
    CoordinatorLayout rootView;
    @BindView(R.id.bluetooth_state)
    TextView bluetoothState;
    @BindView(R.id.scan_fab)
    FloatingActionButton scanFab;
    @BindView(R.id.scan_progress)
    ProgressBar scanProgress;
    @BindView(R.id.exStartButton)
    ImageButton sButton;
    @BindView(R.id.exResetButton)
    ImageButton rButton;
    @BindView(R.id.resultTxt)
    TextView rTextView;
    @BindView(R.id.timeTxt)
    TextView tTextView;
    @BindView(R.id.countTxt)
    TextView nTextView;
    @BindView(R.id.textview_leg)
    TextView legTextView;

    @BindView(R.id.progressBar)
    ProgressBar progressBar;

    String preValue = "noValue";
    String rightValue = "noValue";
    String leftValue = "noValue";

    int ttss = 8;
    public String roll5 = "noValue";
    public String roll6 = "noValue";

    int eFlag_right = 1; //초세기 한번만 가도록, 두번 연속 초세기 하지 않도록
    int eFlag_left = 1; //초세기 한번만 가도록, 두번 연속 초세기 하지 않도록
    int sec = 0;//초세기, 초 세는 동안은 up, down ,유지 안하도록
    int restart = 0; //유지 한 후에는 up 하도록

    int upState_r = 0; // 올라가야 하는 상태인지
    int downState_r = 0; // 내려가야 하는 상태인지
    int upState_l = 0; // 올라가야 하는 상태인지
    int downState_l = 0; // 내려가야 하는 상태인지

    private Handler handler1 = new Handler() {
        public void handleMessage(Message msg) {
            super.handleMessage(msg);
            progressBar.setProgress(msg.arg1);
        }
    };

    private Runnable runnable = new Runnable() {
        public void run() {
            Bundle intent = getIntent().getExtras();
            exnum = Integer.parseInt(intent.getString("E_NUM3"));
            gaptime = Integer.parseInt(intent.getString("E_TIME3"));

            if (flag_leg == 1) {
                //오른쪽 다리 차례
                if (major.equals("1234") && minor.equals("8912")) {
                    if (one == "f")
                        one = "5";
                    one = roll6;
                    rightValue = one;

                    if (sec == 0 && bRunning) {
                        /*운동0초인 상태*/
                        if (rightValue.equals("0")) {
                            /*일어나야 하는 상태*/
                            upState_r = 1;
                            downState_r = 0;
                            myTTS.speak("UP", TextToSpeech.QUEUE_FLUSH, null);
                        } else if (leftValue.equals("4")) {
                            /*내려가야 하는 상태*/
                            upState_r = 0;
                            downState_r = 1;
                            myTTS.speak("DOWN", TextToSpeech.QUEUE_FLUSH, null);
                        } else if (rightValue.equals("1")) {

                            timeCounter = 0;

                            if (eFlag_right == 1) {
                                /*사용자 운동 자세 유지*/
                                sec = 1;
                                eFlag_right = 0;
                                bRunning = true;
                                bPause = true;
                                doTimerTask();
                            } else {
                                preValue = "1";
                                upState_r = 1;
                                downState_r = 0;
                            }
                        } else if (rightValue.equals("3")) {
                            upState_r = 0;
                            downState_r = 1;

                            if (numCounter == exnum) {
                                /*사용자 운동 멈춤*/
                                sec = 1;
                                stopTask();
                            }
                        } else if (upState_r == 1) {
                            /*올라가야 하는 상태*/
                            Log.d("1", "upstate");
                            myTTS.speak("UP", TextToSpeech.QUEUE_FLUSH, null);

                        } else if (downState_r == 1) {
                            /*내려가야 하는 상태*/
                            Log.d("1", "downState");
                            myTTS.speak("Down", TextToSpeech.QUEUE_FLUSH, null);
                        }
                        preValue = rightValue;
                    }
                }
            } else if (flag_leg == 2) {
                //왼쪽 다리 차례
                if (major.equals("1234") && minor.equals("7891")) {
                    if (ttss == 8) {
                        myTTS.speak("왼쪽 다리 시작", TextToSpeech.QUEUE_FLUSH, null);
                        ttss = 5;
                    }

                    if (one == "f")
                        one = "5";

                    one = roll5;
                    leftValue = one;

                    if (sec == 0 && bRunning) {
                        if (leftValue.equals("0")) {
                            /*올라가야 하는 상태*/
                            upState_l = 1;
                            downState_l = 0;
                            myTTS.speak("UP", TextToSpeech.QUEUE_FLUSH, null);
                        } else if (leftValue.equals("4")) {
                            /*내려가야 하는 상태*/
                            upState_l = 0;
                            downState_l = 1;
                            myTTS.speak("DOWN", TextToSpeech.QUEUE_FLUSH, null);
                        } else if (leftValue.equals("1")) {
                            timeCounter = 0;
                            if (eFlag_left == 1) {
                                /*왼쪽다리 자세 유지*/
                                sec = 1;
                                eFlag_left = 0;
                                bRunning = true;
                                bPause = true;
                                doTimerTask();
                            } else {
                                preValue = "1";
                                upState_l = 1;
                                downState_l = 0;
                            }
                        } else if (leftValue.equals("3")) {
                            upState_l = 0;
                            downState_l = 1;
                            flag_leg = 2;
                            upState_l = 0;
                            upState_r = 0;
                            downState_l = 0;
                            downState_r = 0;
                            eFlag_left = 1;
                            eFlag_right = 1;
                            sec = 0;

                            if (numCounter == exnum) {
                                /*왼쪽다리 운동 끝*/
                                myTTS.speak("끝", TextToSpeech.QUEUE_FLUSH, null);
                                sec = 1;
                                stopTask();
                            }
                            upState_l = 0;
                            downState_l = 1;

                        } else if (upState_l == 1) {
                            /*올라가야 하는 상태*/
                            myTTS.speak("UP", TextToSpeech.QUEUE_FLUSH, null);

                        } else if (downState_l == 1) {
                            /*내려가야 하는 상태*/
                            myTTS.speak("Down", TextToSpeech.QUEUE_FLUSH, null);
                        }
                        preValue = leftValue;
                    }
                }
            }
            handler1.postDelayed(this, 1000);
        }
    };

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_lunge);
        ButterKnife.bind(this);
        AppSingleton.activityComponent().inject(this);
        myTTS = new TextToSpeech(this, this);
        setSupportActionBar(toolbar);
        toolbar.inflateMenu(R.menu.main_menu);

        progressBar = (ProgressBar) findViewById(R.id.progressBar);
        tTextView = (TextView) findViewById(R.id.timeTxt);
        nTextView = (TextView) findViewById(R.id.countTxt);
        rTextView = (TextView) findViewById(R.id.resultTxt);
        sButton = (ImageButton) findViewById(R.id.exStartButton);
        sButton.setOnClickListener(sButtonListener);
        rButton = (ImageButton) findViewById(R.id.exResetButton);
        rButton.setOnClickListener(rButtonListener);
        myTTS = new TextToSpeech(this, this);

        myTTS.setSpeechRate(1);
        RealmResults<BeaconSaved> beaconResults = realm.where(BeaconSaved.class).findAllSortedAsync(new String[]{"lastMinuteSeen", "distance"}, new Sort[]{Sort.DESCENDING, Sort.ASCENDING});

        subs.add(rxBus.toObserverable()
                .observeOn(AndroidSchedulers.mainThread()) // realm을 좋은 쓰레드에 사용하기 위해
                .subscribe(e -> {
                    if (e instanceof Events.RangeBeacon) {
                        updateUiWithBeaconsArround(((Events.RangeBeacon) e).getBeacons());
                    }
                }, throwable -> Log.e("Error", throwable.getMessage())));

        runnable.run();

        // 블루투스 변화에 obeservable 설정
        subs.add(bluetooth.observe()
                .observeOn(AndroidSchedulers.mainThread())
                .subscribe(e -> {
                    if (e instanceof Events.BluetoothState) {
                        bluetoothStateChanged(((Events.BluetoothState) e).getState());
                    }
                }));

        subs.add(bluetooth.observe()
                .observeOn(AndroidSchedulers.mainThread())
                .subscribe(e -> {
                    if (savedInstanceState != null && savedInstanceState.getBoolean(STATE_SCANNING)) {
                        startScan();
                    }
                }));
    }

    public void onInit(int status) {
    }

    View.OnClickListener sButtonListener = new View.OnClickListener() {
        public void onClick(View v) {
            bRunning = true;
            if (mTimerTask == null) {
                bRunning = true;

                myTTS.speak("시작", TextToSpeech.QUEUE_FLUSH, null);
            }
        }
    };


    View.OnClickListener rButtonListener = new View.OnClickListener() {
        public void onClick(View v) {
            bRunning = false;
            rTextView.setText("일시정지");
        }
    };

    public void stopTask() {
        if (mTimerTask != null) {
            Bundle intent = getIntent().getExtras();
            exnum = Integer.parseInt(intent.getString("E_NUM3"));
            nTextView.setText("운동진행  " + exnum + "/" + exnum);
            rTextView.setText("운동이 종료되었습니다!");
            myTTS.speak("운동이 종료되었습니다", TextToSpeech.QUEUE_FLUSH, null);
            msg.arg1 = 100;
            bRunning = false;
            Log.d("TIMER", "timer canceled");
            mTimerTask.cancel();
            mTimerTask = null;
        }
    }


    public void doTimerPause(int k) {
        Bundle intent = getIntent().getExtras();

        exnum = Integer.parseInt(intent.getString("E_NUM3"));
        gaptime = Integer.parseInt(intent.getString("E_TIME3"));

        myTTS.speak("일어남" + numCounter + "회", TextToSpeech.QUEUE_FLUSH, null);

        upState_l = 0;
        upState_r = 0;
        downState_l = 0;
        downState_r = 0;
        eFlag_left = 1;
        eFlag_right = 1;
        sec = 0;

        if (flag_leg == 1) {  //왼쪽 다리 차례
            myTTS.speak(numCounter + "회 왼쪽 시작", TextToSpeech.QUEUE_FLUSH, null);

            flag_leg = 2;
            upState_l = 0;
            upState_r = 0;
            downState_l = 0;
            downState_r = 0;
            eFlag_left = 1;
            eFlag_right = 1;

            if (numCounter == exnum) {
                myTTS.speak("종료", TextToSpeech.QUEUE_FLUSH, null);
                sec = 1;
                stopTask();
            }

        } else if (flag_leg == 2) { //오른쪽 다리 차례
            myTTS.speak(numCounter + "회 오른쪽 시작", TextToSpeech.QUEUE_FLUSH, null);
            flag_leg = 1;
            upState_l = 0;
            upState_r = 0;
            downState_l = 0;
            downState_r = 0;
            eFlag_left = 1;
            eFlag_right = 1;

            if (numCounter == exnum) {
                myTTS.speak("종료", TextToSpeech.QUEUE_FLUSH, null);
                sec = 1;
                stopTask();
            }
        }

        String timeText = "";
        timeText = "00:0" + timeCounter;
        rTextView.setText("");
        tTextView.setText(timeText + " 초");
        nTextView.setText("운동진행  " + numCounter + " / " + exnum);

        if (sTimerTask != null)
            sTimerTask.cancel();

        sTimerTask = new TimerTask() {
            public void run() {
            }
        };
        timer2.schedule(sTimerTask, k * 1000);
    }


    /*사용자 운동 시간 측정 함수*/
    public void doTimerTask() {
        Bundle intent = getIntent().getExtras();
        exnum = Integer.parseInt(intent.getString("E_NUM3")); // 운동횟수
        gaptime = Integer.parseInt(intent.getString("E_TIME3")); //1회당 시간

        if (mTimerTask != null)
            mTimerTask.cancel();

        mTimerTask = new TimerTask() {
            public void run() {

                handler1.post(new Runnable() {
                    public void run() {

                        if (flag_leg == 1)
                            legTextView.setText("오른쪽 다리");
                        else if (flag_leg == 2)
                            legTextView.setText("왼쪽 다리");

                        if (sec == 1) {
                            int remainTime = gaptime - timeCounter; // 남은 시간

                            if (!bRunning) {
                                return;
                            }
                            msg = handler1.obtainMessage();
                            msg.arg1 = (100 / gaptime) * timeCounter;
                            handler1.sendMessage(msg);

                            if ((flag_leg == 1 && rightValue.equals("1")) || (flag_leg == 2 && leftValue.equals("1"))) {
                                /*사용자 운동시간 표시*/
                                String timeText = "";
                                if (timeCounter < 10) {
                                    timeText = "00:0" + remainTime;
                                } else if (timeCounter < 60) {
                                    timeText = "00:" + remainTime;
                                }

                                rTextView.setText("");
                                tTextView.setText(timeText + " 초");
                                nTextView.setText("운동진행  " + numCounter + " / " + exnum);
                                if (timeCounter == 0) {
                                    /*운동 Nset TTS 제시*/
                                    bRelease = true;
                                    myTTS.speak(numCounter + "세트", TextToSpeech.QUEUE_FLUSH, null);
                                }
                                timeCounter++;
                                if (gaptime - timeCounter < gaptime) {
                                    /*운동 유지 시간 count*/
                                    String str = remainTime + "초";
                                    myTTS.speak(str, TextToSpeech.QUEUE_FLUSH, null);
                                } else {
                                    sec = 0;
                                }
                            }
                            if (timeCounter > gaptime) {
                                /*운동 시작 알림*/
                                numCounter++;
                                sec = 0;
                                msg.arg1 = 100;
                                timeCounter = 0;
                                nTextView.setText("운동진행  " + numCounter + " / " + exnum);
                                timeCounter = 0;
                                doTimerPause(2);

                                bRelease = false;
                                return;
                            }
                            if (numCounter > exnum) {
                                /*운동 끝*/
                                stopTask();
                            }
                        }
                    }
                });
            }
        };
        timer.schedule(mTimerTask, 0, 1000);
    }

    @Override
    protected void onResume() {
        super.onResume();
    }

    /* Nicolas Bridoux, https://github.com/Bridouille/android-beacon-scanner  참조 */

    private void updateUiWithBeaconsArround(Collection<Beacon> beacons) {
        realm.executeTransactionAsync(tRealm -> {
            Observable.from(beacons)
                    .subscribe(b -> {
                        BeaconSaved beacon = new BeaconSaved();

                        if (b.getServiceUuid() == 0xfeaa) { // Eddystone beacon

                        } else { // This is an iBeacon or ALTBeacon
                            beacon.setBeaconType(b.getBeaconTypeCode() == 0xbeac ? BeaconSaved.TYPE_ALTBEACON : BeaconSaved.TYPE_IBEACON); // 0x4c000215는 iBeacon
                            beacon.setUUID(b.getId1().toString());
                            uuid_value = b.getId1().toString();
                            roll0 = uuid_value.charAt(10);

                            Log.d("roll", uuid_value.charAt(10) + "");
                            roll = roll0 + "";

                            beacon.setMajor(b.getId2().toString());
                            major = b.getId2().toString();
                            beacon.setMinor(b.getId3().toString());
                            minor = b.getId3().toString();

                            switch (minor) {
                                case "7891":
                                    roll5 = roll;
                                    break;
                                case "8912":
                                    roll6 = roll;
                                    break;
                            }
                        }
                        tRealm.copyToRealmOrUpdate(beacon);
                    });
        });
    }

    private void bluetoothStateChanged(int state) {            //블루투스 상태 관련 함수
        bluetoothState.setVisibility(View.VISIBLE);
        switch (state) {
            case BluetoothAdapter.STATE_OFF:
                bluetoothState.setTextColor(ContextCompat.getColor(this, R.color.bluetoothDisabledLight));
                bluetoothState.setBackgroundColor(ContextCompat.getColor(this, R.color.bluetoothDisabled));
                bluetoothState.setText(getString(R.string.bluetooth_disabled));
                invalidateOptionsMenu();
                break;
            case BluetoothAdapter.STATE_TURNING_OFF:
                bluetoothState.setTextColor(ContextCompat.getColor(this, R.color.bluetoothTurningOffLight));
                bluetoothState.setBackgroundColor(ContextCompat.getColor(this, R.color.bluetoothTurningOff));
                bluetoothState.setText(getString(R.string.turning_bluetooth_off));
                stopScan();
                break;
            case BluetoothAdapter.STATE_ON:
                bluetoothState.setVisibility(View.GONE); // If the bluetooth is ON, we don't warn the user
                bluetoothState.setText(getString(R.string.bluetooth_enabled));
                invalidateOptionsMenu();
                break;
            case BluetoothAdapter.STATE_TURNING_ON:
                bluetoothState.setTextColor(ContextCompat.getColor(this, R.color.bluetoothTurningOnLight));
                bluetoothState.setBackgroundColor(ContextCompat.getColor(this, R.color.bluetoothTurningOn));
                bluetoothState.setText(getString(R.string.turning_bluetooth_on));
                break;
        }
    }

    public void bindBeaconManager() {
        if (EasyPermissions.hasPermissions(this, perms)) {
            beaconManager.bind(this);
        } else {
            ActivityCompat.requestPermissions(LungeActivity.this, perms, RC_COARSE_LOCATION);
        }
    }

    @OnClick(R.id.scan_fab)
    public void startStopScan() {
        if (!beaconManager.isBound(this)) {
            if (!bluetooth.isEnabled()) {
                Snackbar.make(rootView, getString(R.string.enable_bluetooth_to_start_scanning), Snackbar.LENGTH_LONG).show();
                return;
            }
            startScan();
        } else {
            stopScan();
        }
    }

    public void startScan() {           /*비콘 스캔*/
        bindBeaconManager();
        rotate.setRepeatCount(Animation.INFINITE);
        scanFab.startAnimation(rotate);
        scanProgress.setVisibility(View.VISIBLE);
        toolbar.setTitle(getString(R.string.scanning_for_beacons));
    }

    public void stopScan() {          /*비콘 스캔 중지*/
        beaconManager.unbind(this);
        rotate.setRepeatCount(0);
        scanProgress.setVisibility(View.INVISIBLE);
        toolbar.setTitle(getString(R.string.app_name));
    }

    @Override
    public void onBeaconServiceConnect() {
        beaconManager.setRangeNotifier((beacons, region) -> {
            rxBus.send(new Events.RangeBeacon(beacons, region));
        });

        try {
            beaconManager.startRangingBeaconsInRegion(new Region("com.bridou_n.beaconscanner", null, null, null));
        } catch (RemoteException e) {
            rxBus.sendError(e);
        }
    }

    @Override
    public void onPermissionsGranted(int requestCode, List<String> perms) {
        bindBeaconManager();
    }

    @Override
    public void onPermissionsDenied(int requestCode, List<String> permList) {
        if (requestCode == RC_COARSE_LOCATION) {
            if (EasyPermissions.somePermissionPermanentlyDenied(this, permList)) {
                showPermissionSnackbar();
            } else {
                ActivityCompat.requestPermissions(LungeActivity.this, perms, RC_COARSE_LOCATION);
            }
        }
    }

    public void showPermissionSnackbar() {
        final Snackbar snackBar = Snackbar.make(rootView, getString(R.string.enable_permission_from_settings), Snackbar.LENGTH_INDEFINITE);
        snackBar.setAction(getString(R.string.enable), v -> {
            snackBar.dismiss();
            Intent intent = new Intent(Settings.ACTION_APPLICATION_DETAILS_SETTINGS, Uri.parse("package:" + getPackageName()));
            intent.addFlags(Intent.FLAG_ACTIVITY_NEW_TASK | Intent.FLAG_ACTIVITY_NO_HISTORY | Intent.FLAG_ACTIVITY_EXCLUDE_FROM_RECENTS);
            startActivityForResult(intent, RC_SETTINGS_SCREEN);
        });
        snackBar.show();
    }

    @Override
    public void onRequestPermissionsResult(int requestCode, String[] permissions, int[] grantResults) {
        super.onRequestPermissionsResult(requestCode, permissions, grantResults);
        EasyPermissions.onRequestPermissionsResult(requestCode, permissions, grantResults, this);
    }

    @Override
    public boolean onCreateOptionsMenu(Menu menu) {
        getMenuInflater().inflate(R.menu.main_menu, menu);
        if (!bluetooth.isEnabled()) {                                   //블루투스 Off
            menu.getItem(2).setIcon(R.drawable.ic_bluetooth_disabled_white_24dp);
        }
        return true;
    }

    private int popupFlag = 0;

    @Override
    public boolean onOptionsItemSelected(MenuItem item) {
        /*메뉴 관련 함수*/
        int id = item.getItemId();
        if (id == R.id.action_bluetooth) {   //블루투스 on/off
            bluetooth.toggle();
            return true;
        }
        if (id == R.id.action_clear) {         //비콘 clear 아이콘
            realm.executeTransactionAsync(tRealm -> {
                tRealm.where(BeaconSaved.class).findAll().deleteAllFromRealm();
            });
        }
        if (id == R.id.action_help) {          //팝업윈도우 아이콘
            if (popupFlag == 0) {               //flag 0 : 팝업윈도우 켜져있지 않은 경우
                showHelpPopup();                //팝업윈도우 show
                popupFlag = 1;
            } else {                             //flag 1 : 팝업윈도우 켜있는 경우
                dismissPopupWindow();           //팝업윈도우 dismiss
                View backGround = (LinearLayout) findViewById(R.id.lunge_rootview);
                backGround.setEnabled(true);        //팝업윈도우 뒷 화면 사용 가능
                popupFlag = 0;
            }
        }
        return super.onOptionsItemSelected(item);
    }


    /**
     * popupWindow 생성함수
     */
    public void showHelpPopup() {
        LayoutInflater inflater = (LayoutInflater) this.getSystemService(this.LAYOUT_INFLATER_SERVICE);
        View helpLayout = inflater.inflate(R.layout.help_lunge, null);      //help_lunge layout을 팝업화면으로 지정

        View backGround = (LinearLayout) findViewById(R.id.lunge_rootview);    //팝업윈도우의 back화면 지정

        backGround.setEnabled(false);                                          //back화면 터치 차단
        PopupWindow lungePopWindow = new PopupWindow(helpLayout, WindowManager.LayoutParams.MATCH_PARENT, 1400, false);

        lungePopWindow.setOutsideTouchable(false);                      //팝업윈도우 외부 터치 차단
        lungePopWindow.showAtLocation(helpLayout, Gravity.CENTER, 50, 50); //팝업윈도우 위치 지정
        setPopupWindow(lungePopWindow);
    }

    //런지 팝업윈도우
    public void setPopupWindow(PopupWindow lungePopupWindow) {
        this.lungePopupWindow = lungePopupWindow;
    }

    //팝업 윈도우 제거
    public void dismissPopupWindow() {
        lungePopupWindow.dismiss();
    }

    @Override
    public void onSaveInstanceState(Bundle savedInstanceState) {
        savedInstanceState.putBoolean(STATE_SCANNING, beaconManager.isBound(this)); // save the scanning state
        super.onSaveInstanceState(savedInstanceState);
    }

    @Override
    protected void onDestroy() {
        super.onDestroy();
        subs.unsubscribe();
        if (beaconManager.isBound(this)) {
            beaconManager.unbind(this);
        }
        realm.close();
        myTTS.shutdown();
    }
}
