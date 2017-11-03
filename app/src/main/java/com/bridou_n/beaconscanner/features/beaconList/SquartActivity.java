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

public class SquartActivity extends AppCompatActivity implements BeaconConsumer, EasyPermissions.PermissionCallbacks, TextToSpeech.OnInitListener {

    protected static final String TAG = "SQUART_ACTIVITY";
    private static final String[] perms = {Manifest.permission.ACCESS_COARSE_LOCATION};
    private static final int RC_COARSE_LOCATION = 1;
    private static final int RC_SETTINGS_SCREEN = 2;
    private static final String PREF_TUTO_KEY = "PREF_TUTO_KEY";
    private static final String STATE_SCANNING = "scanState";
    public static String uuid_value = "-1";
    public static String major = "-1";
    public static String minor = "-1";
    public static String[] yaw = {"-1", "-1", "-1", "-1", "-1", "-1", "-1", "-1", "-1", "-1", "-1", "-1", "-1", "-1", "-1", "-1", "-1", "-1", "-1", "-1"};//지혜
    public static String roll = "-1";
    public static char roll0;
    public static String one = "0"; //(char-->string 변수형 변환)
    public static int test;
    public static int exnum = 0;// 운동 횟수
    public static int gaptime = 0; //쉬는 시간
    public static int timeCounter = 0; //현재 운동 횟수
    public static int numCounter = 0; //현재 쉬는 시간
    public static int count = 1;
    boolean bRunning = true; // 중간에 버튼 시작
    boolean bRelease = true; // 중간에 버튼 중지
    Message msg;
    public static int counterCheck = 1;

    TimerTask mTimerTask;
    TimerTask sTimerTask;
    Timer timer = new Timer();
    Timer timer2 = new Timer();
    TextToSpeech myTTS;

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
    @BindView(R.id.toolbar)
    Toolbar toolbar;
    @BindView(R.id.activity_squart)
    CoordinatorLayout rootView;
    @BindView(R.id.bluetooth_state)
    TextView bluetoothState;
    @BindView(R.id.scan_fab)
    FloatingActionButton scanFab;
    @BindView(R.id.scan_progress)
    ProgressBar scanProgress;
    @BindView(R.id.textDfferent)
    TextView textDfferent;
    @BindView(R.id.exStartButton)
    ImageButton sButton; //지혜
    @BindView(R.id.exResetButton)
    ImageButton rButton; //지혜
    @BindView(R.id.resultTxt)
    TextView rTextView;
    @BindView(R.id.timeTxt)
    TextView tTextView;
    @BindView(R.id.countTxt)
    TextView nTextView;
    @BindView(R.id.progressBar)
    ProgressBar progressBar;
    String leftHandValue = "noValue";
    String preValue = "noValue";
    int state;
    String rightHandValue = "noValue";
    public static String roll5 = "noValue";
    public static String roll6 = "noValue";
    int sec = 0;//초세기// 초 세는 동안은 up, down ,유지 안하도록
    int restart = 0; //유지 한 후에는 up 하도록
    int x = 0;
    int upState = 0; // 올라가야 하는 상태인지
    int downState = 0; // 내려가야 하는 상태인지
    int bflag = 0;
    private Handler handler1 = new Handler() {
        public void handleMessage(Message msg) {
            super.handleMessage(msg);
            progressBar.setProgress(msg.arg1);
        }
    };


    private Handler handler = new Handler();
    private Runnable runnable = new Runnable() {
        public void run() {
            Log.d("run", "run");
            //if (bRunning) {
            Log.d("run", "running");

            textDfferent.setText("");
            if (bRunning) {

            }
            if (major.equals("1234") && minor.equals("7891")) { //왼쪽 허벅지
                if (one == "f") {
                    one = "5";

                }
                one = roll5;
                rightHandValue = one;
            }
            if (major.equals("1234") && minor.equals("8912")) { //오른쪽 허벅지
                one = roll6;
                leftHandValue = one;

                Bundle intent = getIntent().getExtras();
                exnum = Integer.parseInt(intent.getString("E_NUM3"));
                gaptime = Integer.parseInt(intent.getString("E_TIME3"));
                /*  0 :수평 이하
                *  1: 수평
                *  2 : 수평과 수직 사이
                *  3: 수직
                *  4 : 수지에서 더 나아간 상태
                * */

                //위의 자료를 활용하여 운동안내
                if (sec == 0 && bRunning) { // bRunning : 실행 중 , sec==0 : 유지 상태가 아니라면
                    if (leftHandValue.equals("0")) {
                        Log.d("1", "0");
                        upState = 1;
                        downState = 0;
                        myTTS.speak("UP", TextToSpeech.QUEUE_FLUSH, null);
                        // leftHand.setImageResource(R.drawable.lh11);
                    } else if (leftHandValue.equals("4")) {
                        Log.d("1", "4");
                        upState = 0;
                        downState = 1;
                        myTTS.speak("DOWN", TextToSpeech.QUEUE_FLUSH, null);
                    } else if (leftHandValue.equals("1")) {
                        if (preValue.equals("1")) {
                            Log.d("1", "11");
                            upState = 1;
                            downState = 0;
                            myTTS.speak("UP", TextToSpeech.QUEUE_FLUSH, null);
                        } else if (x == 1) {
                            Log.d("1", "11");
                            upState = 1;
                            downState = 0;
                            myTTS.speak("UP", TextToSpeech.QUEUE_FLUSH, null);
                        } else {
                            Log.d("1", "1");
                            myTTS.speak("유지", TextToSpeech.QUEUE_FLUSH, null);
                            timeCounter = 0;
                            sec = 1;
                            if (x == 0) {
                                doTimerTask(); // doTimerTask() 실행, 설정된 유지 시간 동안 유지
                                x = 1;
                            }
                            counterCheck = 1;
                        }
                    } else if (leftHandValue.equals("3") && preValue.equals("3")) {
                        Log.d("1", "33");
                        myTTS.speak("DOWN", TextToSpeech.QUEUE_FLUSH, null);
                        state = 0;
                        upState = 0;
                        downState = 1;

                    } else if (leftHandValue.equals("3")) {
                        Log.d("1", "3");

                        x = 0;
                        upState = 0;
                        downState = 1;
                        if (counterCheck == 1) { //수평에 도달하면 운동횟수 셈
                            nTextView.setText("운동진행  " + numCounter + " / " + exnum);
                            if (numCounter == 0) {
                                myTTS.speak("운동시작", TextToSpeech.QUEUE_FLUSH, null);
                            } else {
                                myTTS.speak("일어남" + numCounter + "회", TextToSpeech.QUEUE_FLUSH, null);
                            }
                        }
                        counterCheck = 0;
                        //끝인 경우 종료
                        if (numCounter == exnum) {
                            myTTS.speak("끝", TextToSpeech.QUEUE_FLUSH, null);
                            sec = 1;
                            stopTask();
                        }
                        upState = 0;
                        downState = 1;
                    } else if (upState == 1) {
                        Log.d("1", "upstate");
                        myTTS.speak("UP", TextToSpeech.QUEUE_FLUSH, null);

                    } else if (downState == 1) {
                        Log.d("1", "downState");
                        myTTS.speak("Down", TextToSpeech.QUEUE_FLUSH, null);

                    }
                    preValue = leftHandValue;
                }
            }

            handler.postDelayed(this, 1000);
        }
    };

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_squart);
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

        subs.add(rxBus.toObserverable() //1
                .observeOn(AndroidSchedulers.mainThread()) // We use this so we use the realm on the good thread & we can make UI changes
                .subscribe(e -> {
                    if (e instanceof Events.RangeBeacon) {
                        updateUiWithBeaconsArround(((Events.RangeBeacon) e).getBeacons());
                    }
                }, throwable -> Log.e("Error", throwable.getMessage())));

        runnable.run();


        // Setup an observable on the bluetooth changes
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
                    if (!getPreferences(Context.MODE_PRIVATE).getBoolean(PREF_TUTO_KEY, false)) {
                        showTutorial();
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

    //시작 버튼
    View.OnClickListener sButtonListener = new View.OnClickListener() {
        public void onClick(View v) {
            bRunning = true;
            rTextView.setText("");
            if (mTimerTask == null) {
                bRunning = true;
                myTTS.speak("시작", TextToSpeech.QUEUE_FLUSH, null);
                rTextView.setText("");
            }
        }
    };

    //일시정지
    View.OnClickListener rButtonListener = new View.OnClickListener() {

        public void onClick(View v) {
            // bPause = false;
            bRunning = false;
            rTextView.setText("일시정지");
        }
    };

    //운동 종료
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
        String timeText = "";
        timeText = "00:0" + timeCounter;
        rTextView.setText("");
        tTextView.setText(timeText + " 초");
        myTTS.speak(Integer.toString(k) + "초간 휴식", TextToSpeech.QUEUE_FLUSH, null);

        sTimerTask = new TimerTask() {
            public void run() {

            }
        };

        timer2.schedule(sTimerTask, k * 1000);

        rTextView.setText("휴식 끝");
    }

    //유지 프로세스
    public void doTimerTask() {
        Log.d("1", "1");
        Bundle intent = getIntent().getExtras();
        exnum = Integer.parseInt(intent.getString("E_NUM3")); // 운동횟수
        gaptime = Integer.parseInt(intent.getString("E_TIME3")); //1회당 시간
        if (mTimerTask != null)
            mTimerTask.cancel();

        mTimerTask = new TimerTask() {
            public void run() {

                handler.post(new Runnable() {
                    public void run() {
                        if (sec == 1) {
                            int remainTime = gaptime - timeCounter; // 남은 시간

                            if (!bRunning) {//bRunning이 false이면 return
                                Log.d("bRunning", "bRnning");
                                return;
                            }
                            msg = handler.obtainMessage();
                            msg.arg1 = (100 / gaptime) * timeCounter;
                            handler1.sendMessage(msg);
                            //세트 횟수와 초를 세어 표시
                            if (leftHandValue.equals("1")) {
                                String timeText = "";
                                myTTS.speak("진행", TextToSpeech.QUEUE_FLUSH, null);
                                if (timeCounter < 10) {
                                    timeText = "00:0" + remainTime;
                                } else if (timeCounter < 60) {
                                    timeText = "00:" + remainTime;
                                }
                                rTextView.setText("");
                                tTextView.setText(timeText + " 초");

                                if (timeCounter == 0) {
                                    bRelease = true;
                                    myTTS.speak(numCounter + "세트", TextToSpeech.QUEUE_FLUSH, null);
                                }
                                timeCounter++;
                                if (gaptime - timeCounter < gaptime) {

                                    String str = remainTime + "초";
                                    myTTS.speak(str, TextToSpeech.QUEUE_FLUSH, null);
                                } else {
                                    sec = 0;
                                }
                            } else if (leftHandValue.equals("0")) {
                                myTTS.speak("UP", TextToSpeech.QUEUE_FLUSH, null);
                            } else if (leftHandValue.equals("3") || leftHandValue.equals("4") || leftHandValue.equals("2")) {
                                myTTS.speak("DOWN", TextToSpeech.QUEUE_FLUSH, null);
                            }
                            if (timeCounter > gaptime) {
                                numCounter++;
                                sec = 0;
                                msg.arg1 = 100;
                                timeCounter = 0;
                                nTextView.setText("운동진행  " + numCounter + " / " + exnum);
                                timeCounter = 0;
                                bRelease = false;
                                Log.d("1", "운동");
                                return;


                            }
                            if (numCounter > exnum) {
                                stopTask();
                            }
                            Log.d("TIMER", "TimerTask run");
                        }
                    }
                });
            }
        };
        timer.schedule(mTimerTask, 0, 1000); //
    }

    //튜토리얼
    public void showTutorial() {
        AppCompatActivity _this = this;

        TapTargetView.showFor(this,
                TapTarget.forToolbarMenuItem(toolbar, R.id.action_bluetooth, getString(R.string.bluetooth_control), getString(R.string.feature_bluetooth_content)).cancelable(false).dimColor(R.color.primaryText).drawShadow(true),
                new TapTargetView.Listener() {
                    @Override
                    public void onTargetClick(TapTargetView view) {
                        super.onTargetClick(view);
                        bluetooth.enable();
                        TapTargetView.showFor(_this,
                                TapTarget.forView(scanFab, getString(R.string.feature_scan_title), getString(R.string.feature_scan_content)).tintTarget(false).cancelable(false).dimColor(R.color.primaryText).drawShadow(true),
                                new TapTargetView.Listener() {
                                    @Override
                                    public void onTargetClick(TapTargetView view) {
                                        super.onTargetClick(view);
                                        startStopScan(); // We start scanning for beacons
                                        TapTargetView.showFor(_this,
                                                TapTarget.forToolbarMenuItem(toolbar, R.id.action_clear, getString(R.string.feature_clear_title), getString(R.string.feature_clear_content)).cancelable(false).dimColor(R.color.primaryText).drawShadow(true),
                                                new TapTargetView.Listener() {
                                                    @Override
                                                    public void onTargetClick(TapTargetView view) {
                                                        super.onTargetClick(view);
                                                        getPreferences(Context.MODE_PRIVATE).edit().putBoolean(PREF_TUTO_KEY, true).apply();
                                                    }
                                                });
                                    }
                                });
                    }
                });
    }


    /* Nicolas Bridoux, https://github.com/Bridouille/android-beacon-scanner  참조 */


    @Override
    protected void onResume() {
        super.onResume();
    }

    //비콘의 신홀르 받아옴
    private void updateUiWithBeaconsArround(Collection<Beacon> beacons) {
        realm.executeTransactionAsync(tRealm -> {
            Observable.from(beacons)
                    .subscribe(b -> {
                        BeaconSaved beacon = new BeaconSaved();

                        if (b.getServiceUuid() == 0xfeaa) {

                        } else { //비콘 신호가 있다면
                            //major, minor, roll값 추출
                            beacon.setBeaconType(b.getBeaconTypeCode() == 0xbeac ? BeaconSaved.TYPE_ALTBEACON : BeaconSaved.TYPE_IBEACON); // 0x4c000215 is iBeacon
                            beacon.setUUID(b.getId1().toString());
                            uuid_value = b.getId1().toString();
                            roll0 = uuid_value.charAt(10);
                            Log.d("roll", uuid_value.charAt(10) + "");
                            roll = roll0 + "";
                            beacon.setMajor(b.getId2().toString());
                            major = b.getId2().toString();
                            beacon.setMinor(b.getId3().toString());
                            minor = b.getId3().toString();

                            switch (minor) { //왼쪽 허벅지를 roll값 설정
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

    private void bluetoothStateChanged(int state) {
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
            ActivityCompat.requestPermissions(SquartActivity.this, perms, RC_COARSE_LOCATION);
        }
    }

    //비콘 신호 스캔
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


    private PopupWindow squatPoppWindow;

    private int popupFlag = 0;

    @Override
    public boolean onOptionsItemSelected(MenuItem item) {
        int id = item.getItemId();
        if (id == R.id.action_bluetooth) {
            bluetooth.toggle();
            return true;
        }
        if (id == R.id.action_clear) {
            realm.executeTransactionAsync(tRealm -> {
                tRealm.where(BeaconSaved.class).findAll().deleteAllFromRealm();
            });
        }
        if (id == R.id.action_help) {
            if (popupFlag == 0) {
                showHelpPopup();
                popupFlag = 1;
            } else {
                dismissPopupWindow();
                View backGround = (LinearLayout) findViewById(R.id.squat_rootview);
                backGround.setEnabled(true);
                popupFlag = 0;
            }
        }
        return super.onOptionsItemSelected(item);
    }


    /**
     * popupWindow관련 코드
     */

    public void showHelpPopup() {

        LayoutInflater inflater = (LayoutInflater) this.getSystemService(this.LAYOUT_INFLATER_SERVICE);
        View helpLayout = inflater.inflate(R.layout.help_squat, null);

        View backGround = (LinearLayout) findViewById(R.id.squat_rootview);

        backGround.setEnabled(false);
        PopupWindow squatPoppWindow = new PopupWindow(helpLayout, WindowManager.LayoutParams.MATCH_PARENT, 1400, false);

        squatPoppWindow.setOutsideTouchable(false);
        squatPoppWindow.showAtLocation(helpLayout, Gravity.CENTER, 50, 50);
        setPopupWindow(squatPoppWindow);
    }

    public void setPopupWindow(PopupWindow helpPopupWindow) {
        this.squatPoppWindow = helpPopupWindow;
    }

    /**
     * 팝업윈도우 제거
     */
    public void dismissPopupWindow() {
        squatPoppWindow.dismiss();
    }


    public void startScan() {
        bindBeaconManager();
        rotate.setRepeatCount(Animation.INFINITE);
        scanFab.startAnimation(rotate);
        scanProgress.setVisibility(View.VISIBLE);
        toolbar.setTitle(getString(R.string.scanning_for_beacons));
    }

    public void stopScan() {
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
                ActivityCompat.requestPermissions(SquartActivity.this, perms, RC_COARSE_LOCATION);
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
    //상단의 블루투스 여부
    @Override
    public boolean onCreateOptionsMenu(Menu menu) {
        getMenuInflater().inflate(R.menu.main_menu, menu);
        if (!bluetooth.isEnabled()) {
            menu.getItem(2).setIcon(R.drawable.ic_bluetooth_disabled_white_24dp);
        }
        return true;
    }


    @Override
    public void onSaveInstanceState(Bundle savedInstanceState) {
        savedInstanceState.putBoolean(STATE_SCANNING, beaconManager.isBound(this)); // save the scanning state
        super.onSaveInstanceState(savedInstanceState);
    }
    //종료
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