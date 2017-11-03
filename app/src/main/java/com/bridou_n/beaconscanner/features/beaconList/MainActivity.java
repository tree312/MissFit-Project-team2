package com.bridou_n.beaconscanner.features.beaconList;

import android.Manifest;
import android.bluetooth.BluetoothAdapter;
import android.content.Intent;
import android.net.Uri;
import android.os.Bundle;
import android.os.Handler;
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
import android.widget.ImageView;
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

import org.altbeacon.beacon.Beacon;
import org.altbeacon.beacon.BeaconConsumer;
import org.altbeacon.beacon.BeaconManager;
import org.altbeacon.beacon.Region;

import java.util.Collection;
import java.util.List;

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

//줄넘기 운동
public class MainActivity extends AppCompatActivity implements BeaconConsumer, EasyPermissions.PermissionCallbacks, TextToSpeech.OnInitListener {
    protected static final String TAG = "MAIN_ACTIVITY";
    private static final String[] perms = {Manifest.permission.ACCESS_COARSE_LOCATION};
    private static final int RC_COARSE_LOCATION = 1;
    private static final int RC_SETTINGS_SCREEN = 2;
    private static final String STATE_SCANNING = "scanState"; //블루투스 스캔 상태
    public static String major = "-1";
    public static String minor = "-1";
    public static String uuid_value = "-1";
    public static String yaw = "-1"; //블루이노부터 받은 가속도 값
    public static String yaw5 = "noValue"; //왼손의 가속도 값
    public static String yaw6 = "noValue"; //오른손의 가속도 값
    public static String yaw7 = "noValue"; //왼발의 가속도 값
    public static String yaw8 = "noValue"; //오른발의 가속도 값
    public static char yaw0; //가속도 값의 첫째자리 수
    public static char yaw10; // 가속도 값의 둘째자리 수
    public static char yaw100; //가속도 값의 셋째자리 수
    public static char yaw1; //가속도 값의 넷째자리 수
    public static int test = 0;

    private PopupWindow helpPopupWindow;

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
    @BindView(R.id.activity_main)
    CoordinatorLayout rootView;
    @BindView(R.id.bluetooth_state)
    TextView bluetoothState;
    @BindView(R.id.scan_fab)
    FloatingActionButton scanFab;
    @BindView(R.id.scan_progress)
    ProgressBar scanProgress;
    @BindView(R.id.leftHand)
    ImageView leftHand;
    @BindView(R.id.rightHand)
    ImageView rightHand;
    @BindView(R.id.leftFoot)
    ImageView leftFoot;
    @BindView(R.id.rightFoot)
    ImageView rightFoot;

    int leftHandValue; //왼손의 가속도 값
    int rightHandValue; //오른손의 가속도 값
    int leftFootValue; // 왼발의 가속도 값
    int rightFootValue; //오른발의 가속도 값
    Toast toast;
    TextToSpeech myTTS;

    //양손, 양발의 차이가 1500을 넘는 경우의 횟수를 저장하는 변수
    int limitLeftHand = 0, limitRightHand = 0;
    int limitLeftFoot = 0, limitRightFoot = 0;

    private Handler handler = new Handler();

    private Runnable runnable = new Runnable() {
        public void run() {
            // 각 보드로부터 오는 값이 없을 경우 0으로 표시
            if (yaw5 != "noValue") { //왼손
                leftHandValue = Integer.parseInt(yaw5);
            } else {
                leftHandValue = 0;
            }

            if (yaw6 != "noValue") { //오른손
                rightHandValue = Integer.parseInt(yaw6);
            } else {
                rightHandValue = 0;
            }

            if (yaw7 != "noValue") { //왼발
                leftFootValue = Integer.parseInt(yaw7);
            } else {
                leftFootValue = 0;
            }

            if (yaw8 != "noValue") { //오른발
                rightFootValue = Integer.parseInt(yaw8);
            } else {
                rightFootValue = 0;
            }

            /*
                가속도 값의 범위에 따라 5단계로 나누어 보여준다.
             */
            if (leftHandValue == 0) { //왼손의 가속도 값 보여주기
                leftHand.setImageResource(R.drawable.handbase);
            } else if (leftHandValue > 0 && leftHandValue < 1800) {
                leftHand.setImageResource(R.drawable.h1);
            } else if (leftHandValue > 1800 && leftHandValue < 2800) {
                leftHand.setImageResource(R.drawable.h2);
            } else if (leftHandValue > 2800 && leftHandValue < 4000) {
                leftHand.setImageResource(R.drawable.h3);
            } else {
                leftHand.setImageResource(R.drawable.h4);
            }

            if (rightHandValue == 0) { //오른손의 가속도 값 보여주기
                rightHand.setImageResource(R.drawable.handbaser);
            } else if (rightHandValue > 0 && rightHandValue < 1800) {
                rightHand.setImageResource(R.drawable.righthand1);
            } else if (rightHandValue > 1800 && rightHandValue < 2800) {
                rightHand.setImageResource(R.drawable.righthand2);
            } else if (rightHandValue > 2800 && rightHandValue < 4000) {
                rightHand.setImageResource(R.drawable.righthand3);
            } else {
                rightHand.setImageResource(R.drawable.righthand4);
            }

            if (leftFootValue == 0) { //왼발의 가속도 값 보여주기
                leftFoot.setImageResource(R.drawable.footbase);
            } else if (leftFootValue > 0 && leftFootValue < 1600) {
                leftFoot.setImageResource(R.drawable.lf1);
            } else if (leftFootValue > 1600 && leftFootValue < 2800) {
                leftFoot.setImageResource(R.drawable.lf2);
            } else if (leftFootValue > 2800 && leftFootValue < 4000) {
                leftFoot.setImageResource(R.drawable.lf3);
            } else {
                leftFoot.setImageResource(R.drawable.lf5);
            }

            if (rightFootValue == 0) { //오른발의 가속도 값 보여주기
                rightFoot.setImageResource(R.drawable.footbaser);
            } else if (rightFootValue > 0 && rightFootValue < 1600) {
                rightFoot.setImageResource(R.drawable.rf1);
            } else if (rightFootValue > 1600 && rightFootValue < 2800) {
                rightFoot.setImageResource(R.drawable.rf2);
            } else if (rightFootValue > 2800 && rightFootValue < 4000) {
                rightFoot.setImageResource(R.drawable.rf3);
            } else {
                rightFoot.setImageResource(R.drawable.rf5);
            }

            /*
                양손/양발에 대해 차이가 1500을 넘어서는 경우 더 큰 손/발의 변수 증가
             */
            if (leftHandValue > rightHandValue) { //양손 비교
                if (leftHandValue - rightHandValue > 1500) {
                    limitLeftHand++;
                } else {
                    limitLeftHand = 0;
                }
            } else {
                if (rightHandValue - leftHandValue > 1500) {
                    limitRightHand++;
                } else {
                    limitRightHand = 0;
                }
            }
            if (leftFootValue > rightFootValue) { //양발 비교
                if (leftFootValue - rightFootValue > 1500) {
                    limitLeftFoot++;
                } else {
                    limitLeftFoot = 0;
                }
            } else {
                if (rightFootValue - leftFootValue > 1500) {
                    limitRightFoot++;
                } else {
                    limitRightFoot = 0;
                }
            }

            /*
                양쪽 손/발에 대한 limit변수가 5번을 넘어설 경우 사용자에게 더 빠른 쪽을 알려준다.
             */
            if (leftHandValue != 0 && rightHandValue != 0) {
                if (limitLeftHand > 5) {
                    Toast.makeText(MainActivity.this, "LeftHand Is Fast", Toast.LENGTH_SHORT).show();
                    myTTS.speak("왼손빠름", TextToSpeech.QUEUE_FLUSH, null);
                    limitLeftHand = 0;
                }
                if (limitRightHand > 5) {
                    Toast.makeText(MainActivity.this, "RightHand Is Fast", Toast.LENGTH_SHORT).show();
                    myTTS.speak("오른손빠름", TextToSpeech.QUEUE_FLUSH, null);
                    limitRightHand = 0;
                }
            }
            if (leftFootValue != 0 && rightFootValue != 0) {
                if (limitLeftFoot > 5) {
                    Toast.makeText(MainActivity.this, "LeftFoot Is Fast", Toast.LENGTH_SHORT).show();
                    myTTS.speak("왼발빠름", TextToSpeech.QUEUE_FLUSH, null);
                    limitLeftFoot = 0;
                }
                if (limitRightFoot > 5) {
                    Toast.makeText(MainActivity.this, "RightFoot Is Fast", Toast.LENGTH_SHORT).show();
                    myTTS.speak("오른발빠름", TextToSpeech.QUEUE_FLUSH, null);
                    limitRightFoot = 0;
                }
            }
            handler.postDelayed(this, 500); //0.5초마다 화면에 표시
        }
    };

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);
        ButterKnife.bind(this);
        AppSingleton.activityComponent().inject(this);
        myTTS = new TextToSpeech(this, this);
        setSupportActionBar(toolbar);
        toolbar.inflateMenu(R.menu.main_menu);
        myTTS.setSpeechRate(1);

        RealmResults<BeaconSaved> beaconResults = realm.where(BeaconSaved.class).findAllSortedAsync(new String[]{"lastMinuteSeen", "distance"}, new Sort[]{Sort.DESCENDING, Sort.ASCENDING});

        subs.add(rxBus.toObserverable()
                .observeOn(AndroidSchedulers.mainThread()) // realm을 좋은 쓰레드에 사용하기 위해
                .subscribe(e -> {
                    if (e instanceof Events.RangeBeacon) {
                        updateUiWithBeaconsArround(((Events.RangeBeacon) e).getBeacons()); //주기적으로 블루이노로부터 값 받기
                    }
                }, throwable -> Log.e("Error", throwable.getMessage())));

        runnable.run();

        subs.add(bluetooth.observe()
                .observeOn(AndroidSchedulers.mainThread())
                .subscribe(e -> {
                    if (e instanceof Events.BluetoothState) { //블루투스 상태 확인
                        bluetoothStateChanged(((Events.BluetoothState) e).getState());
                    }
                }));
        subs.add(bluetooth.observe()
                .observeOn(AndroidSchedulers.mainThread())
                .subscribe(e -> { //블루투스 찾기
                    if (savedInstanceState != null && savedInstanceState.getBoolean(STATE_SCANNING)) {
                        startScan();
                    }
                }));
    }

    /* Nicolas Bridoux, https://github.com/Bridouille/android-beacon-scanner  참조 */

    @Override
    protected void onResume() {
        super.onResume();
    }

    //블루이노로부터 값 받아 저장
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

                            //자리수 별로 값을 받아 합쳐서 저장
                            yaw0 = uuid_value.charAt(1);
                            yaw100 = uuid_value.charAt(3);
                            yaw10 = uuid_value.charAt(5);
                            yaw1 = uuid_value.charAt(7);
                            yaw = yaw0 + "" + yaw100 + "" + yaw10 + "" + yaw1;

                            //들어온 값의 major, minor 저장
                            beacon.setMajor(b.getId2().toString());
                            major = b.getId2().toString();
                            beacon.setMinor(b.getId3().toString());
                            minor = b.getId3().toString();

                            //minor 값에 따라 블루이노 구별
                            switch (minor) { //왼손
                                case "5678":
                                    yaw5 = yaw;
                                    break;
                                case "6789": //오른손
                                    yaw6 = yaw;
                                    break;
                                case "7891": //왼발
                                    yaw7 = yaw;
                                    break;
                                case "8912": //오른발
                                    yaw8 = yaw;
                                    break;
                            }
                        }
                        tRealm.copyToRealmOrUpdate(beacon);
                    });
        });
    }

    /*
        블루투스 연결 및 상태 표시
    */
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
            ActivityCompat.requestPermissions(MainActivity.this, perms, RC_COARSE_LOCATION);
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
                ActivityCompat.requestPermissions(MainActivity.this, perms, RC_COARSE_LOCATION);
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
    public boolean onCreateOptionsMenu(Menu menu) { //블루투스 켜졌을 때
        getMenuInflater().inflate(R.menu.main_menu, menu);
        if (!bluetooth.isEnabled()) {
            menu.getItem(2).setIcon(R.drawable.ic_bluetooth_disabled_white_24dp);
        }
        return true;
    }

    int popupFlag = 0;

    @Override
    public boolean onOptionsItemSelected(MenuItem item) { //블루투스 팝업 설정
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
                View backGround = (LinearLayout) findViewById(R.id.rootView);
                backGround.setEnabled(true);
                popupFlag = 0;
            }
        }
        return super.onOptionsItemSelected(item);
    }

    // 블루투스 켜는 도우미 팝업
    public void showHelpPopup() {

        LayoutInflater inflater = (LayoutInflater) this.getSystemService(this.LAYOUT_INFLATER_SERVICE);
        View helpLayout = inflater.inflate(R.layout.help_ropeskipping, null);
        View backGround = (LinearLayout) findViewById(R.id.rootView);
        backGround.setEnabled(false);
        PopupWindow helpPopupWindow = new PopupWindow(helpLayout, WindowManager.LayoutParams.MATCH_PARENT, 1200, false);

        helpPopupWindow.setOutsideTouchable(false);
        helpPopupWindow.showAtLocation(helpLayout, Gravity.CENTER, 50, 50);
        setPopupWindow(helpPopupWindow);
    }

    public void setPopupWindow(PopupWindow helpPopupWindow) {
        this.helpPopupWindow = helpPopupWindow;
    }

    /**
     * 팝업윈도우 제거
     */
    public void dismissPopupWindow() {
        helpPopupWindow.dismiss();
    }

    @Override
    public void onSaveInstanceState(Bundle savedInstanceState) {
        savedInstanceState.putBoolean(STATE_SCANNING, beaconManager.isBound(this)); // 스캐닝 상태 저장
        super.onSaveInstanceState(savedInstanceState);
    }

    @Override
    protected void onDestroy() { //종료될 때 subscribe 모두 정리
        super.onDestroy();
        subs.unsubscribe();
        if (beaconManager.isBound(this)) {
            beaconManager.unbind(this);
        }
        realm.close();
        myTTS.shutdown();
        if (toast != null) {
            toast.cancel();
        }
    }

    @Override
    public void onInit(int i) {
    }
}
