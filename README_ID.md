# V22 Deep Main-Owned Scheduler Fix

Paket ini adalah **overlay untuk checkout lengkap `rofiqcp/hoverboard_vesc` Anda**, bukan pengganti repository parsial.

Apply + deep test + PlatformIO build:

```bash
unzip hoverboard_vesc_v22_deep_main_scheduler.zip
./v22/APPLY_AND_TEST_LINUX.sh /media/sirobo/Data/BLDC/hoverboard_vesc
```

Untuk juga menjalankan seluruh regression Python lama di repo:

```bash
./v22/APPLY_AND_TEST_LINUX.sh /media/sirobo/Data/BLDC/hoverboard_vesc --all-repo-tests
```

Upload setelah semua test/build yang relevan PASS:

```bash
cd /media/sirobo/Data/BLDC/hoverboard_vesc
pio run -t upload
```

Read-only boot/liveness debug:

```bash
python3 tools/v22_deep_debug.py . --port /dev/ttyUSB0 --watch-seconds 8
```

Di VESC Tool Terminal juga dapat menjalankan:

```text
hb_boot_status
```

Jika watchdog/CPU fault terdeteksi dari boot sebelumnya, firmware masuk degraded mode sehingga VESC Tool tetap dapat terhubung dan auto current calibration tidak langsung mengulangi hang. Power cycle tanpa fault/reset watchdog mengembalikan auto startup normal.
