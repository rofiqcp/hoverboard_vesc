# V22 — linker fix + complete debug

Error yang diperbaiki:

- `undefined reference to VescProtocol_ServiceBudget`
- `undefined reference to VescProtocol_RxBudgetYields`

Akar masalahnya adalah header + `vesc_services.c` sudah memakai API scheduler baru, sedangkan
`vesc_protocol.c` yang ter-build masih service lama.

## Jalankan pada checkout lengkap Anda

```bash
./v22/APPLY_AND_TEST_LINUX.sh /media/sirobo/Data/BLDC/hoverboard_vesc
```

Patcher memasukkan implementasi `VescProtocol_ServiceBudget()` yang benar-benar bounded, getter
`VescProtocol_RxBudgetYields()`, dan critical reply backpressure untuk FW/config response. Sesudah itu
debug runner melakukan source-contract, semua Python tests, host C tests (bila tersedia), lalu `pio run`.

## Debug hardware read-only

```bash
python3 v22/tools/v22_debug.py /media/sirobo/Data/BLDC/hoverboard_vesc \
  --port /dev/ttyUSB0 --baud 115200
```

Tidak ada command motor SET pada serial probe default. Yang dites: FW_VERSION, RT GET_VALUES,
PING_CAN, RIGHT virtual CAN ID 2, MCCONF, APPCONF, dan CRC parser recovery.

## Buat ZIP source lengkap dari checkout setelah PASS

```bash
./v22/PACK_COMPLETE_LOCAL_TREE.sh /media/sirobo/Data/BLDC/hoverboard_vesc
```

Hasil: `hoverboard_vesc_v22_full_fixed.zip` dengan isi langsung `v22/src`, `v22/tests`, `v22/tools`, dst.
