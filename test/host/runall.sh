#!/bin/bash
# Host test suite for alds-gg-vvm701-fw.
#
# Compiles the REAL firmware sources for the host against stub ESP-IDF headers
# and runs them. Nothing here is a transcription of firmware logic: whole files
# are compiled where possible (yenen.c, nvs_config.c, store.c), and the
# functions that live inside main.c are pulled out of it verbatim by
# extract.py at test time, so a test can never drift from the code it covers.
#
# The fault-injection suites are built with ASan + UBSan on. A clean run without
# the sanitizers proves much less, so do not remove them to silence something.
#
# Needs: gcc, python3. No hardware, no ESP-IDF. Run it from anywhere:
#     bash test/host/runall.sh
set -u
cd "$(dirname "$0")"
SRC="$(cd ../../src && pwd)"
CFLAGS="-std=gnu11 -Wall -Wextra -Wno-unused-parameter -I. -Istub -I$SRC"
SAN="-g -O1 -fsanitize=address,undefined -Wno-format-truncation"
rc=0

# ── pull the functions under test out of main.c / nvs_config.c ──────────────
python3 extract.py "$SRC/main.c" func:json_get_str \
    func:json_has_key func:json_get_bool func:setcfg_has_secret \
    func:payload_is_one_json_object \
    > extracted_json.inc || rc=1
python3 extract.py "$SRC/main.c" func:mqtt_reconnect_if_needed \
    > extracted_ladder.inc || rc=1
python3 extract.py "$SRC/nvs_config.c" func:fnv1a func:fnv1a_str \
    func:nvs_config_compiled_cred_fp func:nvs_config_enforce_firmware_creds \
    > extracted_nvs.inc || rc=1
python3 extract.py "$SRC/nvs_config.c" func:erase_sibling_namespaces \
    func:nvs_config_note_reverted_foreign_image func:product_check \
    func:nvs_config_product_wiped func:terminate_strings \
    > extracted_product.inc || rc=1
python3 extract.py "$SRC/main.c" func:validate_suffixes func:radar_slave_for \
    func:validate_bus_addresses > extracted_plan.inc || rc=1
python3 extract.py "$SRC/main.c" func:json_get_str func:json_has_key \
    func:setcfg_get_sfx func:setcfg_value func:setcfg_value_ends \
    func:setcfg_int_in_range func:setcfg_genset_addr \
    func:setcfg_get_text func:streq_ci func:setcfg_du_slots \
    > extracted_setcfg.inc || rc=1
python3 extract.py "$SRC/main.c" func:json_get_str func:json_has_key \
    func:setcfg_get_sfx func:setcfg_value func:setcfg_value_ends func:setcfg_word \
    func:setcfg_int_in_range func:setcfg_get_bool func:setcfg_get_text \
    func:setcfg_genset_addr func:setcfg_check_members func:payload_is_one_json_object \
    func:factory_reset_refusal \
    > extracted_setcfg_all.inc || rc=1
python3 extract.py "$SRC/dispenser.c" func:du_bytes_to_hex func:clamp_pos \
    func:du_build_nozzle_payload > extracted_payload.inc || rc=1
python3 extract.py "$SRC/gps.c" func:next_field func:gps_parse_loc func:add_field \
    func:gps_build_payload func:gps_gsv_summary > extracted_gps.inc || rc=1
python3 extract.py "$SRC/di.c" func:di_enqueue func:di_publish_buffer \
    > extracted_di.inc || rc=1
python3 extract.py "$SRC/modem.c" func:days_from_civil func:parse_and_set_rtc \
    func:qntp_time > extracted_rtc.inc || rc=1
python3 extract.py "$SRC/fota.c" func:sanitize_url func:url_normalize func:prepare_url \
    func:fota_reason_is_final > extracted_fota_url.inc || rc=1
python3 extract.py "$SRC/mqtt.c" func:qmtrecv_payload_start func:qmtrecv_payload_end \
    > extracted_qmtrecv.inc || rc=1
python3 extract.py "$SRC/main.c" func:hw_id_is_mine > extracted_hw_id.inc || rc=1

# ── functional suites ───────────────────────────────────────────────────────
gcc $CFLAGS test_yenen.c "$SRC/yenen.c"     -o t  || rc=1   # Yenen poll path + noise
gcc $CFLAGS test_du_plan.c  simnvs.c "$SRC/nvs_config.c" -o td || rc=1   # slot ON with its suffix = dispenser number
gcc $CFLAGS test_setcfg.c                   -o tsc || rc=1  # setconfig: bad values refused, never clamped
gcc $CFLAGS test_payload.c                  -o ty || rc=1   # nozzle payload = field shape
gcc $CFLAGS test_genset.c "$SRC/genset.c"   -o tg || rc=1   # DG reads = the spec's frames
gcc $CFLAGS test_nvs_creds.c                -o tn || rc=1   # firmware creds win
gcc $CFLAGS test_product_guard.c            -o tp || rc=1   # product guard
gcc $CFLAGS test_cpu_load.c "$SRC/cpu_load.c" -o tcpu || rc=1   # per-core CPU load in diagnostics

# ── fault injection, sanitizers on ──────────────────────────────────────────
gcc $CFLAGS $SAN test_ladder.c                              -o tl  || rc=1
gcc $CFLAGS $SAN test_nvs_fuzz.c  simnvs.c "$SRC/nvs_config.c" -o tfz || rc=1
gcc $CFLAGS $SAN -include pathmap.h -c "$SRC/store.c" -o store_host.o  || rc=1
gcc $CFLAGS $SAN test_store_fuzz.c simfs.c store_host.o     -o tst || rc=1
gcc $CFLAGS $SAN test_json_fuzz.c                           -o tj  || rc=1
gcc $CFLAGS $SAN test_gps.c -lm                             -o tgps || rc=1  # AT+QGPSLOC parser + payload (bwsr's suite)
gcc $CFLAGS $SAN test_kfactor.c "$SRC/dispenser.c"            -o tkf || rc=1   # k-factor cache vs a simulated pump
gcc $CFLAGS $SAN test_bluesky_frame.c                         -o tbf || rc=1   # BlueSky reply check vs field frames
gcc $CFLAGS $SAN test_rsp_tunnel.c "$SRC/rsp_tunnel.c"        -o trt || rc=1   # rsp/kfactor tunnel: bytes in = bytes out
gcc $CFLAGS $SAN test_radar.c "$SRC/radar.c"                  -o trd || rc=1   # radar reply timing vs a simulated radar
gcc $CFLAGS $SAN test_di.c                                    -o tdi || rc=1   # DI queue: change time kept, oldest dropped
gcc $CFLAGS $SAN test_rtc.c                                   -o trtc || rc=1  # NTP result -> UTC clock, same guards
gcc $CFLAGS $SAN test_fota_url.c                              -o tfu || rc=1   # FOTA link as the modem gets it + HW_ID match
gcc $CFLAGS $SAN test_qmtrecv.c                               -o tqr || rc=1   # inbound MQTT message boundaries

for f in t td tsc ty tg tn tp tcpu tl tfz tst tj tgps tkf tbf trt trd tdi trtc tfu tqr; do
    out=$(./$f 2>/dev/null)
    echo "$f: $(echo "$out" | tail -1)"
    echo "$out" | grep -q "0 failures" || rc=1
done

if [ $rc -eq 0 ]; then echo "ALL PASS"; else echo "FAILURES — see above"; fi
exit $rc
