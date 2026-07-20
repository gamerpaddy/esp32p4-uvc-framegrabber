# ESP32-C6 coprocessor firmware

ESP-Hosted coprocessor build for the ESP32-C6 on the JC-ESP32P4-M3. The P4 has
no radio; this is what makes `esp_wifi_*` calls on the P4 actually reach an
antenna.

## Why this exists as a separate project

Different target (`esp32c6` vs `esp32p4`), so it cannot be a component of the
P4 app — it needs its own build. The output is flashed **into the C6**, not the
P4, using the P4 as the programmer (`c6flash` on the P4 console), because the
module gives the C6 no USB of its own.

## Configuration that matters

| Setting | Value | Why |
|---|---|---|
| esp_hosted version | `==3.0.1` | Must be **identical** to the P4 host's version in `../main/idf_component.yml`. Host and CP speak a versioned RPC wire; a mismatch presents as a link that never finishes handshaking. |
| Transport | SDIO | Matches `CONFIG_ESP_HOSTED_HOST_TRANSPORT_BUS_SDIO` on the P4. |
| SDIO mode | STREAM | Not the component's SW_AGGR default — see below. |
| Reset GPIO | `-1` | Means "the host resets me via my EN pin", which is how P4 GPIO54 is wired. |
| SDIO pins | fixed | The C6's SDIO slave pins are fixed in silicon: CLK=19, CMD=18, D0–D3=20,21,22,23. Kconfig range-locks them, so there is nothing to set. |

### The SDIO mode choice

esp_hosted defaults the CP to **SW_AGGR**, which packs several frames into one
transfer (~10x fewer bus transactions). It requires an ESP-IDF containing
commit `4814514`, which lifts the 4092-byte cap on SDIO slave sends. IDF 6.0.1
does not have it, and the component's build guard hard-fails rather than
silently degrading:

```
SDIO SW_AGGR: this ESP-IDF lacks the send-cap fix
```

This project uses **STREAM** (IDF hardware batching) instead, which needs no
patched IDF. If `sdiospeed` on the P4 comes back disappointing and you want the
extra throughput, you can patch the shared IDF install:

```sh
python <esp_hosted>/eh.py patch-idf --idf-path C:/esp/v6.0.1/esp-idf
```

then set `CONFIG_EH_TRANSPORT_CP_SDIO_MODE_SW_AGGR=y` here, delete `sdkconfig`,
and rebuild both sides. Note that this modifies your ESP-IDF installation and
affects every project using it.

## Flash size assumption

`partitions_eh_cp_ota_4m.csv` assumes the C6-MINI-1 carries **4 MB** of flash,
which is the standard part. If `c6flash` reports the image is too large for the
target, check the actual size and switch to the 2 MB table.

## Build

Normally you do not run this directly — `..\tools\flash_all.ps1` drives the
whole sequence. Standalone:

```sh
idf.py set-target esp32c6
idf.py build
```

Then from the project root:

```sh
python tools/pack_c6_fw.py                       # -> build/c6_fw.bin
esptool write-flash 0x310000 build/c6_fw.bin     # into the P4's c6_fw partition
```

and on the P4 console: `c6boot -d` to confirm download mode, then `c6flash`.
