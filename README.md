# Tildagon WiFi Radar

Single-badge directional WiFi radar for the [Tildagon badge](https://tildagon.badge.emfcamp.org/) at EMF Camp. Rotate the badge to sweep &mdash; APs appear as blips on a polar display.

![WiFi Radar on Tildagon badge](screenshot.png)

## Features

- **Polar radar display** &mdash; scrolling sweep driven by the onboard IMU gyroscope
- **Live WiFi scanning** &mdash; captures nearby access points with RSSI, stamped to your current heading
- **Logarithmic distance mapping** &mdash; RSSI converted to estimated range via path-loss model
- **Persistent blips** &mdash; APs accumulate on the display and fade over time
- **Three colour themes** &mdash; green, blue, and amber radar styles
- **Credits screen** &mdash; Web Order logo with IMU-driven parallax

## Controls

| Button | Action |
|--------|--------|
| A (UP) | Cycle colour theme |
| D (DOWN) | Credits screen |
| CANCEL | Exit app / back from credits |
| IMU (rotate) | Sweep radar heading |

## Install

### From the app store

Search **WiFi Radar** in the [Tildagon App Store](https://apps.badge.emfcamp.org/).

### Manual install via mpremote

```
mpremote mkdir apps/tildagon_wifi_radar
mpremote cp app.py :apps/tildagon_wifi_radar/app.py
mpremote cp logo.png :apps/tildagon_wifi_radar/logo.png
mpremote cp tildagon.toml :apps/tildagon_wifi_radar/tildagon.toml
```

Hold the **reboop** button for 2 seconds to restart, then select WiFi Radar from the menu.

## Credits

[@webboggles](https://github.com/webboggles) &mdash; [weborder.uk](https://weborder.uk)

## Licence

MIT
