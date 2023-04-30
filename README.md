# `ESPHome` components

[![License][license-shield]][license]
[![ESPHome release][esphome-release-shield]][esphome-release]
[![Open in Visual Studio Code][open-in-vscode-shield]][open-in-vscode]

[license-shield]: https://img.shields.io/static/v1?label=License&message=MIT&color=orange&logo=license
[license]: https://opensource.org/licenses/MIT

[esphome-release-shield]: https://img.shields.io/static/v1?label=ESPHome&message=2022.9.0&color=green&logo=esphome
[esphome-release]: https://GitHub.com/esphome/esphome/releases/

[open-in-vscode-shield]: https://img.shields.io/static/v1?label=+&message=Open+in+VSCode&color=blue&logo=visualstudiocode
[open-in-vscode]: https://open.vscode.dev/dentra/esphome-components


A collection of my ESPHome components.

To use this repository you should confugure it inside your yaml-configuration:
```yaml
external_components:
  - source: github://phuebner/esphome-components
```

## [Bticino SCS Bus](components/bticino_bus/)
Custom UART component for ESP32-S2 which uses IRDA to communicate with Bticino SCS devices.

## [Bticino Cover](components/bticino_cover/)
Cover compoenent to control covers on the Bticino SCS bus

