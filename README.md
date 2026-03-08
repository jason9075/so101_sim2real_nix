# SO-101 Sim-to-Real (NixOS + Isaac Sim)

本專案致力於在 NixOS 環境下，透過 NVIDIA Isaac Sim 實現 SO-101 機器人手臂的 Sim-to-Real 橋接。

## 🎬 Demo（Real→Sim→Real）

[![Demo（Real→Sim→Real）](https://img.youtube.com/vi/0H6wGFKfvSM/hqdefault.jpg)](https://youtu.be/0H6wGFKfvSM)

- YouTube: <https://youtu.be/0H6wGFKfvSM>

## 🚀 快速開始

### 1. 前置需求 (Prerequisites)

確保您的 NixOS 系統已配置以下內容：
- **NVIDIA Driver**: 建議版本 >= 535。
- **Docker**: 啟用 NVIDIA Container Toolkit 與 CDI 支援。
  ```nix
  # configuration.nix
  virtualisation.docker.enable = true;
  hardware.nvidia-container-toolkit.enable = true;
  ```
- **Wayland/X11**: 支援 GUI 視窗顯示。

### 2. NVIDIA NGC 憑證設定

Isaac Sim 的 Docker Image 存放於 NVIDIA NGC (nvcr.io)，啟動前需進行登入：

1. **申請 API Key**:
   - 前往 [NVIDIA NGC 官網](https://catalog.ngc.nvidia.com/) 並登入。
   - 點擊右上角個人頭像 -> **Setup** -> **Get API Key**。
   - 點擊 **Generate API Key** 並妥善保存。

2. **Docker 登入**:
   使用以下指令登入（使用者名稱固定為 `$oauthtoken`）：
   ```bash
   docker login nvcr.io
   # Username: $oauthtoken
   # Password: <您的 API Key>
   ```

### 3. 環境啟動

1. **進入 Nix 開發環境**:
   ```bash
   nix develop
   ```

2. **配置環境變數**:
   ```bash
   cp .env.example .env
   # 根據您的系統路徑調整 .env 中的 WAYLAND_DISPLAY 等設定
   ```

3. **啟動 Isaac Sim**:
    ```bash
    just up
    ```
    *註：初次啟動會編譯 Shader，可能需要 3-5 分鐘，請使用 `just logs` 觀察進度。*

### 4. 運行 Bridge（Real↔Sim 分離）

本專案採用兩條獨立資料流（避免互相依賴/誤觸發）：
- **real2sim (Leader → Isaac Sim)**: Host 讀取 leader（Serial）並透過 ZMQ `5555` 發送關節角度。
- **sim2real (Isaac Sim → Follower)**: Isaac Sim 透過 ZMQ `5556` 發送「實際 joint positions」，Host 寫入 follower（Serial）。

重要：Serial 與 calibration 的設定以 `~/.cache/huggingface/lerobot/profile.json` 為單一事實來源。
- follower 預設 `/dev/ttyACM0`
- leader 預設 `/dev/ttyACM1`

#### 模式 A: WebRTC 遠端串流（Headless）
1. **啟動 Isaac Sim（Container）**:
   ```bash
   just webrtc
   ```
2. **連線**: 使用 Isaac Sim WebRTC Streaming Client 連接 `127.0.0.1`（UDP 47998, TCP 49100）。

#### 模式 B: 本地 X11 視窗
- **場景 A（簡單場景）**：
  ```bash
  just gui-bridge
  ```
- **場景 B（桌面場景 sim2real.usd）**：
  ```bash
  just gui-desk
  ```

兩個 GUI 模式預設都同時啟用：
- real2sim: SUB `tcp://127.0.0.1:5555`（按 Play 後才套用）
- sim2real: PUB `tcp://127.0.0.1:5556`（按 Play 後以 30Hz 發送）

### 5. Host 端指令（Serial）

#### real2sim（Leader → Isaac Sim）
- **Mock（測試用）**:
  ```bash
  just real2sim-mock
  ```
- **實機（讀 leader）**:
  ```bash
  just real2sim
  # 或覆寫
  just real2sim /dev/ttyACM1
  ```

#### sim2real（Isaac Sim → Follower）
- **Dry-run（不寫馬達，只印 targets）**:
  ```bash
  just sim2real-dry
  ```
- **實機（寫 follower）**:
  ```bash
  just sim2real
  # 或覆寫
  just sim2real /dev/ttyACM0
  ```

安全機制：`sim2real` 會在第一次 torque enable 前做對齊檢查（`--max-start-delta-raw`，預設 200），若 Isaac Sim 的目標姿態離真機太遠會拒絕啟用，避免暴衝。

## 🛠 常用指令 (Justfile)

本專案使用 `Justfile` 封裝常用操作：

| 指令 | 說明 |
| :--- | :--- |
| `just up` | 啟動 Isaac Sim 容器 |
| `just down` | 停止並移除容器 |
| `just logs` | 查看容器日誌 |
| `just shell` | 進入容器終端機 |
| `just webrtc` | 啟動 Isaac Sim（WebRTC Headless） |
| `just gui-bridge` | 啟動 Isaac Sim GUI（場景 A） |
| `just gui-desk` | 啟動 Isaac Sim GUI（桌面場景 `sim2real.usd`） |
| `just real2sim` | 啟動 leader→sim（使用 `profile.json`） |
| `just real2sim-mock` | 啟動 leader→sim（Mock Mode） |
| `just sim2real` | 啟動 sim→follower（使用 `profile.json` + calibration + safety gate） |
| `just sim2real-dry` | 啟動 sim→follower（Dry-run，不寫馬達） |


## 🔧 Calibration（必要）

`sim2real` 寫入真機前必須有 calibration 檔，否則很容易因為中心/範圍不一致造成暴衝。

- Profile（單一事實來源）：`~/.cache/huggingface/lerobot/profile.json`
- Calibration（常見路徑）：
  - leader：`~/.cache/huggingface/lerobot/calibration/teleoperators/so_leader/*.json`
  - follower：`~/.cache/huggingface/lerobot/calibration/robots/so_follower/*.json`

## 📁 目錄結構

- `docker/`: Docker Compose 設定與環境配置。
- `assets/`: 存放機器人 URDF/USD 模型檔案。
- `configs/`: 存放 Isaac Sim 場景與參數設定。
- `scripts/`: 宿主機使用的 Python/Bash 輔助腳本。
- `data/`: 數據採集結果（由容器映射出）。

##    Assets

- https://github.com/LightwheelAI/leisaac/releases

## 📝 開發規範

請參考 [AGENTS.md](./AGENTS.md) 了解詳細的代碼風格、測試與建置規範。

---
*Created for SO-101 Open Source Robotic Arm project.*
