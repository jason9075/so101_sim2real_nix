# SO-101 Sim-to-Real (NixOS + Isaac Sim)

本專案致力於在 NixOS 環境下，透過 NVIDIA Isaac Sim 實現 SO-101 機器人手臂的 Sim-to-Real 橋接。

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
   make up
   ```
   *註：初次啟動會編譯 Shader，可能需要 1-3 分鐘，請使用 `make logs` 觀察進度。*

## 🛠 常用指令 (Makefile)

本專案使用 `Makefile` 封裝常用操作：

| 指令 | 說明 |
| :--- | :--- |
| `make up` | 啟動 Isaac Sim 容器 |
| `make down` | 停止並移除容器 |
| `make logs` | 查看容器日誌 |
| `make shell` | 進入容器終端機 |
| `make clean` | 清除 `data/` 中的暫存數據 |

## 📁 目錄結構

- `docker/`: Docker Compose 設定與環境配置。
- `assets/`: 存放機器人 URDF/USD 模型檔案。
- `configs/`: 存放 Isaac Sim 場景與參數設定。
- `scripts/`: 宿主機使用的 Python/Bash 輔助腳本。
- `data/`: 數據採集結果（由容器映射出）。

## 📝 開發規範

請參考 [AGENTS.md](./AGENTS.md) 了解詳細的代碼風格、測試與建置規範。

---
*Created for SO-101 Open Source Robotic Arm project.*
