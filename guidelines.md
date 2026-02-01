Project Guidelines: so101_sim2real_nix
1. 專案願景 (Project Vision)

本專案旨在 NixOS 環境下，透過 NVIDIA Isaac Sim 實現 SO-101 機器人手臂的 Sim-to-Real 橋接。利用 Docker 隔離複雜的顯卡依賴，並透過實體 Leader Arm 進行 Teleoperation 與數據採集。
2. 核心架構 (Core Architecture)

    Host (NixOS): 負責硬體驅動 (NVIDIA, Serial) 與基礎環境管理。

    Container (Isaac Sim): 運行 Omniverse 引擎、LeIsaac 橋接器與物理模擬。

    Bridge: 透過 /dev/ttyACM* 將實體 Leader Arm 的 Joint States 傳入容器。

3. 開發規範 (Development Standards)
3.1 環境配置 (Environment)

    不可直接在 NixOS 安裝 Isaac Sim: 必須使用專案目錄下的 docker-compose.yml 啟動。

    Nix Flake: 使用 flake.nix 建立開發環境，提供 docker-compose、git 以及用於輔助腳本的 python/go 工具。

    硬體存取: 啟動容器前，必須確認 dialout 群組權限。

3.2 目錄結構 (Directory Structure)
Plaintext

.
├── docker/                 # Dockerfile 與相關設定
├── scripts/                # 宿主機使用的輔助腳本 (如：硬體檢測)
├── assets/                 # SO-101 URDF 與 USD 模型檔案
├── configs/                # Isaac Sim 場景與機器人參數
├── .env.example            # 環境變數範本 (含 NGC API Key)
├── flake.nix               # Nix 開發環境定義
└── guidelines.md           # 本文件

4. 關鍵工作流程 (Key Workflows)
4.1 啟動流程

    nix develop 進入環境。

    複製並設定 .env 檔案（包含你的 NVIDIA_DRIVER_VERSION）。

    執行 docker-compose up -d 啟動 Isaac Sim。

    檢查硬體：ls /dev/ttyACM* 確保 Leader Arm 已連線。

4.2 數據採集 (Data Collection)

    所有採集的 .h5 或錄製數據應映射回宿主機的 data/ 目錄，避免容器刪除後數據丟失。

    採集時需標註 leader_id 與 session_timestamp。

5. 技術債與注意事項 (Technical Notes)

    GPU Passthrough: 若遇到 Vulkan 報錯，請檢查宿主機的 nvidia-container-toolkit 是否正確載入。

    Serial Latency: 若 Teleop 出現延遲，應檢查 setserial 設定，確保 /dev/ttyACM0 處於 low_latency 模式。

    NixOS Updates: 更新系統 Kernel 後，記得重新啟動 Docker 服務以確保 NVIDIA 模組匹配。

6. 定義完成 (Definition of Done)

    [ ] 實體 Leader Arm 能驅動 Isaac Sim 中的模型運動。

    [ ] 模擬器中的傳感器數據能正確寫入宿主機磁碟。

    [ ] 所有 Python 腳本皆能在容器內自動執行，無需手動配置依賴。
