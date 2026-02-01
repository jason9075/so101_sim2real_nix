# Agent Instructions: so101_sim2real_nix

本文件為針對此專案開發的 AI Agent 指南。開發時請嚴格遵守以下規範。

## 1. 專案概覽 (Project Overview)
本專案致力於在 NixOS 環境下，透過 NVIDIA Isaac Sim 實現 SO-101 機器人手臂的 Sim-to-Real 橋接。
核心架構包含：
- **Host (NixOS)**: 硬體驅動與 Nix 環境管理。
- **Container (Docker/Isaac Sim)**: 物理模擬與數據採集。
- **Bridge**: 實體與模擬間的狀態同步。

## 2. 開發環境與指令 (Development Commands)

本專案使用 `flake.nix` 進行環境編排，並以 `Makefile` 封裝常用指令。

### 2.1 環境啟動
- 進入開發環境：`nix develop`
- 啟動 Isaac Sim：`docker-compose up -d`
- 停止容器：`docker-compose down`

### 2.2 建置、檢查與測試 (Build/Lint/Test)
所有開發輔助指令應定義在 `Makefile` 中。以下為標準目標：

| 任務 | 指令 | 說明 |
| :--- | :--- | :--- |
| **Lint** | `make lint` | 執行 Python (ruff), C++ (clang-format), Bash (shellcheck) 檢查 |
| **Format** | `make format` | 自動修復格式問題 |
| **Test All** | `make test` | 執行所有單元測試 |
| **Single Test**| `pytest <path_to_test>` | 執行特定 Python 測試檔 |
| **Build** | `make build` | 編譯必要的 C++ 插件或輔助工具 |
| **Clean** | `make clean` | 清除建置產物與暫存檔 |

### 2.3 執行特定測試範例
- **Python (單一函數)**: `pytest scripts/tests/test_hardware.py::test_serial_connection`
- **Python (標籤過濾)**: `pytest -m "hardware"`
- **C++**: `ctest -R <test_name>` (若有使用 CMake)
- **Bash**: `shellcheck scripts/*.sh`

### 2.4 Makefile 實作範例
```makefile
.PHONY: lint format test build clean

lint:
	ruff check .
	clang-format --dry-run --Werror src/*.cpp
	shellcheck scripts/*.sh

format:
	ruff format .
	clang-format -i src/*.cpp

test:
	pytest scripts/tests
	# ctest --output-on-failure

build:
	mkdir -p build && cd build && cmake .. && make
```

## 3. 目錄結構規範 (Directory Structure)

請遵循以下結構進行檔案操作，確保符合 `guidelines.md` 的定義：
- `docker/`: 存放 Dockerfile, docker-compose.yml 與容器環境變數。
- `scripts/`: 存放 Host 端的 Python/Bash 輔助腳本（如硬體通訊、數據後處理）。
- `assets/`: 存放機器人 URDF/USD 模型、貼圖與場景描述檔案。
- `configs/`: 存放 Isaac Sim 控制器參數、PID 設定與機器人 Kinematics 設定。
- `data/`: 數據採集結果（由容器映射出），此目錄應列入 `.gitignore`。
- `src/`: 存放 C++ 核心插件或高效能通訊組件。
- `flake.nix`: 專案環境定義與依賴管理中心。
- `Makefile`: 自動化任務定義。

## 4. 代碼風格指引 (Code Style)

### 4.1 通用原則
- **Nix-First**: 嚴禁使用 `sudo apt` 或 `pip install` 指令。所有依賴必須定義於 `flake.nix`。若需新增工具，請修改 `devShell` 的 `buildInputs`。
- **Pure Functions**: 優先使用純函數編程模式。在 Python 與 C++ 中，儘量減少全域變數的使用，並確保函數輸出僅取決於輸入參數。
- **Traditional Chinese**: 說明、註釋與對話使用繁體中文（台灣），技術名詞保留英文（例如：Buffer, Thread, Latency）。

### 4.2 Python 規範 (3.10+)
- **Type Safety**: 所有函數必須包含 Type Hints。
  ```python
  def get_joint_state(device_path: str) -> dict[str, float]:
      ...
  ```
- **Formatting**: 遵循 `ruff` 規範。行寬上限建議為 88 或 100 字元。
- **Naming**: 
  - 變數與函數：`snake_case`
  - 類別：`PascalCase`
  - 常數：`UPPER_SNAKE_CASE`
- **Error Handling**: 嚴禁使用 `try: ... except: pass`。必須捕捉具體異常並記錄 Traceback。
- **Logging**: 使用 `logging` 模組而非 `print()`，並根據嚴重程度選擇 DEBUG/INFO/WARNING/ERROR。

### 4.3 C++ 規範 (C++17/20)
- **Memory Safety**: 強制使用 RAII 與 Smart Pointers。
  - 避免 `std::shared_ptr` 除非確實有共享所有權需求，優先使用 `std::unique_ptr`。
  - 嚴禁手動調用 `malloc/free` 或 `new/delete`。
- **Standards**: 遵循 Modern C++ 實踐，利用 `auto` 推導與 `structured bindings` 提升可讀性。
- **Build**: 使用 CMake 管理，並提供 Nix 衍生式 (Derivation) 以整合進 `flake.nix`。

### 4.4 Bash 規範
- **Robustness**: 腳本開頭必須包含 `set -euo pipefail`。
- **Linting**: 必須通過 `shellcheck` 檢查，無視特定警告需標註 `SCxxxx`。
- **Environment**: 使用 `#!/usr/bin/env bash` 確保跨平台相容性。

### 4.5 導入規範 (Imports)
- **Python**: 
  1. Standard library (e.g., `os`, `sys`, `typing`)
  2. Third-party packages (e.g., `numpy`, `isaacsim`)
  3. Local imports (e.g., `from .utils import ...`)
  （每組之間空一行，按字母順序排列）


## 5. 硬體與 GPU 注意事項

- **Serial**: 存取 `/dev/ttyACM*` 前需確認使用者在 `dialout` 群組。若有延遲，需調用 `setserial` 進入 `low_latency` 模式。
- **NVIDIA**: Isaac Sim 依賴宿主機的 `nvidia-container-toolkit`。更新 Kernel 後必須重啟 Docker 服務。
- **Paths**: 容器內的數據路徑應映射至宿主機的 `data/` 目錄，確保持久化。

## 6. Git 提交規範

- 使用 Conventional Commits 規範（feat, fix, docs, style, refactor, test, chore）。
- 不可提交秘密資訊（如 NGC API Key），請使用 `.env` 搭配 `.env.example`。

## 7. AI Agent 專屬指令

- 在修改代碼前，優先搜尋 `guidelines.md` 與現有 `flake.nix` 以確保符合專案慣例。
- 產出代碼後，優先調用 `make lint` 進行自我驗證。
- 若需新增依賴，請修改 `flake.nix` 並告知使用者。

## 8. 常見任務執行指南 (Common Tasks)

### 8.1 建立新的 Python 輔助腳本
1. 在 `scripts/` 目錄下建立檔案。
2. 確保包含 `#!/usr/bin/env python3`。
3. 加入完整 Type Hints。
4. 撰寫對應的 `scripts/tests/test_*.py` 測試檔。

### 8.2 調整機器人模擬參數
1. 檢查 `configs/` 下的 YAML 或 JSON 設定。
2. 修改後建議先啟動模擬器進行視覺化確認（若環境支援）。
3. 確保更新對應的模型資產路徑（`assets/`）。

### 8.3 處理硬體通訊異常
1. 檢查 `/dev/ttyACM*` 權限與連線。
2. 確認 `setserial` 是否已設定 low_latency。
3. 若為通訊協議變動，需同步修改 `src/` (C++) 與 `scripts/` (Python) 中的 Bridge 邏輯。

---
*Created by opencode agent based on project context and senior engineer preferences.*
