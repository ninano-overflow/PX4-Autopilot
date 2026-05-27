# NINANO 에어프레임 넘버링 규정 (Airframe Numbering Scheme)

NINANO에서 정의하는 PX4 에어프레임 넘버링 시스템 가이드라인입니다.

## 넘버링 규칙 (Numbering Convention)

에어프레임 번호는 총 5자리 숫자로 구성됩니다: **`3` `B` `A` `C` `D`** (예: `31401`)

### 1. 첫 번째 자리 (`3`): 브랜드 식별자
*   **`3`**: NINANO (니나노) 독자 에어프레임 라인업을 나타냅니다.

### 2. 두 번째 자리 (`B`): 상세 타입 및 구성 (Detail Type & Configuration)
*   **`1`**: X Type MC (일반 X자)
*   **`2`**: + Type MC (일반 +자)
*   **`3`**: Fixed Wing (하이브리드/고정익)
*   **`4`**: Tailsitter (테일시터)
*   **`5`**: Lift & Cruise (리프트 & 크루즈)
*   **`6`**: Tilt Rotor (틸트로터)

### 3. 세 번째 자리 (`A`): 비행체 분류 (Vehicle Type)
*   **`4`**: Quadrotor (쿼드콥터) 계열

### 4. 네 번째 & 다섯 번째 자리 (`CD`): 넘버링 (Sequential Number)
*   동일 분류 및 상세 타입 내에서 부여되는 고유 일련번호 (`01` ~ `99`)

---

## 에어프레임 예시 및 번호 매핑 목록

| 에어프레임 번호 | 에어프레임 이름 | 타입 분류 | 상세 타입 설명 |
| :--- | :--- | :--- | :--- |
| **31401** | `31401_bubo_4s` | Quadrotor | Quad_MC_X |
| **32401** | - | Quadrotor | Quad_MC_+ |
| **33401** | - | Quadrotor | Quad_Fixed_Wing |
| **34401** | - | Quadrotor | Quad_Tailsitter |
| **35401** | - | Quadrotor | Quad_Lift_Cruise |
| **36401** | - | Quadrotor | Quad_Tilt_Rotor |

---

## 적용 방법

PX4 Autopilot 소스코드에서 본 넘버링을 적용하려면 다음 파일들을 수정해야 합니다.

1. **에어프레임 설정 파일 추가**
   - 경로: `ROMFS/px4fmu_common/init.d/airframes/`
   - 파일명: `[번호]_[이름]` (예: [31401_bubo_4s](file:///home/ninano/dev/PX4-Autopilot/ROMFS/px4fmu_common/init.d/airframes/31401_bubo_4s))

2. **빌드 목록 등록**
   - 경로: [CMakeLists.txt](file:///home/ninano/dev/PX4-Autopilot/ROMFS/px4fmu_common/init.d/airframes/CMakeLists.txt)
   - 내용: `px4_add_romfs_files` 블록 내에 파일 이름 추가
