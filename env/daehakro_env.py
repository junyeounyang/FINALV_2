"""대학로 도로용 학습 환경 — 수업의 SumoHighwayEnv 를 대학로에 연결하고, 관측과 보상을 대학로에 맞춘다.

수업 환경(env/sumo_env.py)은 직선 고속도로에 맞춰져 있다. 이 파일은 그 클래스를 물려받아
대학로에 필요한 부분만 바꾼다. sumo_env.py 는 고치지 않는다.

1) 연결
    도로        daehakro_builder.build() 가 만든 대학로
    AV 투입     경로 "r_ego" (이화장길 동쪽 끝 → 로터리 남서쪽), 차종 "ego_type"
    매 스텝     EmergencyAssist.step() — 긴급차 길 터주기·우선신호·꼬리물기 방지 (AV 는 건드리지 않는다)
    에피소드    SUMO 를 매번 다시 켜지 않고, 1시간 시뮬레이션을 이어 가며 AV 만 새로 넣는다
    제한속도    차로에서 읽는다 (어린이 보호구역 30, 그 밖 50 km/h)
    차로변경    모두 정책이 한다. SUMO 는 AV 의 차로를 스스로 바꾸지 않는다 (아래 5) 와 '차로변경' 설정 참고)

2) 관측 — 49차원, 범위 [-1, 1]
    [0..30]  수업 코드의 31차원 그대로. 단, 세 가지를 대학로에 맞게 고쳤다.
             · 바로 옆 차로의 앞차·뒤차 [5..8] (왼쪽) [13..16] (오른쪽): 나란히 있는 차도 보인다 (간격 0).
               수업 코드는 간격이 음수인 차 — 내 차와 앞뒤로 겹쳐 있거나(나란히) 범퍼 사이가 2.5 m 안쪽인 차 — 를
               '차 없음'으로 넣는다. 그래서 옆에 차가 나란히 달리는 동안 관측에는 옆 차로가 빈 것으로 보인다.
               안전 검사가 보는 것과 같은 차를 넣고, 그런 차의 간격은 0 으로 한다 (_neighbor)
             · 차로 연결성 [21..25]: '내 경로를 따라' 그 차로로 얼마나 더 갈 수 있는가.
               보도·버스 전용 차로는 없는 차로(-1)로 본다
             · 차로 밀도 [26..30]: 보도·버스 전용 차로는 꽉 찬 차로(1)로 본다
    [31]     지금 차로의 제한속도 / 50 km/h                       (보호구역 0.6, 그 밖 1)
    [32]     다음 신호의 정지선까지 거리 √(d / 100 m)             (100 m 안에 없으면 1)
    [33]     그 신호가 빨강이면 1
    [34]     그 신호가 노랑이면 1
    [35]     다음 횡단보도까지 거리 √(d / W)                      (W 안에 없으면 1, 올라서 있으면 0)
    [36]     그 횡단보도에서 기다려야 할 보행자가 있으면 1  (내 경로 쪽으로 건너는 중이거나 건너려는 사람)
    [37]     양보해야 하는 차가 있을 때: 내가 상충 지점까지 남은 거리 √(d / W)   (없으면 1)
    [38]     양보해야 하는 차가 있을 때: 그 차가 상충 지점까지 남은 거리 / W     (없으면 1)
    [39]     양보해야 하는 차의 속도 / 50 km/h                    (없으면 0)
    [40]     양보해야 하는 차가 있으면 1  (합류, 회전교차로 진입, 비보호 진입)
    [41]     뒤에서 다가오는 사이렌 차까지 거리 / 100 m           (없으면 1)
    [42]     그 사이렌 차가 내 차로 바로 뒤에서 오면 1
    [43]     앞차가 '정해진 정차' 중이면 1  (정류장의 버스, 아이를 태우는 차 — 신호 대기와 구분)
    [44]     앞차와의 간격 √(d / W)                               (W 안에 없으면 1)
    [45]     서야 할 곳 앞에 멈추려면 필요한 감속 / AV 의 최대 감속 (4.5 m/s²)   (서야 할 곳이 없으면 0)
             서야 할 곳 = 빨강·노랑 신호의 정지선, 보행자가 있는 횡단보도, 지금 들어가면 끼어들게 되는 양보선,
                          내 경로로 이어지지 않는 차로의 끝 (그 전에 옆 차로로 옮기면 사라진다).
             0.5 면 '보통 세기로 지금 밟아야 한다', 1 이면 '온 힘으로 밟아도 늦었다'
    [46]     앞차에 닿지 않으려면 필요한 감속 / AV 의 최대 감속        (앞차가 없거나 나보다 빠르면 0)
    [47]     차로 안내: 경로를 따라가려면 옮겨야 하는 차로 수 / 2     (왼쪽 +, 오른쪽 −. 지금 차로가 맞으면 0)
             내비게이션의 차로 안내와 같다. 지금 차로와 옆 차로들 가운데 '차로변경 없이 내 경로를 가장 멀리
             갈 수 있는 차로'까지의 차로 수. LANE_RANGE (1000 m) 앞까지만 내다본다
    [48]     지금 차로가 내 경로에서 끊기는 곳까지 거리 √(d / 1000 m)   (1000 m 안에 없으면 1)
    W = mdp_config.OBSERVATION["visibility"] (기본 50 m)
    √ 눈금: 멈출 곳까지의 거리는 가까울수록 중요한데, d / W 로는 1 m 가 0.02 밖에 안 된다.
            √ 를 쓰면 1 m = 0.14, 3 m = 0.24, 10 m = 0.45 로 가까운 곳이 넓게 펼쳐진다.
            [44] 는 수업 코드의 [9] (같은 간격을 d / W 로 나타낸 값)를 √ 눈금으로 한 번 더 넣은 것이다.
    [45] [46]: 거리와 속도가 따로 들어 있으면 '이 속도로 이 거리 안에 설 수 있나'를 신경망이 스스로 계산해 내야 한다
            (속도² ÷ 거리). 그 계산을 미리 해서 넣어 준다. 운전자가 어림하는 '지금 밟아야 하나'와 같은 값이다.

3) 보상 — 아래 REWARD 표. 수업 코드의 고속도로용 보상(train.py 의 collision_penalty 등)은 쓰지 않는다.
    값을 바꾸려면 아래 표를 고치거나, train.py 의 REWARD_OVERRIDES 에 같은 이름으로 적는다
    (예: red_light_penalty=20.0). 그러면 그 값이 우선한다.
    PPO 를 처음부터 학습할 때 쓰는 것 두 가지 (둘 다 기본값은 '끔'이라 BC 와 평가에는 영향이 없다):
    · env.penalty_scale     충돌·신호위반처럼 '일이 생겼을 때 한 번 주는 벌점'에 곱하는 값 (기본 1).
                            train.py 가 학습 초반에는 작게 두고 1 까지 올린다 (PENALTY_START, PENALTY_RAMP_STEPS).
                            처음부터 1 이면 PPO 는 "움직이면 손해"를 먼저 배워서 서 있기만 한다.
    · lane_guide_penalty    차로 안내(관측 [47])가 가리키는 차로에 있지 않은 동안 스텝마다 주는 감점 (기본 0).
                            경로 이탈 벌점은 한참 뒤에 한 번 오기 때문에, 그것만으로는 차로변경을 배우기 어렵다.

4) 시범 주행 (BC 용) — DaehakroEnv(..., demo=True). collect_demo.py 가 쓴다.
    SUMO 의 안전장치를 켠 채 SUMO 가 AV 의 속도를 정하고, 그 운전을 '정책이 냈어야 할 행동'으로 바꿔 돌려준다.
    차로변경은 시범에서도 정책과 똑같은 길(행동의 차로변경 값 → _apply_lane_change)로 한다:
    시범 운전자는 차로 안내(관측 [47])가 가리키는 쪽으로 차로변경 값을 낸다.
    옆 차 때문에 못 옮기고 있으면 그 차보다 조금 느리게 가서 뒤쪽 틈으로 빠진다 ('틈 맞추기', DEMO_SEEK).
    아래 '시범 주행' 설정과 _demo_command / _demo_action 참고.

5) 정책이 운전할 때(demo=False) AV 를 움직이는 것
    속도      정책의 가감속 출력 → traci.vehicle.setSpeed. SUMO 의 속도 안전장치는 끈다 (setSpeedMode 0).
              수업 코드와 같이 제한속도는 넘지 못한다 (대학로는 차로마다 값이 달라 그 차로의 제한속도를 쓴다).
    차로변경  정책의 차로변경 출력 → traci.vehicle.changeLane. 정책이 값을 내지 않으면 차로는 바뀌지 않는다.
              SUMO 의 차로변경 기능은 쓰지 않는다 (setLaneChangeMode 0, 수업 환경과 같다).
              SUMO 는 AV 의 차로를 스스로 바꾸지 않고, 받은 명령이 안전한지 따지지도 않고, 다른 차를 비켜 주게
              하지도 않는다. 받은 명령을 그대로 실행할 뿐이다.
              안전한지는 환경이 검사한다 — 수업 코드의 검사식과 기준값 그대로 (_lane_change_is_safe).
              통과하지 못한 명령은 실행하지 않는다. 옆 차로의 틈은 정책이 요청을 내는 동안 저절로 생겨야 한다.
    핸들      행동에 조향이 없다. 차로 안을 달리고 교차로에서 경로의 다음 구간으로 넘어가는 것은
              수업 환경과 같이 SUMO 가 한다.
    경로가 이어지지 않는 차로로 끝까지 가면 SUMO 는 차를 더 보내지 못하고 차로 끝에 세운다. 이것은 정책이
    세운 것이 아니므로, 그때는 '경로 이탈'로 에피소드를 끝낸다 (OFF_ROUTE_SLACK).

쓰는 법 — train.py / test.py / collect_pomdp_data.py 맨 위의 import 두 줄만 바꾼다.
    from env import road_builder             →  from env import daehakro_env as road_builder
    from env.sumo_env import SumoHighwayEnv  →  from env.daehakro_env import DaehakroEnv as SumoHighwayEnv

관측이 화면과 맞는지 눈으로 보기 (SUMO 안전장치를 켠 시험 주행, 1초마다 관측을 말로 출력):
    python -m env.daehakro_env --gui
"""
import math
import os

import numpy as np
import gymnasium as gym
from gymnasium import spaces

from env.sumo_env import SumoHighwayEnv   # 이 줄이 SUMO_HOME/tools 를 경로에 넣는다 (아래 traci 보다 먼저)
import traci
import traci.constants as tc

from env import daehakro_builder
from env import daehakro_config as cfg
from env.daehakro_emergency import EmergencyAssist
from env.daehakro_map import RoadMap


# ══════════════════════════════════════════════════════════════════
#  설정 — 바꿀 일이 있으면 여기서
# ══════════════════════════════════════════════════════════════════
EGO_ROUTE = "r_ego"         # daehakro_builder 가 만들어 둔 AV 경로
EGO_TYPE = "ego_type"       # daehakro_builder 가 만들어 둔 AV 차종

WARMUP_TIME = 300.0         # s. SUMO 를 새로 켠 뒤 AV 없이 돌려 도로에 차를 채우는 시간
LAST_DEPARTURE = 2600.0     # s. 이 시각이 지나면 다음 에피소드는 SUMO 를 새로 켜서 시작한다.
                            #    대학로 시뮬레이션은 3600 s 에 끝나므로, 마지막 에피소드도
                            #    최대 길이(MAX_STEPS = 1000 s)를 다 쓸 수 있게 남겨 둔다
MAX_STEPS = 2000            # 에피소드 최대 길이 (1000 s). SUMO 기본 운전자도 400~1,200 스텝이 걸린다
STUCK_TIME = 240.0          # s. 이만큼 계속 서 있으면 에피소드를 끝낸다.
                            #    SUMO 기본 운전자의 가장 긴 정지는 110 s 안팎이고,
                            #    SUMO 는 300 s 서 있는 차를 순간이동시키므로 그 사이 값으로 둔다
INSERT_WAIT = 240           # 스텝. 출발 지점이 막혀 있을 때 AV 가 들어갈 때까지 기다리는 한도

# ---- 차로변경: 모두 정책이 한다. SUMO 의 차로변경 기능은 쓰지 않는다 ----
#  행동의 차로변경 값이 ±0.5 를 넘으면 환경이 그 방향으로 한 차로 옮긴다 (수업 코드와 같다).
#  값이 그 안쪽이면 아무 명령도 보내지 않고, 그러면 차로는 바뀌지 않는다.
#  명령을 보내기 전에 환경이 확인하는 것 (하나라도 걸리면 보내지 않는다):
#    · 갈 수 있는 차로인가   없는 차로, 보도·버스 전용 차로, 실선(차로변경 금지 구간) 너머는 안 된다
#    · 이 구간에서 끝나는가  교차로 안, 이번 스텝에 교차로로 들어서게 될 때는 안 된다
#    · 안전한가              수업 코드의 안전 검사 (sumo_env.py 의 _lane_change_is_safe, 기준값은 mdp_config.py)
LANE_CHANGE_MODE = 0        # traci.vehicle.setLaneChangeMode 에 넣는 값. 수업 환경과 같은 0:
                            #  SUMO 는 AV 의 차로를 스스로 바꾸지 않고, 받은 명령을 안전한지 따지지 않고 그대로 실행한다.
                            #  명령이 기다리는 일이 없으므로 SUMO 의 다른 차가 AV 에게 자리를 내주는 일도 없다
LANE_SCAN = 60.0            # m. 안전 검사에서 옆 차로의 앞차·뒤차를 구간 경계 너머 이 거리까지 찾는다.
                            #  수업 코드의 검사는 옆 차로의 차를 두 가지로 찾는다: SUMO 의 이웃 조회(getNeighbors)와,
                            #  같은 구간에 있는 차를 직접 훑는 것(_scan_lane_neighbors).
                            #   ① 이웃 조회는 '범퍼 사이 거리 − 최소 간격 2.5 m'를 돌려주고, 수업 코드는 이 값이 음수면
                            #      '차 없음'으로 처리한다. 그래서 바로 옆에 나란히 있거나 2.5 m 안쪽으로 붙은 차가 빠진다.
                            #   ② 그런 차는 직접 훑기가 잡게 되어 있는데, 직접 훑기는 같은 구간 안의 차만 본다.
                            #  고속도로는 구간이 하나라 ②가 항상 통한다. 대학로는 구간이 짧아서(10 m 안팎인 곳도 많다)
                            #  내 차는 새 구간에 막 들어섰고 옆 차는 아직 앞 구간이나 교차로 안에 있는 때가 흔하다.
                            #  그러면 ①도 ②도 그 차를 못 보고, '옆이 비었다'며 그 차 위로 차로를 바꾼다.
                            #  그래서 옆 차로의 앞뒤로 이어지는 차로까지 따라가며 찾은 차에도 같은 검사식을 적용한다 (_side_gaps)
                            #  검사식과 기준값(mdp_config.py 의 lane_change_safety)은 바꾸지 않는다
SIDE_RANGE = LANE_SCAN + 40.0   # m. AV 둘레 이 거리 안에 있는 차의 차로·위치·속도·길이를 SUMO 에서 스텝마다 한 번에 받아 온다
                            #  (보행자를 받아 오는 것과 같은 방법). 옆 차로의 차를 찾을 때 차마다 따로 묻지 않아도 되어 빠르다
LANE_RANGE = 1000.0         # m. 관측의 '차로 안내' [47] [48] 가 내다보는 거리.
                            #  주변 차가 자리를 내주지 않으므로 옆 차로가 비어 있을 때 옮겨야 한다. 그러려면 일찍 알아야 한다.
                            #  대학로에 들어설 때 승용차 차로는 셋이고 AV 는 가장 오른쪽으로 들어선다. 로터리를 도는 길로 이어지는
                            #  차로는 로터리 600 m 앞에서 '가운데 차로'가 둘로 갈라지며 생긴다 (가장 왼쪽 차로는 그 앞 교차로에서
                            #  경로를 벗어난다). 그러니 먼저 가운데 차로에 가 있어야 한다.
                            #   · 600 m 를 내다볼 때: 갈라지는 곳에 와서야 '왼쪽으로 두 차로'를 알게 된다. 그때는 가운데 차로가
                            #     차 무리로 차 있어서, 나란히 달리다가 못 옮긴 채 로터리에 닿는 일이 생긴다.
                            #   · 1000 m 를 내다볼 때: 대학로에 들어서자마자 가운데 차로로 옮긴다 (큰길의 차가 지나간 틈에 들어선
                            #     직후라 옆이 비어 있다). 갈라지는 곳에서는 막 생긴 빈 차로로 한 번만 더 옮기면 된다.
                            #  (시범 운전자 240회 시험에서 로터리 입구에서 끝내 끼어들지 못한 횟수: 600 m 일 때 5회, 1000 m 일 때 0회)
LANE_END_GAP = 1.0          # m. 경로가 이어지지 않는 차로의 끝 앞에 두는 거리 (관측 [45] 의 '서야 할 곳')
OFF_ROUTE_SLACK = 1.0       # m. 경로가 이어지지 않는 차로의 끝을 이만큼 넘게 지나치려 하면 '경로 이탈'로 끝낸다.
                            #    그 차로로 계속 가면 실제로는 다른 길로 빠진다. SUMO 는 차를 차로 끝에 세워 두지만,
                            #    그건 정책이 세운 것이 아니므로 인정하지 않는다

SPEED_LIMIT_CAP = True      # True : 정책이 지금 차로의 제한속도를 넘기려 하면 가속을 '제한속도에 딱 맞는 값'으로 줄인다.
                            #        수업 코드에도 있는 장치다 (도로의 제한속도 위로는 속도를 낼 수 없다).
                            #        대학로는 차로마다 제한속도가 달라서(보호구역 30, 그 밖 50) 그 차로의 값을 쓴다
                            # False: 줄이지 않는다. 수업 코드의 상한(도로 전체 50 km/h)만 남고,
                            #        보호구역에서 30 km/h 를 지키는 것은 정책 몫이 된다

GUI_ZOOM = 600.0            # sumo-gui 에서 AV 를 따라갈 때의 확대(%). 클수록 가까이

# ---- 관측 ----
SIGNAL_RANGE = 100.0        # m. 이 거리 안의 다음 신호를 본다
SIREN_RANGE = 100.0         # m. 뒤에서 오는 사이렌 차를 보는 거리
PERSON_RANGE = 50.0         # m. AV 둘레 이 거리 안의 보행자를 SUMO 에서 받아 온다
PED_LOOKAHEAD = float(cfg.PED_LOOKAHEAD)   # m. 횡단보도 위에서 내 경로 쪽으로 걸어오는 보행자를 이 거리부터 본다
PED_PASSED = 1.8            # m. 보행자가 내 경로 중심을 이만큼 지나가면 관측에서 '지나갔다'고 본다.
                            #    SUMO 운전자의 기준(차 반폭 0.9 + 여유 0.5 + 사람 0.3)보다 조금 넉넉하게
PED_CLEAR = 1.1             # m. 보행자가 내 경로 중심을 이만큼 지나가면 내 차와 닿지 않는다 (차 반폭 + 사람).
                            #    '보행자 있는 횡단보도 진입' 감점은 이 기준으로 준다 — 이미 비켜난 보행자 때문에
                            #    감점하지 않게
PED_NEAR = 4.0              # m. 횡단보도 끝에서 이 거리 안까지 와 있는 보행자를 '건너려는 보행자'로 본다 (걸어서 약 3초)
YIELD_GAP = float(cfg.MINOR_GAP)           # s. 양보해야 하는 차 앞을 지날 때 필요한 시간 여유 (SUMO 운전자와 같은 기준)
RED_STATES = "ru"           # 멈춰야 하는 신호: r 빨강, u 빨강+노랑
SIGNAL_STOP_GAP = 1.0       # m. 정지선 앞에 두는 거리 (SUMO 운전자가 서는 자리와 같다)
PED_STOP_GAP = float(cfg.PED_YIELD_GAP)   # m. 보행자를 기다릴 때 횡단보도 앞에 두는 거리 (SUMO 운전자와 같다)

# ---- 시범 주행 (BC 용): SUMO 가 AV 를 운전하고, 그 운전을 '정책이 냈어야 할 행동'으로 바꿔 기록한다 ----
DEMO_GAP_MARGIN = (1.0, 3.0)   # m. 시범에서는 앞차와 이만큼 더 띄운다 (에피소드마다 이 범위에서 하나를 뽑는다).
                            #    SUMO 운전자는 대기 줄에서 충돌 기준(범퍼 간격 2.5 m)에 딱 맞춰 서므로,
                            #    그대로 따라 하면 조금만 밀려도 충돌이 된다. 값을 바꿔 가며 보여 주면
                            #    '이 거리 안쪽에서는 서 있는다'는 것을 넓은 구간에 걸쳐 배운다
DEMO_HOLD = -0.5            # 서 있는 동안(그리고 멈춰 서는 마지막 스텝)의 정답 행동 = 브레이크를 확실히 밟음.
                            #    0 근처로 두면 따라 하는 신경망이 조금씩 밀려 정지선·앞차 쪽으로 기어간다
DEMO_TAU = 1.8              # s. 시범 운전자가 앞차와 두는 시간 간격 (SUMO 기본은 1.0).
                            #    따라 하는 신경망은 시범보다 조금씩 가까이 붙는다. 시범이 넉넉해야 그 오차를 견딘다
DEMO_ACCEL = 2.0            # m/s². 시범 운전자의 가속 세기. AV 의 한계(2.6)보다 부드럽게
DEMO_DECEL = 2.5            # m/s². 시범 운전자가 평소에 쓰는 감속 세기. AV 의 한계(4.5)보다 훨씬 부드럽게 둬서
                            #    일찍·완만하게 멈춘다. 늦었을 때는 한계까지 밟는다 (아래 '급하게' 흔들기 참고).
                            #    2.5 보다 낮추지 않는다: 50 km/h 에서 노란불(3초) 안에 멈추지도 지나가지도 못하게 된다
DEMO_YIELD = True           # 시범 운전자가 사이렌 차에 길을 비켜 준다 (_yield_lane). 시범 관측의 사이렌 값도 그대로 둔다
YIELD_KEEP = 150.0           # m. 비켜 갈 옆 차로로 내 경로를 이만큼은 더 갈 수 있어야 한다
YIELD_URGENT = 100.0         # m. 지금 차로가 이 안에서 끊기면 사이렌 차가 뒤에 있어도 차로 안내를 따른다
YIELD_MAX_DIST = 100.0       # m. 사이렌 차가 이 거리 안에 왔을 때부터 비킨다
DEMO_SEEK = True            # 틈 맞추기. 차로를 옮겨야 하는데(차로 안내가 켜져 있는데) 옆 차로의 차 때문에 못 옮기고 있으면
                            #    시범 운전자는 그 차보다 조금 느리게 간다. 그러면 그 차가 앞으로 나가고 뒤쪽 틈이 옆으로 온다.
                            #    이게 없으면 옆 차와 같은 속도로 나란히 달리기만 하다가 차로를 못 바꾼 채 로터리에 닿는다
                            #    (SUMO 가 정해 주는 속도는 내 차로의 앞차만 보고 정한 것이라 옆 차로와는 무관하다).
                            #    AV 는 이 동작을 시범에서 가감속으로 배운다. 정책이 운전할 때 환경이 해 주는 것은 없다
DEMO_SEEK_SLOWER = 2.5      # m/s. 막고 있는 옆 차보다 이만큼 느리게 (9 km/h)
DEMO_SEEK_DECEL = 1.5       # m/s². 틈 맞추기로 속도를 줄일 때의 세기 (가속 페달에서 발을 떼는 정도)
DEMO_SEEK_MIN = 4.0         # m/s. 막고 있는 옆 차가 이보다 느리면(정체 줄) 하지 않는다. 같이 느려지면 내 차로만 막는다
DEMO_STOP_NEAR = 8.0        # m. 서 있는 이유로 인정하는 거리: 빨간불 정지선·보행자 있는 횡단보도가 이 안에 있을 때
# 흔들기 (아래 demo_noise 확률로 시작. 두 가지 중 하나를 고른다. 흔드는 동안은 저장하지 않는다)
#   굼뜨게: SUMO 가 내려던 것보다 약한 가감속(브레이크 포함)을 넣는다
#           → 엉뚱한 곳에서 느려지거나 선다 → 그 뒤 '다시 출발하는' 장면이 시범에 들어간다
#   급하게: 잠깐 '늦게 밟고 바짝 붙는 운전자'로 바꾼다 (감속 = AV 의 한계, 시간 간격 DEMO_RUSH_TAU)
#           → 평소보다 빠르고 가까운 상태가 된다 → 그 뒤 '세게 밟아 바로잡는' 장면이 시범에 들어간다
#   둘 다 SUMO 의 안전장치 안에서 일어나므로 흔들기 때문에 사고가 나지는 않는다.
DEMO_NOISE_ACCEL = (-0.8, 0.2)   # 굼뜨게: 그때 넣는 가감속의 범위
DEMO_NOISE_STEPS = (1, 10)       # 굼뜨게: 한 번의 길이 (스텝, 0.5~5초)
DEMO_RUSH_TAU = 1.0              # 급하게: 그동안의 시간 간격 (s)
DEMO_RUSH_STEPS = (2, 8)         # 급하게: 한 번의 길이 (스텝, 1~4초)
DEMO_RUSH_BRAKING = 1.5          # 급하게: 속도를 줄이는 중일 때는 demo_noise 의 이 배수만큼 더 자주 시작한다
                                 #         (멈추러 가는 길에 흔들어야 '늦었을 때 세게 밟는' 장면이 생긴다)

EXTRA_OBS = (               # 관측 [31..48] 의 이름 (순서대로)
    "speed_limit", "signal_dist", "signal_red", "signal_yellow",
    "crosswalk_dist", "crosswalk_ped",
    "yield_my_dist", "yield_foe_dist", "yield_foe_speed", "yield_needed",
    "siren_dist", "siren_same_lane", "leader_stopped", "leader_near",
    "stop_need", "close_need",
    "lane_guide", "lane_end",
)

# ══════════════════════════════════════════════════════════════════
#  보상 — 스텝마다 더하는 값과, 일이 생겼을 때 한 번 주는 값
#
#  크기를 정한 기준 (γ = 0.99 일 때):
#    · 막힘 없이 달리면 스텝당 +0.09 (진행 0.1 − 시간 0.01). 끝까지 가면 진행 보상 합이 약 +23.
#    · 빨간불에서 60초(120스텝) 기다리는 대신 그냥 지나가서 얻는 이득은 많아야 7 정도다.
#      그래서 신호위반 감점(15)을 그보다 크게 두었다. 횡단보도·양보도 같은 방식으로 정했다.
#    · 충돌은 그 뒤의 보상을 모두 잃는 데 더해 큰 감점을 준다.
# ══════════════════════════════════════════════════════════════════
REWARD = {
    # ---- 스텝마다 ----
    "progress_weight": 0.1,           # 진행: 지금 차로의 제한속도로 달리면 이 값, 서 있으면 0
    "time_penalty": 0.01,             # 스텝마다 감점 (가만히 서 있기만 하는 정책 방지)
    "tailgate_penalty": 0.05,         # 앞차에 바짝 붙음: 스텝당 최대 이 값 (가까울수록 큼)
    "tailgate_gap": 1.0,              #   기준 간격 = tailgate_gap + tailgate_time × 내 속도  (m)
    "tailgate_time": 0.5,             #   간격은 SUMO 방식대로 '범퍼 사이 거리 − 2.5 m' 로 잰다.
                                      #   이 값이 0 미만이 되면 SUMO 가 충돌로 기록한다
                                      #   (SUMO 기본 운전자는 대기 줄에서 정확히 0 에 선다)
    "siren_block_penalty": 0.1,       # 사이렌 차의 차로 앞을 막고 있는 동안 스텝당
    # ---- 일이 생겼을 때 한 번 ----
    "goal_bonus": 10.0,               # 도착 (에피소드 종료)
    "crash_penalty": 20.0,            # 차와 충돌 (에피소드 종료)
    "crash_pedestrian_penalty": 40.0, # 보행자와 충돌 (에피소드 종료)
    "red_light_penalty": 15.0,        # 빨간불에 정지선을 넘음
    "crosswalk_penalty": 10.0,        # 보행자가 건너는(건너려는) 횡단보도에 들어감
    "yield_penalty": 8.0,             # 양보해야 하는 차 바로 앞으로 끼어듦
    "stuck_penalty": 5.0,             # 오래 서 있어서 에피소드가 끝남
    "off_route_penalty": 10.0,        # 경로가 이어지지 않는 차로로 끝까지 가서 경로를 벗어남 (에피소드 종료)
    "lane_change_cost": 0.0,          # 실제로 일어난 차로변경 1회당 (수업 코드와 같은 이유로 처음엔 0)
    "refused_lane_change_cost": 0.0,  # 실행되지 않은 차로변경 명령 1회당 — 없는 차로, 실선, 교차로 안, 안전 검사 탈락 (처음엔 0)
    "lane_guide_penalty": 0.0,        # 차로 안내가 가리키는 차로에 있지 않은 동안 스텝당, 옮겨야 하는 차로 수만큼
                                      #   (0 이면 끔. PPO 를 처음부터 학습할 때 train.py 에서 0.05 로 켠다)
}

# 일이 생겼을 때 주는 벌점 가운데 env.penalty_scale 을 곱하는 것 (도착 보너스와 스텝마다 주는 값에는 곱하지 않는다)
SCALED_PENALTIES = ("crash_penalty", "crash_pedestrian_penalty", "red_light_penalty", "crosswalk_penalty",
                    "yield_penalty", "off_route_penalty")

# 수업 클래스가 요구하는 도로·차량 설정. AV 가감속 한계는 ego_type 과 같은 값이다.
_ROAD = {"length": 1516.0, "num_lanes": 1, "speed_limit": cfg.SPEED["default"]}
_EGO = {"accel": 2.6, "decel": 4.5, "depart_lane": "best", "depart_speed": 0}
_INF = float("inf")
_CAR_LENGTH = 4.6           # m. ego_type 의 길이
_PED_HALF = 1.4             # m. SUMO 가 보행자를 볼 때 쓰는 차 반폭 (0.9) + 여유 (0.5)
_CROSSING_WIDTH = 6.0       # m. 횡단보도를 차 진행 방향으로 잰 폭의 넉넉한 값


def _near(dist, full):
    """거리를 0~1 로 바꾼다. 가까운 곳을 자세히 보도록 √ 눈금을 쓴다 (1 m 와 3 m 가 뚜렷이 다른 값이 되게)."""
    return math.sqrt(min(max(dist, 0.0) / full, 1.0))


_ROAD_FILES = ("sumocfg", "gui.xml", "net.xml", "rou.xml", "tll.xml", "stops.xml", "deco.xml")   # 도로 파일 (sumocfg 를 맨 마지막에 쓴다)
_ROAD_CODE = ("daehakro_builder.py", "daehakro_config.py", "daehakro_deco.py")                   # 도로 파일을 만드는 코드


def _built_road():
    """만들어 둔 도로 파일을 그대로 써도 되면 그 sumocfg 경로, 다시 만들어야 하면 None.

    그대로 쓰는 조건 (모두 만족):
      · 도로 파일이 모두 있다
      · sumocfg 가 나머지 도로 파일보다 새것이다 — 빌더가 맨 마지막에 쓰는 파일이므로, 끝까지 만들어졌다는 뜻
      · sumocfg 가 도로를 만드는 코드(_ROAD_CODE)보다 새것이다 — 코드를 고쳤으면 다시 만든다
    """
    here = os.path.dirname(os.path.abspath(__file__))
    files = [os.path.join(here, "sumo", "daehakro", "daehakro." + ext) for ext in _ROAD_FILES]
    code = [os.path.join(here, name) for name in _ROAD_CODE]
    try:
        done = os.path.getmtime(files[0])
        if (all(os.path.getmtime(p) <= done for p in files[1:])
                and all(os.path.getmtime(p) < done for p in code)):
            return files[0]
    except OSError:                                  # 없는 파일이 있다
        pass
    return None


def build(road=None, ego=None, traffic=None, out_dir=None):
    """road_builder.build(ROAD, EGO, TRAFFIC, 폴더) 자리에 그대로 들어간다.

    인자는 쓰지 않는다. 대학로 SUMO 파일(env/sumo/daehakro/)의 sumocfg 경로를 돌려준다.
    도로 파일은 한 번 만들어 두면 다시 만들지 않는다 (_built_road 의 조건 참고).
    왜: 수집·평가·학습은 SUMO 를 몇 분마다 새로 켜면서 도로 파일을 다시 읽는다. 다른 터미널에서 프로그램을
        하나 더 시작할 때마다 도로 파일을 다시 쓰면, 쓰는 도중의 파일을 읽은 쪽이 죽을 수 있다.
        다시 만들지 않으면 여러 프로그램을 언제 시작해도 서로 건드리지 않는다. (빌더는 매번 같은 파일을 만들므로
        다시 만들든 그대로 쓰든 도로와 교통은 같다.)
    도로 코드(_ROAD_CODE)를 고치면 다음 실행 때 자동으로 다시 만든다. 그때는 돌고 있는 다른 프로그램이 없을 때 실행한다.
    강제로 다시 만들려면: python -m env.daehakro_builder
    """
    sumocfg = _built_road()
    reused = sumocfg is not None
    if not reused:
        sumocfg = daehakro_builder.build(verbose=False)
    print(f"[daehakro_env] 대학로 도로를 사용합니다: {sumocfg}" + ("  (만들어 둔 파일 그대로)" if reused else "  (새로 만듦)"))
    return sumocfg


class DaehakroEnv(SumoHighwayEnv):
    def __init__(self, cfg_path, road, ego, mdp_sim, mdp_obs, action, reward,
                 traffic=None, gui=False, gui_autostart=True, demo=False):
        """인자는 SumoHighwayEnv 와 같다 (스크립트를 고치지 않아도 되게).

        고속도로용 road / ego / traffic 은 쓰지 않고 대학로 값으로 바꿔 넣는다.
        reward 에서는 REWARD 표에 있는 이름만 읽어 표의 값을 덮어쓴다.
        demo=True 면 시범 주행: step() 에 넘긴 행동은 무시하고 SUMO 가 운전한다. info 로 돌려주는 것:
            demo_action       '정책이 냈어야 할 행동' [가감속, 차로변경]. 정답이 없는 스텝은 None (저장하지 않는다)
            demo_obs          그 행동과 짝이 되는 관측 (SUMO 운전자가 그 스텝에 실제로 본 신호·보행자로 맞춘 것)
            demo_disturbed    흔드는 스텝인가
            demo_unexplained  이유가 관측에 보이지 않는데 서 있던 스텝인가

        시범 주행에서 self.demo_noise (0~1, 스텝당 확률) 를 주면 '흔들기'를 한다 (위 설정의 '흔들기' 참고).
        왜 필요한가: 시범 그대로만 배운 신경망은 시범 운전자가 한 번도 가 보지 않은 상태 — 앞이 비었는데 어정쩡한
        자리에 서 있거나, 멈출 곳에 너무 빠르게 다가간 상태 — 에서 어떻게 해야 할지 모른다. 작은 실수가 쌓여 그런
        상태에 들어서면 다시 출발하지 못하거나 제때 서지 못한다. 흔들기는 시범 운전자를 일부러 그런 상태로
        보낸 뒤 바로잡는 장면을 시범에 넣는다.
        """
        sim = {**mdp_sim,
               "step_length": cfg.SIM["step_length"],   # 대학로 모델을 검증한 0.5 s
               "max_steps": MAX_STEPS,
               "warmup_steps": 0,                       # 차 채우기는 reset() 에서 WARMUP_TIME 으로 한다
               "gui_zoom": GUI_ZOOM}
        super().__init__(cfg_path, _ROAD, _EGO, sim, mdp_obs, action, reward,
                         traffic=None,                  # 고속도로용 '차 미리 깔기' 끔
                         gui=gui, gui_autostart=gui_autostart)
        if not hasattr(traci.vehicle, "getJunctionFoes"):
            raise RuntimeError("이 환경은 traci.vehicle.getJunctionFoes 가 있는 SUMO 버전이 필요합니다. "
                               "SUMO 를 최신 버전으로 올려 주세요.")
        # 차로변경 명령은 보낸 그 스텝에만 살아 있게 한다 (수업 값은 1초 = 2스텝).
        # SUMO 는 명령을 '몇 번 차로로'로 기억하는데, 그 사이 다음 구간으로 넘어가면 차로 번호가 달라져
        # 정책이 내지 않은 차로변경이 될 수 있다. 계속 옮기고 싶으면 정책이 스텝마다 값을 내면 된다.
        self.lane_change_duration = 0.5 * self.step_length
        # 수업 코드 안전 검사의 '구간 처음 60 m 는 차로변경 금지'는 고속도로 입구용이다. 대학로는 구간이 그보다
        # 짧은 곳이 많아 끈다. 나머지 기준값은 mdp_config 의 값 그대로 쓴다.
        self.lc_spawn_protect = 0.0
        self.net_file = os.path.join(os.path.dirname(os.path.abspath(cfg_path)), "daehakro.net.xml")
        self.map = RoadMap(self.net_file)
        self.sim_end = float(cfg.SIM["end"])
        self.assist = None

        # 대학로 보상 (표 + train.py 에서 같은 이름으로 넘어온 값)
        self.drw = {**REWARD, **{k: v for k, v in (reward or {}).items() if k in REWARD}}
        # 벌점 크기 (SCALED_PENALTIES 에 곱한다). train.py 가 학습 중에 바꾼다. 평가·시범·BC 에서는 1 그대로다
        self.penalty_scale = 1.0

        # 관측: 수업 코드 31차원 + 대학로 18차원
        self.base_obs_dim = self.obs_dim
        self.obs_dim = self.base_obs_dim + len(EXTRA_OBS)
        self.observation_space = spaces.Box(-1.0, 1.0, shape=(self.obs_dim,), dtype=np.float32)
        self._i_red, self._i_yellow, self._i_ped, self._i_siren, self._i_siren_lane, self._i_need = (
            self.base_obs_dim + EXTRA_OBS.index(k)
            for k in ("signal_red", "signal_yellow", "crosswalk_ped", "siren_dist", "siren_same_lane", "stop_need"))

        self._route = []           # AV 경로 (edge 목록)
        self._reach = {}           # 경로의 각 차로 → 차로변경 없이 경로를 따라 갈 수 있는 거리 (그 차로 시작점 기준)
        self._move_plan = 0.0      # 이번 스텝에 가려는 거리 (차로변경을 이 구간 안에서 끝낼 수 있는지 볼 때 쓴다)
        self._overrun = (None, 0.0)   # (차로, 경로가 이어지지 않는 그 차로의 끝을 넘어가려 한 거리의 합)
        self._snap = None          # 가장 최근에 본 AV 주변 상황 (_look 결과)
        self._still = 0            # AV 가 연속으로 서 있는 스텝 수
        self._foe_passed = (-1e9, 0.0)   # 양보 대상이 마지막으로 상충 지점을 지난 (시각, 그때 내 남은 거리)
        self._fresh = True         # 지난 에피소드가 도로를 어지럽히지 않았는가 (아니면 SUMO 를 새로 켠다)
        self.demo = bool(demo)     # 시범 주행 여부
        self.demo_noise = 0.0      # 시범 주행 중 흔들기를 시작할 확률 (스텝당)
        self._noise_kind = None    # 진행 중인 흔들기: None / "slow" (굼뜨게) / "rush" (급하게)
        self._noise_left = 0       # 진행 중인 흔들기의 남은 스텝
        self._noise_accel = 0.0    # 굼뜨게 흔드는 동안 넣는 가감속
        self._braking = False      # 바로 앞 스텝에 시범 운전자가 속도를 줄였는가
        self._rng = np.random.default_rng(np.random.randint(0, 2**31 - 1))
        self._gap_shift = 0.0      # 시범 주행에서 앞차 간격 읽을 때 더해 주는 값 (아래 _neighbor 참고)
        self._side_at = None       # _side_gaps 결과를 기억해 둔 시뮬레이션 시각 (같은 시각에는 다시 찾지 않는다)
        self._side_memo = {}       # 그 시각의 결과: 방향(+1 / −1) → (앞차, 뒤차)
        self._cw_busy = 0.0        # 차체가 횡단보도를 다 벗어나는 지점 (달린 거리 기준). 그 전에는 흔들지 않는다
        self._last_obs = None      # 가장 최근에 돌려준 관측
        self.look_errors = 0       # _look 에서 SUMO 조회가 실패한 횟수 (정상이면 0 에 가깝다)

    # ==================================================================
    # 시뮬레이션 진행
    # ==================================================================
    def _advance(self):
        """AV 행동 없이 시뮬레이션 1 스텝 (차 채우기, AV 가 들어오길 기다릴 때)."""
        self.assist.step()
        traci.simulationStep()

    def _restart(self):
        """SUMO 를 새로 켜고 도로에 차가 찰 때까지 돌린다."""
        if self._started:
            traci.close()
            self._started = False
        self._start_sumo()
        # 긴급차 보조. skip: AV 는 보조가 움직이지 않는다.
        self.assist = EmergencyAssist(net_file=self.net_file, skip=(self.ego_id,))
        while traci.simulation.getTime() < WARMUP_TIME:
            self._advance()

    def _insert_ego(self):
        """AV 를 경로 시작점에 넣고 도로에 나타날 때까지 기다린다. 끝내 못 들어가면 False."""
        # 같은 id 를 바로 다시 쓸 수 없으면 한 스텝 지난 뒤 다시 시도한다.
        for _ in range(3):
            try:
                traci.vehicle.add(self.ego_id, EGO_ROUTE, typeID=EGO_TYPE,
                                  departLane="best", departSpeed="0")
                break
            except traci.TraCIException:
                self._advance()
        else:
            raise RuntimeError(
                f"AV 를 넣지 못했습니다. 경로 '{EGO_ROUTE}' 와 차종 '{EGO_TYPE}' 이 "
                f"대학로 rou.xml 에 있는지 확인하세요.")
        # 출발 지점이 막혀 있으면 SUMO 가 투입을 미룬다. 들어올 때까지 기다린다.
        for _ in range(INSERT_WAIT):
            self._advance()
            if self.ego_id in traci.vehicle.getIDList():
                return True
        return False

    # ==================================================================
    # 에피소드 초기화
    # ==================================================================
    def reset(self, seed=None, options=None):
        gym.Env.reset(self, seed=seed)
        self._side_at = None

        if not self._started or not self._fresh or traci.simulation.getTime() >= LAST_DEPARTURE:
            self._restart()
        elif self.ego_id in traci.vehicle.getIDList():
            # 시간초과·보행자 충돌 등으로 끝나 도로에 남아 있는 지난 에피소드의 AV 를 치운다.
            # 둘레 조회 등록(보행자, 차)을 먼저 풀어야 한다 (등록이 남은 채 차를 지우면 SUMO 가 오류로 꺼진다).
            for domain, dist in ((tc.CMD_GET_PERSON_VARIABLE, PERSON_RANGE),
                                 (tc.CMD_GET_VEHICLE_VARIABLE, SIDE_RANGE)):
                try:
                    traci.vehicle.unsubscribeContext(self.ego_id, domain, dist)
                except traci.TraCIException:
                    pass
            traci.vehicle.remove(self.ego_id)
            self._advance()

        self._fresh = True
        self._foe_passed = (-1e9, 0.0)
        for attempt in range(3):
            if self._insert_ego():
                break
            # 출발 지점이 오래 막혀 있다 (앞선 에피소드가 만든 정체 등) → SUMO 를 새로 켜서 다시 시도한다.
            self._restart()
        else:
            raise RuntimeError(
                f"AV 투입 실패: SUMO 를 새로 켜도 출발 지점이 {INSERT_WAIT} 스텝 넘게 막혀 있습니다.")

        if self.demo:
            # 시범 주행: SUMO 의 안전장치를 켜 둔다 → 신호·보행자·앞차에 맞춰 SUMO 가 속도를 정한다.
            # 27 = 안전장치는 모두 켜되 '평소 감속 세기까지만' 제한(4)은 푼다. 늦었을 때 한계까지 밟을 수 있게.
            traci.vehicle.setSpeedMode(self.ego_id, 27)
            traci.vehicle.setSpeedFactor(self.ego_id, 1.0)      # 제한속도를 넘지 않게
            margin = float(self._rng.uniform(*DEMO_GAP_MARGIN))
            ahead = traci.vehicle.getLeader(self.ego_id, 20.0)
            if ahead and ahead[0]:
                # 막 들어선 자리 바로 앞에 차가 있으면 그 사이만큼만 띄운다
                # (SUMO 는 '범퍼 간격 < 최소 간격'을 충돌로 기록하므로, 간격 기준을 키우는 순간 충돌이 될 수 있다)
                margin = min(margin, max(float(ahead[1]) - 0.3, 0.0))
            traci.vehicle.setMinGap(self.ego_id, traci.vehicle.getMinGap(self.ego_id) + margin)
            traci.vehicle.setAccel(self.ego_id, DEMO_ACCEL)
            traci.vehicle.setEmergencyDecel(self.ego_id, self.max_decel)   # 아무리 급해도 AV 의 한계까지만
            traci.vehicle.setImperfection(self.ego_id, 0.0)     # SUMO 운전자의 무작위 굼뜸을 끈다 (정답이 흔들리지 않게)
            self._rush(False)
            self._noise_kind, self._noise_left, self._braking = None, 0, False
            self._gap_shift = margin
        else:
            # 수업 환경과 같다: SUMO 의 자동 안전장치를 끄고 정책이 속도를 온전히 제어한다.
            traci.vehicle.setSpeedMode(self.ego_id, 0)
            self._gap_shift = 0.0
        # 차로변경: 수업 환경과 같다. SUMO 는 AV 의 차로를 스스로 바꾸지 않는다. 행동의 차로변경 값으로만 바뀐다
        # (시범 주행도 같다).
        traci.vehicle.setLaneChangeMode(self.ego_id, LANE_CHANGE_MODE)
        # SUMO 의 차로변경 모델이 AV 를 두고 '경로상 옮겨야 한다'는 판단 자체를 하지 않게 한다.
        # (이 설정을 모르는 SUMO 버전이면 넘어간다. 위의 LANE_CHANGE_MODE 만으로도 SUMO 는 차로를 스스로 바꾸지 않는다.)
        try:
            traci.vehicle.setParameter(self.ego_id, "laneChangeModel.lcStrategic", "-1")
        except traci.TraCIException:
            pass

        self._route = list(traci.vehicle.getRoute(self.ego_id))
        self._reach = self.map.reach_table(self._route)
        self._overrun, self._move_plan = (None, 0.0), 0.0
        # AV 둘레의 보행자를 스텝마다 한 번에 받아 오도록 등록한다 (횡단보도 관측용).
        traci.vehicle.subscribeContext(
            self.ego_id, tc.CMD_GET_PERSON_VARIABLE, PERSON_RANGE,
            [tc.VAR_ROAD_ID, tc.VAR_LANEPOSITION, tc.VAR_ANGLE, tc.VAR_POSITION, tc.VAR_NEXT_EDGE])
        # AV 둘레의 차도 같은 방법으로 받아 온다 (옆 차로의 앞차·뒤차 찾기용, _side_gaps).
        traci.vehicle.subscribeContext(
            self.ego_id, tc.CMD_GET_VEHICLE_VARIABLE, SIDE_RANGE,
            [tc.VAR_LANE_ID, tc.VAR_LANEPOSITION, tc.VAR_SPEED, tc.VAR_LENGTH])

        if self.gui:
            try:
                view_ids = traci.gui.getIDList()
                if view_ids:
                    if self.gui_track_ego:
                        traci.gui.trackVehicle(view_ids[0], self.ego_id)
                    traci.gui.setZoom(view_ids[0], self.gui_zoom)
            except traci.TraCIException:
                pass

        self.step_count = 0
        self._still = 0
        self._cw_busy = 0.0
        return self._get_obs(), {}

    # ==================================================================
    # AV 주변 상황 조회 — 스텝마다 한 번. 관측과 보상이 같은 값을 쓴다.
    # ==================================================================
    def _look(self):
        ego = self.ego_id
        lane = traci.vehicle.getLaneID(ego)
        pos = traci.vehicle.getLanePosition(ego)
        v = traci.vehicle.getSpeed(ego)
        inside = lane.startswith(":")                # 교차로 안을 지나는 중
        s = {"lane": lane, "pos": pos, "v": v, "inside": inside,
             "odo": traci.vehicle.getDistance(ego),  # 출발 후 달린 거리
             "limit": self.map.speed.get(lane, self.vmax),
             "best": {}, "tls": None, "cw": None, "foe": None, "siren": None,
             "gap": None, "lead_v": 0.0, "leader_stopped": False}
        # 차로 안내: (옮겨야 하는 차로 수, 지금 차로로 더 갈 수 있는 거리, 이 차로가 이 구간 끝에서 끊기는가)
        s["guide"], s["reach"], s["dead"] = self._lane_plan(lane, pos, inside)
        try:
            # 차로별 '경로를 따라 더 갈 수 있는 거리' (그 차로의 시작점부터 잰 값)
            if not inside:
                s["best"] = {b[0]: float(b[1]) for b in traci.vehicle.getBestLanes(ego)}

            # 다음 신호: (신호등 id, 정지선까지 거리, 색, 그 신호등 안에서의 번호)
            nt = traci.vehicle.getNextTLS(ego)
            if nt and nt[0][2] <= SIGNAL_RANGE:
                s["tls"] = (nt[0][0], float(nt[0][2]), nt[0][3], int(nt[0][1]))

            # 다음 횡단보도: (id, 가장자리까지 거리, 기다려야 할 보행자가 있는가, 지금 들어가면 닿을 수 있는가)
            path = self.map.path_ahead(lane, pos, self._route, self.visibility + 5.0)
            nc = self.map.next_crossing(path)
            if nc and nc[1] <= self.visibility:
                s["cw"] = (nc[0], nc[1]) + self._ped_risk(nc[0], nc[2])

            s["foe"] = self._yield_foe(v)
            s["siren"] = self._siren()

            # 앞차: 간격은 SUMO 방식대로 '충돌 기준까지 남은 거리' (범퍼 사이 거리 − 2.5 m)
            ld = traci.vehicle.getLeader(ego, self.visibility)
            if ld and ld[0] and 0.0 <= ld[1] <= self.visibility:
                s["gap"] = float(ld[1]) + self._gap_shift
                s["lead_v"] = float(traci.vehicle.getSpeed(ld[0]))
                s["leader_stopped"] = bool(traci.vehicle.isStopped(ld[0]))
        except traci.TraCIException:
            self.look_errors += 1                    # 조회 중 사라진 차 등: 못 읽은 항목은 '없음'으로 둔다
        return s

    def _ped_may_start(self, c):
        """그 횡단보도로 보행자가 새로 들어설 수 있는가 (신호 없는 곳은 항상, 있는 곳은 보행 신호가 초록일 때)."""
        if c["tl"] is None:
            return True
        state = traci.trafficlight.getRedYellowGreenState(c["tl"])
        return c["idx"] < len(state) and state[c["idx"]] in "Gg"

    def _ped_risk(self, cid, my_s):
        """횡단보도 cid 에서 보행자 때문에 기다려야 하는가. 내 경로는 횡단보도 위 위치 my_s 를 지난다.

        Returns: (기다려야 하는가, 지금 들어가면 닿을 수 있는가)

        기다려야 하는가 — 관측의 crosswalk_ped. 다음 중 하나면 True.
          · 횡단보도 위에서 내 경로 쪽으로 걸어오는 사람이 PED_LOOKAHEAD 안에 있다
          · 횡단보도 위의 사람이 내 경로를 아직 PED_PASSED 만큼 지나가지 않았다
          · 건너려는 사람이 횡단보도 끝 PED_NEAR 안까지 와 있다
            (신호 있는 횡단보도는 보행 신호가 초록일 때만 — 빨강이면 새로 건너기 시작하지 않는다)
        닿을 수 있는가 — '보행자 있는 횡단보도 진입' 감점에 쓴다. 위와 같되 '지나갔다'의 기준이
            PED_CLEAR (내 차와 닿지 않는 거리) 로 더 좁다. 이미 비켜난 보행자 때문에 감점하지 않게.
        """
        c = self.map.cross[cid]
        people = traci.vehicle.getContextSubscriptionResults(self.ego_id) or {}
        wait, touch, may_start = False, False, None
        for d in people.values():
            if tc.VAR_NEXT_EDGE not in d:
                continue                             # 보행자가 아니라 차 (둘레의 차도 같은 조회 결과에 들어 있다)
            road = d[tc.VAR_ROAD_ID]
            if road == cid:                          # 건너는 중
                forward = math.cos(math.radians(d[tc.VAR_ANGLE] - c["angle"])) >= 0.0
                s = d[tc.VAR_LANEPOSITION]
                togo = (my_s - s) if forward else (s - my_s)     # 내 경로까지 남은 거리 (음수 = 지나감)
                wait = wait or -PED_PASSED <= togo <= PED_LOOKAHEAD + _PED_HALF
                touch = touch or -PED_CLEAR <= togo <= PED_LOOKAHEAD
            elif road in c["wa"] and d[tc.VAR_NEXT_EDGE] == cid:  # 건너려고 대기 공간에 들어섬
                xy = d[tc.VAR_POSITION]
                d0, d1 = math.dist(xy, c["shape"][0]), math.dist(xy, c["shape"][-1])
                if min(d0, d1) > PED_NEAR or (my_s if d0 <= d1 else c["length"] - my_s) > PED_LOOKAHEAD:
                    continue                         # 아직 횡단보도에서 멀거나, 내 경로와 먼 쪽 끝이다
                if may_start is None:
                    may_start = self._ped_may_start(c)
                if may_start:
                    wait = touch = True
            if wait and touch:
                break
        return wait, touch

    def _time_to(self, dist, v):
        """지금 속도 v 에서 최대로 가속하며 dist 를 가는 데 걸리는 시간 (s)."""
        if dist <= 0.0:
            return 0.0
        a = self.max_accel
        return (-v + math.sqrt(v * v + 2.0 * a * dist)) / a

    def _yield_foe(self, v):
        """교차로에서 내가 기다려 줘야 하는 차들 중 상충 지점에 가장 먼저 닿는 차.

        기다려 줘야 하는 차:
          · 내가 양보해야 하는 차 (합류할 본선의 차, 회전교차로 안의 차 등) 중 아직 안 지나간 차
          · 우선권과 상관없이 이미 교차로 안에 들어와 내 길을 지나는 중인 차
          · 내가 양보해야 하는 차가 방금 지나간 직후 YIELD_GAP 초 (SUMO 운전자도 그만큼 띄우고 들어간다)
        Returns: (내가 상충 지점까지 남은 거리, 그 차가 남은 거리, 그 차 속도,
                  지금 그대로 들어가면 양보 대상 앞을 가로막게 되는가) 또는 None.
        """
        now = traci.simulation.getTime()
        best, cut_in = None, False
        for f in traci.vehicle.getJunctionFoes(self.ego_id, self.visibility):
            foe, my_d, foe_d, my_out, foe_out, _, _, i_yield, _ = f
            if foe_out <= 0.0:                       # 그 차는 이미 지나갔다
                if i_yield:
                    self._foe_passed = (now, my_d)   # 방금 지나간 양보 대상
                continue
            if not i_yield and not traci.vehicle.getRoadID(foe).startswith(":"):
                continue                             # 내가 우선이고 그 차는 아직 교차로 밖 → 그 차가 기다린다
            fv = traci.vehicle.getSpeed(foe)
            t_foe = 0.0 if foe_d <= 0.0 else (foe_d / fv if fv > 0.1 else _INF)
            t_foe_out = foe_out / fv if fv > 0.1 else _INF
            # 그 차가 '내가 빠져나간 뒤 YIELD_GAP 초'보다 먼저 오고, 내가 닿기 전에 지나가지도 않는다
            if i_yield and not (t_foe > self._time_to(my_out, v) + YIELD_GAP
                                or t_foe_out < self._time_to(my_d, v)):
                cut_in = True
            if foe_d <= 0.0 and i_yield:
                self._foe_passed = (now, my_d)       # 지금 상충 지점을 지나는 중
            if best is None or t_foe < best[0]:
                best = (t_foe, my_d, foe_d, fv)
        if best is None:
            t_passed, my_d = self._foe_passed
            if 0.0 <= now - t_passed < YIELD_GAP:
                return my_d, 0.0, 0.0, False         # 방금 지나감: 아직 들어가지 않는다
            return None
        return best[1], max(best[2], 0.0), best[3], cut_in

    def _siren(self):
        """뒤에서 다가오는 사이렌 차.

        Returns: (거리 m, 내가 그 차의 차로 앞을 막고 있는가) 또는 None.
        '막고 있다'는 기준은 긴급차 보조가 일반 차를 비키게 하는 기준과 같다
        (그 차의 차로 앞 make_way_dist 안에 있음).
        """
        evs = getattr(self.assist, "evs", None)
        if not evs:
            return None
        ex, ey = traci.vehicle.getPosition(self.ego_id)
        ea = math.radians(traci.vehicle.getAngle(self.ego_id))
        live = set(traci.vehicle.getIDList())
        best = None
        for ev in list(evs):
            if ev not in live:
                continue                             # 방금 도착해서 사라진 차
            try:
                x, y = traci.vehicle.getPosition(ev)
                dist = math.hypot(ex - x, ey - y)
                if dist > max(SIREN_RANGE, self.assist.make_way_dist) + 20.0:
                    continue
                a = math.radians(traci.vehicle.getAngle(ev))
                blocking = self.ego_id in self.assist._leaders(ev)
            except traci.TraCIException:
                continue
            ahead = (ex - x) * math.sin(a) + (ey - y) * math.cos(a)     # 내가 그 차의 앞쪽에 있는가
            if blocking or (ahead > 0.0 and math.cos(a - ea) > 0.5 and dist <= SIREN_RANGE):
                if best is None or dist < best[0]:
                    best = (dist, blocking)
        return best

    # ==================================================================
    # 관측
    # ==================================================================
    def _neighbor(self, offset: int, leader: bool):
        """수업 코드의 이웃 조회에 두 가지를 더한다.

        ① 시범 주행 때 앞차 간격 보정
           SUMO 는 앞차 간격을 '범퍼 사이 거리 − 내 차의 최소 간격'으로 돌려준다. 시범 주행에서는
           최소 간격을 키워 두므로(DEMO_GAP_MARGIN), 같은 거리라도 값이 그만큼 작게 나온다.
           정책이 운전할 때와 같은 뜻(충돌 기준까지 남은 거리)이 되도록 도로 더해 준다.
        ② 바로 옆 차로(±1)에 나란히 있는 차
           수업 코드는 간격이 음수인 차를 버린다 (if gap < 0 ...: return None). 간격이 음수라는 것은 그 차가
           내 차와 앞뒤로 겹쳐 있거나(나란히) 범퍼 사이가 2.5 m 안쪽이라는 뜻이다. 속도와는 상관없다.
           그런 차는 관측에서 '차 없음'이 되어, 옆에 차가 나란히 달리는 동안 옆 차로가 빈 것으로 보인다.
           그런데 안전 검사는 그 차를 보고 차로변경을 거절하므로, 정책에게는 '옆이 비었는데 차로변경이
           안 되는' 상태가 된다. 그래서 안전 검사가 보는 옆 차(_side_gaps — 구간 경계 너머까지 찾는다)도
           함께 보고 더 가까운 쪽을 관측에 넣는다. 나란히 있는 차의 간격은 0 으로 한다.
        """
        res = super()._neighbor(offset, leader)
        if res is not None and leader and self._gap_shift and abs(offset) <= 1:
            res = (res[0] + self._gap_shift, res[1])
        if abs(offset) == 1:
            side = self._side_seen(offset, leader)
            if side is not None and (res is None or side[0] < res[0]):
                res = side
        return res

    def _side_seen(self, direction, leader):
        """관측에 넣을 바로 옆 차로의 앞차(leader=True) 또는 뒤차.

        Returns: (간격 m [0 이상. 나란히 있거나 2.5 m 안쪽이면 0], 속도 m/s). 없거나 W 밖이면 None.
        """
        try:
            lane = traci.vehicle.getLaneID(self.ego_id)
            if lane.startswith(":"):
                return None                          # 교차로 안: 수업 코드의 조회만 쓴다
            edge, idx = lane.rsplit("_", 1)
            if f"{edge}_{int(idx) + direction}" not in self.map.length:
                return None                          # 그쪽에 차로가 없다
            hit = self._side_gaps(direction)[0 if leader else 1]
        except (traci.TraCIException, ValueError):
            return None
        if hit is None or hit[0] > self.visibility:
            return None
        return max(hit[0], 0.0), hit[1]

    def _need(self, v, dist):
        """속도 v 로 달리다 dist 안에 서려면 필요한 감속 / AV 가 낼 수 있는 최대 감속 (0~1). 1 이면 이미 늦었다."""
        if v < 0.1:
            return 0.0
        return min(v * v / (2.0 * max(dist, 0.05)) / self.max_decel, 1.0)

    def _stop_need(self, s, tls_state=None, cw_wait=None):
        """서야 할 곳 앞에 멈추려면 얼마나 세게 밟아야 하는가 (0~1). 서야 할 곳이 없으면 0.

        서야 할 곳: 빨강·노랑 신호의 정지선, 기다려야 할 보행자가 있는 횡단보도, 지금 들어가면 끼어들게 되는 양보선,
                    내 경로로 이어지지 않는 차로의 끝.
        여럿이면 가장 급한 것. tls_state / cw_wait 를 주면 그 값으로 판단한다 (시범용 관측을 맞출 때).
        """
        tls, cw, foe = s["tls"], s["cw"], s["foe"]
        tls_state = tls[2] if (tls and tls_state is None) else tls_state
        cw_wait = bool(cw and cw[2]) if cw_wait is None else cw_wait
        need = 0.0
        if tls and tls_state in RED_STATES + "yY":
            need = max(need, self._need(s["v"], tls[1] - SIGNAL_STOP_GAP))
        if cw and cw_wait and cw[1] > 0.0:
            need = max(need, self._need(s["v"], cw[1] - PED_STOP_GAP))
        if foe and foe[3]:
            # 양보선 = 지금 차로의 끝. 이미 교차로 안이면 상충 지점 바로 앞.
            line = foe[0] if s["inside"] else self.map.length.get(s["lane"], s["pos"]) - s["pos"]
            need = max(need, self._need(s["v"], line - SIGNAL_STOP_GAP))
        if s["reach"] <= self.visibility:
            # 이 차로는 곧 내 경로에서 끊긴다. 그 전에 옆 차로로 옮기지 못하면 차로 끝 앞에 서야 한다.
            need = max(need, self._need(s["v"], s["reach"] - LANE_END_GAP))
        return need

    def _close_need(self, s):
        """앞차에 닿지 않으려면 얼마나 세게 밟아야 하는가 (0~1). 앞차가 지금 속도를 유지한다고 볼 때.
        앞차가 없거나 나보다 빠르면 0."""
        if s["gap"] is None:
            return 0.0
        return self._need(s["v"] - s["lead_v"], s["gap"])

    def _lane_plan(self, lane, pos, inside):
        """차로 안내. 지금 차로와 옆 차로들 가운데 '차로변경 없이 내 경로를 가장 멀리 갈 수 있는 차로'를 찾는다.

        Returns: (그 차로까지 옮겨야 하는 차로 수 [왼쪽 +, 오른쪽 −, 지금 차로가 맞으면 0],
                  지금 차로로 더 갈 수 있는 거리 m [경로 끝까지 이어지면 inf],
                  지금 차로가 이 구간 끝에서 끊기는가 [다음 구간으로 가는 연결이 없다])
        · 거리는 교차로 안에서 가는 거리까지 포함한 실제 거리다 (RoadMap.reach_table).
        · LANE_RANGE 앞까지만 비교한다. 그보다 먼 곳에서 갈리는 차로는 아직 같다고 본다.
        · 보도·버스 전용 차로, 실선을 넘어야 갈 수 있는 차로는 후보에서 뺀다.
        · 교차로 안에서는 차로를 바꿀 수 없으므로 안내는 0 이고, 거리는 이어질 차로 기준으로 잰다.
        지도(net.xml)와 경로만으로 계산한다. SUMO 의 운전 모델에는 묻지 않는다.
        """
        if inside:
            dist, cur = self.map.length.get(lane, pos) - pos, self.map._next.get(lane)
            while cur is not None and cur.startswith(":"):
                dist, cur = dist + self.map.length.get(cur, 0.0), self.map._next.get(cur)
            return 0, dist + self._reach.get(cur, _INF), False
        edge, idx = lane.rsplit("_", 1)
        idx = int(idx)
        lanes = self.map.lanes_of.get(edge, ())
        here = max(self._reach.get(lane, 0.0) - pos, 0.0)
        dead = here <= self.map.length.get(lane, _INF) - pos + 1e-6
        best_k, best_d = 0, min(here, LANE_RANGE)
        for side, step in ((0, 1), (1, -1)):                 # 왼쪽으로, 오른쪽으로 한 차로씩 가 본다
            k = idx
            while (0 <= k + step < len(lanes)
                   and self.map.change_ok.get(lanes[k], (True, True))[side]
                   and self.map.car_ok.get(lanes[k + step], False)):
                k += step
                d = min(self._reach.get(lanes[k], 0.0) - pos, LANE_RANGE)
                if d > best_d + 0.5:                         # 같으면 덜 옮기는 쪽
                    best_k, best_d = k - idx, d
        return best_k, here, dead

    def _extras(self, s):
        """_look 결과 → 관측 [31..48]."""
        w = self.visibility
        tls, cw, foe, siren = s["tls"], s["cw"], s["foe"], s["siren"]
        return [
            s["limit"] / self.vmax,
            _near(tls[1], SIGNAL_RANGE) if tls else 1.0,
            1.0 if tls and tls[2] in RED_STATES else 0.0,
            1.0 if tls and tls[2] in "yY" else 0.0,
            _near(cw[1], w) if cw else 1.0,
            1.0 if cw and cw[2] else 0.0,
            _near(foe[0], w) if foe else 1.0,
            foe[1] / w if foe else 1.0,
            foe[2] / self.vmax if foe else 0.0,
            1.0 if foe else 0.0,
            siren[0] / SIREN_RANGE if siren else 1.0,
            1.0 if siren and siren[1] else 0.0,
            1.0 if s["leader_stopped"] else 0.0,
            _near(s["gap"], w) if s["gap"] is not None else 1.0,
            self._stop_need(s),
            self._close_need(s),
            s["guide"] / 2.0,
            _near(s["reach"], LANE_RANGE),
        ]

    def _get_obs(self):
        if self.ego_id not in traci.vehicle.getIDList():
            self._snap = None
            self._last_obs = np.zeros(self.obs_dim, dtype=np.float32)
            return self._last_obs
        self._snap = self._look()
        base = super()._get_obs()                    # 수업 코드의 31차원
        extra = np.asarray(self._extras(self._snap), dtype=np.float32)
        self._last_obs = np.clip(np.concatenate([base, extra]), -1.0, 1.0)
        return self._last_obs

    def _offset_lane(self, offset):
        """지금 차로에서 offset 만큼 옆 차로의 id (교차로 안이거나 조회 전이면 None)."""
        s = self._snap
        if s is None or s["inside"]:
            return None
        edge, idx = s["lane"].rsplit("_", 1)
        return f"{edge}_{int(idx) + offset}"

    def _lane_connectivity(self, offset: int) -> float:
        """수업 코드와 같은 뜻, 기준만 대학로에 맞춤.

        -1 : 그 차로가 없거나 승용차가 못 가는 차로(보도·버스 전용)
         1 : W 이내에서 내 경로를 따라 계속 이어짐
        0~1: (그 차로로 더 갈 수 있는 거리 / W) — 곧 내 경로에서 벗어나는 차로
        """
        lane = self._offset_lane(offset)
        if lane is None:
            return 1.0
        if not self.map.car_ok.get(lane, False):
            return -1.0
        remaining = self._snap["best"].get(lane, 0.0) - self._snap["pos"]
        return float(np.clip(remaining / self.visibility, 0.0, 1.0))

    def _lane_density(self, offset: int) -> float:
        lane = self._offset_lane(offset)
        if lane is not None and lane in self.map.car_ok and not self.map.car_ok[lane]:
            return 1.0                               # 보도·버스 전용 차로 = 꽉 찬 차로 취급
        return super()._lane_density(offset)

    # ==================================================================
    # 차로변경: 행동의 차로변경 값(−1 / 0 / +1)을 실행한다. AV 의 차로가 바뀌는 길은 여기 하나뿐이다.
    # ==================================================================
    def _apply_lane_change(self, lane_change: int) -> bool:
        """Returns: 명령을 실행했는가 (0 = '유지'는 보낼 것이 없으므로 True).

        환경의 확인(갈 수 있는 차로인가, 이 구간에서 끝나는가, 안전한가)을 모두 통과한 명령만 SUMO 에 보내고,
        SUMO 는 받은 명령을 그대로 실행한다. 실제로 차로가 바뀌었는지는 step() 이 info["lane_changed"] 로 알려 준다.
        """
        if lane_change == 0:
            # 수업 코드는 '지금 차로 번호에 머물러라'를 보내는데, 다음 구간에서 차로 번호가 달라지면
            # 그 명령이 차로변경이 되어 버린다. SUMO 는 AV 의 차로를 스스로 바꾸지 않으므로 보내지 않는다.
            return True
        if self.ego_id not in traci.vehicle.getIDList():
            return False
        try:
            lane = traci.vehicle.getLaneID(self.ego_id)
            if lane.startswith(":"):
                return False                         # 교차로 안
            edge, idx = lane.rsplit("_", 1)
            if not self.map.car_ok.get(f"{edge}_{int(idx) + lane_change}", False):
                return False                         # 없는 차로, 보도, 버스 전용 차로
            if not self.map.change_ok.get(lane, (True, True))[0 if lane_change > 0 else 1]:
                return False                         # 실선: 이 차로에서 그쪽으로는 차로변경 금지
            length = self.map.length.get(lane, _INF)
            dead = self._reach.get(lane, 0.0) <= length + 1e-6
            if not dead and traci.vehicle.getLanePosition(self.ego_id) + self._move_plan >= length:
                return False                         # 이번 스텝에 교차로로 들어선다 → 이 구간에서는 끝낼 수 없다
        except (traci.TraCIException, ValueError):
            return False
        return super()._apply_lane_change(lane_change)   # 수업 코드: 안전 검사를 통과하면 traci.vehicle.changeLane

    def _lane_change_is_safe(self, direction: int) -> bool:
        """수업 코드의 안전 검사 + 같은 검사식을 '구간 경계 너머까지 찾은 옆 차로의 차'에도 적용한다.

        수업 코드의 검사가 대학로에서 옆 차를 놓치는 이유는 위 LANE_SCAN 설명 참고.
        둘 다 통과해야 안전하다고 본다 (수업 코드가 본 차는 수업 코드가, 못 본 차는 _side_clear 가 걸러 낸다).
        """
        return super()._lane_change_is_safe(direction) and self._side_clear(direction)

    def _side_gaps(self, direction):
        """옆 차로(+1 왼쪽, −1 오른쪽)에서 가장 가까운 앞차와 뒤차. 구간 경계 너머 LANE_SCAN 까지 찾는다.

        옆 차로와, 그 차로의 앞으로 이어지는 차로들·뒤에서 들어오는 차로들(교차로 안 차로 포함)에 있는 차를
        모두 모아, 옆 차로를 따라 잰 위치로 내 차와 비교한다. 차가 어느 구간에 있든 같은 방법으로 본다.
        Returns: (앞차, 뒤차) — 각각 (간격 m, 속도 m/s) 또는 None.
        간격은 '충돌 기준(범퍼 사이 2.5 m)까지 남은 거리'. 내 차와 앞뒤로 겹쳐 있으면(나란히 있으면) 음수.
        관측(_side_seen), 안전 검사(_side_clear), 시범의 틈 맞추기(_seek_speed)가 같은 결과를 쓴다.
        같은 시뮬레이션 시각에는 한 번만 찾는다.
        """
        now = traci.simulation.getTime()
        if self._side_at != now:
            self._side_at, self._side_memo = now, {}
        if direction not in self._side_memo:
            self._side_memo[direction] = self._scan_side(direction)
        return self._side_memo[direction]

    def _scan_side(self, direction):
        """_side_gaps 의 실제 계산."""
        ego = self.ego_id
        lane = traci.vehicle.getLaneID(ego)
        edge, idx = lane.rsplit("_", 1)
        target = f"{edge}_{int(idx) + direction}"
        pos = traci.vehicle.getLanePosition(ego)
        spots = {target: [0.0]}                              # 차로 → [그 차로 시작점의 위치 — 옆 차로 시작점 기준]
        for ln, off in self.map.lanes_ahead(target, LANE_SCAN).items():
            if off - pos <= LANE_SCAN:
                spots.setdefault(ln, []).append(off)
        for ln, back in self.map.lanes_behind(target, LANE_SCAN).items():
            spots.setdefault(ln, []).append(-back)
        lead = foll = None
        # 둘레의 차는 reset() 에서 등록해 둔 조회(SIDE_RANGE)로 스텝마다 한 번에 받아 온 것을 쓴다.
        around = traci.vehicle.getContextSubscriptionResults(ego) or {}
        for vid, d in around.items():
            if vid == ego or tc.VAR_LANE_ID not in d:        # 내 차 / 보행자
                continue
            for off in spots.get(d[tc.VAR_LANE_ID], ()):
                ahead = off + d[tc.VAR_LANEPOSITION] - pos                  # 그 차 앞머리가 내 앞머리보다 앞선 거리
                if not -LANE_SCAN - 20.0 <= ahead <= LANE_SCAN + 20.0:
                    continue
                front = ahead - d[tc.VAR_LENGTH]                            # 그 차 뒤끝 − 내 앞머리
                rear = -ahead - _CAR_LENGTH                                 # 내 뒤끝 − 그 차 앞머리
                if front >= 0.0:
                    if lead is None or front - 2.5 < lead[0]:
                        lead = (front - 2.5, d[tc.VAR_SPEED])
                elif rear >= 0.0:
                    if foll is None or rear - 2.5 < foll[0]:
                        foll = (rear - 2.5, d[tc.VAR_SPEED])
                else:                                                       # 앞뒤로 겹친다 (나란히 있다)
                    side = (max(front, rear) - 2.5, d[tc.VAR_SPEED])
                    if lead is None or side[0] < lead[0]:
                        lead = side
                    if foll is None or side[0] < foll[0]:
                        foll = side
        return lead, foll

    def _side_clear(self, direction):
        """_side_gaps 로 찾은 앞차·뒤차에 수업 코드와 같은 검사식을 적용한다 (기준값도 수업 코드가 읽어 둔 것).

          앞: 간격 > min_front_gap + front_tau × 내 속도,   간격 > (내 속도 − 앞차 속도)⁺ × approach_horizon
          뒤: 간격 > min_rear_gap  + rear_tau  × 뒤차 속도, 간격 > (뒤차 속도 − 내 속도)⁺ × approach_horizon
        """
        try:
            lead, foll = self._side_gaps(direction)
            v = traci.vehicle.getSpeed(self.ego_id)
        except (traci.TraCIException, ValueError):
            return False
        if lead is not None:
            gap, lv = lead[0] - self._gap_shift, lead[1]     # 시범 주행에서는 키워 둔 최소 간격만큼 더 띄운다
            if gap <= self.lc_front_min + self.lc_front_tau * v or gap <= max(v - lv, 0.0) * self.lc_horizon:
                return False
        if foll is not None:
            gap, fv = foll
            if gap <= self.lc_rear_min + self.lc_rear_tau * fv or gap <= max(fv - v, 0.0) * self.lc_horizon:
                return False
        return True

    # ==================================================================
    # 1 스텝
    # ==================================================================
    def _stop_speed(self, dist, decel):
        """dist 안에 멈추려면 이번 스텝에 낼 수 있는 가장 빠른 속도 (그 뒤로 decel 로 계속 줄인다고 볼 때)."""
        if dist <= 0.0:
            return 0.0
        h = 0.5 * decel * self.step_length
        return -h + math.sqrt(h * h + 2.0 * decel * dist)

    def _rush(self, on):
        """시범 운전자를 '급한 운전자'로 바꾸거나(on) 원래대로 돌린다."""
        traci.vehicle.setDecel(self.ego_id, self.max_decel if on else DEMO_DECEL)
        traci.vehicle.setTau(self.ego_id, DEMO_RUSH_TAU if on else DEMO_TAU)

    def _toward(self, v, target):
        """속도 v 에서 target 쪽으로 가는 행동 (가속·감속 한계 안에서)."""
        acc = (min(target, v + self.max_accel * self.step_length) - v) / self.step_length
        raw = acc / self.max_accel if acc >= 0.0 else acc / self.max_decel
        return [float(np.clip(raw, -1.0, 1.0)), 0.0]

    def _demo_command(self, s):
        """시범 주행에서 이번 스텝에 넘길 행동. Returns: (행동, 흔들기인가).

        평소   가속 페달을 끝까지 밟는다 — SUMO 가 안전한 만큼만 속도를 낸다.
        차로   차로 안내(관측 [47])가 가리키는 쪽으로 차로변경 값을 낸다 (안내가 0 이면 유지).
               옮길 때까지 스텝마다 낸다. 안전 검사를 통과하는 스텝에 실행된다 (정책이 낼 때와 똑같은 길).
               경로를 따라가는 데 필요한 차로변경만 한다 (추월은 하지 않는다).
        틈     옮겨야 하는데 옆 차로의 차 때문에 안전 검사를 통과하지 못하는 동안은, 막고 있는 차보다
               조금 느리게 간다 (_seek_speed). 그 차 뒤로 빠져서 틈이 옆으로 오면 위의 차로변경이 실행된다.
        보행자 기다려야 할 보행자가 있는 횡단보도(관측의 crosswalk_ped = 1) 앞에서는 멈춘다.
               SUMO 운전자는 보행자가 닿기 전에 먼저 지나갈 수 있으면 그냥 간다. 그대로 두면 같은 관측에서
               어떤 때는 서고 어떤 때는 가는 시범이 섞여서, 따라 하는 쪽이 어중간하게 줄이다 횡단보도에 들어간다.
               그래서 시범 운전자는 '보이면 선다'로 통일한다. 늦게 봤으면 한계까지 밟는다.
        흔들기 위 '흔들기' 설정 참고.
        """
        if s is None:
            return [1.0, 0.0], False
        sir = s["siren"]
        # 사이렌 차가 뒤에 있지만 내가 막고 있지는 않다 → 지나갈 때까지 차로를 바꾸지 않는다 (지금 차로가 곧 끊기면 예외)
        self._demo_wait = bool(DEMO_YIELD and sir and not sir[1] and s["guide"] and s["reach"] > YIELD_URGENT)
        act, disturbed = self._demo_pedal(s)
        if self._demo_wait:
            lane_cmd = 0.0
        else:
            lane_cmd = float(np.sign(s["guide"])) or float(self._yield_lane(s))    # 차로 안내가 먼저, 없으면 긴급차 양보
        self._demo_lane_cmd = lane_cmd
        return [act[0], lane_cmd], disturbed     # 차로: 안내 쪽으로 (+1 왼쪽, −1 오른쪽, 0 유지)

    def _yield_lane(self, s):
        """사이렌 차의 차로 앞을 막고 있을 때(관측 [42] = 1) 비켜 줄 옆 차로. Returns: +1 왼쪽, −1 오른쪽, 0 없음.

          · 옆 차로가 차가 다닐 수 있는 차로이고 (보도·버스 전용 차로 제외), 실선이 아니고,
          · 그 차로로 내 경로를 YIELD_KEEP 이상 더 갈 수 있을 때 (사이렌 차를 보내고 돌아올 거리가 있다)
          · 오른쪽 먼저. 지금 안전 검사를 통과하는 쪽이 있으면 그쪽.
        비킨 뒤에는 차로 안내가 원래 차로를 가리키지만, 사이렌 차가 지나갈 때까지는 돌아가지 않는다 (_demo_command).
        지도(net.xml)와 경로만으로 정한다.
        """
        sir = s["siren"]
        if not DEMO_YIELD or not sir or not sir[1] or sir[0] > YIELD_MAX_DIST or s["inside"] or s["guide"]:
            return 0
        lane, pos = s["lane"], s["pos"]
        edge, idx = lane.rsplit("_", 1)
        idx = int(idx)
        lanes = self.map.lanes_of.get(edge, ())
        cand = []
        for side, step in ((1, -1), (0, 1)):
            k = idx + step
            if not (0 <= k < len(lanes)) or not self.map.change_ok.get(lane, (True, True))[side]:
                continue
            if not self.map.car_ok.get(lanes[k], False):
                continue
            if self._reach.get(lanes[k], 0.0) - pos >= YIELD_KEEP:
                cand.append(step)
        for step in cand:
            if self._side_clear(step):
                return step
        return cand[0] if cand else 0

    def _demo_pedal(self, s):
        """_demo_command 의 가감속 부분. Returns: ([가감속, 0], 흔들기인가)."""
        v, cw = s["v"], s["cw"]
        ped_ahead = bool(cw and cw[2] and cw[1] > 0.0)
        # 횡단보도 위(차체가 다 벗어날 때까지)에서는 흔들지 않는다 — 보행자 길을 막고 서게 되므로.
        busy = s["odo"] < self._cw_busy or bool(cw and cw[1] <= 0.0)
        if busy or self._noise_left <= 0:
            if self._noise_kind == "rush":
                self._rush(False)                    # 급하게 구간 끝 → 원래 운전자로
            self._noise_kind, self._noise_left = None, 0
            u = self._rng.random() / self.demo_noise if (self.demo_noise > 0.0 and not busy) else _INF
            if u < 0.5:
                self._noise_kind = "slow"
                self._noise_left = int(self._rng.integers(DEMO_NOISE_STEPS[0], DEMO_NOISE_STEPS[1] + 1))
                self._noise_accel = float(self._rng.uniform(*DEMO_NOISE_ACCEL))
            elif u < 1.0 + (DEMO_RUSH_BRAKING if self._braking else 0.0):
                self._noise_kind = "rush"
                self._noise_left = int(self._rng.integers(DEMO_RUSH_STEPS[0], DEMO_RUSH_STEPS[1] + 1))
                self._rush(True)

        if self._noise_kind is not None:
            self._noise_left -= 1
            if self._noise_kind == "slow":
                return [self._noise_accel, 0.0], True
            if ped_ahead:                            # 급하게 가더라도 보행자 있는 횡단보도에는 들어가지 않는다
                return self._toward(v, self._stop_speed(cw[1] - 0.5, self.max_decel)), True
            return [1.0, 0.0], True

        if ped_ahead:
            # 평소 세기로 설 수 있는 속도까지 줄인다. 이미 그보다 빠르면(늦게 봤으면) 한계까지 밟는다.
            return self._toward(v, self._stop_speed(cw[1] - PED_STOP_GAP, DEMO_DECEL)), False
        seek = self._seek_speed(s) if (DEMO_SEEK and not getattr(self, "_demo_wait", False)) else None
        if seek is not None:
            return self._toward(v, seek), False      # 틈 맞추기: 막고 있는 옆 차보다 조금 느리게
        return [1.0, 0.0], False

    def _seek_speed(self, s):
        """틈 맞추기. 차로 안내가 켜져 있는데 옆 차로의 차 때문에 지금은 옮길 수 없을 때, 이번 스텝에 낼 속도.

        Returns: 속도 m/s. 틈 맞추기를 할 상황이 아니면 None (평소대로 달린다).
        안전 검사(_side_clear)와 같은 식으로 '지금 막고 있는 차'를 찾아, 그 차보다 DEMO_SEEK_SLOWER 만큼 느리게 간다.
        (막는 차가 앞뒤로 둘이면 느린 쪽에 맞춘다.) 속도는 DEMO_SEEK_DECEL 로 부드럽게 줄인다.
        막고 있는 차가 DEMO_SEEK_MIN 보다 느리면(정체 줄) 하지 않는다.
        """
        d = int(np.sign(s["guide"]))
        if not d or s["inside"]:
            return None
        try:
            lane = s["lane"]
            edge, idx = lane.rsplit("_", 1)
            if not self.map.car_ok.get(f"{edge}_{int(idx) + d}", False):
                return None                          # 갈 수 없는 차로
            if not self.map.change_ok.get(lane, (True, True))[0 if d > 0 else 1]:
                return None                          # 실선
            lead, foll = self._side_gaps(d)
        except (traci.TraCIException, ValueError):
            return None
        v, block = s["v"], None
        if lead is not None:
            gap, lv = lead[0] - self._gap_shift, lead[1]
            if gap <= self.lc_front_min + self.lc_front_tau * v or gap <= max(v - lv, 0.0) * self.lc_horizon:
                block = lv
        if foll is not None:
            gap, fv = foll
            if gap <= self.lc_rear_min + self.lc_rear_tau * fv or gap <= max(fv - v, 0.0) * self.lc_horizon:
                block = fv if block is None else min(block, fv)
        if block is None or block < DEMO_SEEK_MIN:
            return None
        return max(block - DEMO_SEEK_SLOWER, v - DEMO_SEEK_DECEL * self.step_length, 0.0)

    def step(self, action):
        before = self._snap                          # 행동을 정할 때 본 상황
        seen = self._last_obs                        # 그때의 관측
        disturbed = False
        if self.demo:
            action, disturbed = self._demo_command(before)
        a = np.asarray(action, dtype=np.float32).reshape(-1).copy()
        accel_raw, limit, planned_move = None, None, 0.0

        if a.size >= self.act_dim and before is not None:
            accel_raw = float(np.clip(a[self.accel_index], -1.0, 1.0))
            # 제한속도: 지금 있는 차로의 값. 수업 코드는 도로 전체에 하나만 쓰므로,
            # 차로 제한속도를 넘게 되는 가속만 '제한속도에 딱 맞는 가속'으로 줄여서 넘긴다.
            limit, v = before["limit"], before["v"]
            gain = self.max_accel if accel_raw >= 0.0 else self.max_decel
            target = v + accel_raw * gain * self.step_length
            if SPEED_LIMIT_CAP and target > limit:
                need = (limit - v) / self.step_length
                a[self.accel_index] = need / (self.max_accel if need >= 0.0 else self.max_decel)
                target = max(limit, v - self.max_decel * self.step_length)
            planned_move = max(target, 0.0) * self.step_length

        # 이번 스텝에 가려는 거리. 시범에서는 SUMO 가 속도를 정하므로 낼 수 있는 가장 빠른 속도로 어림한다.
        self._move_plan = planned_move
        if self.demo and before is not None:
            self._move_plan = (before["v"] + DEMO_ACCEL * self.step_length) * self.step_length
        # 경로가 이어지지 않는 차로의 끝을 넘어가려 하는가 (넘어가려는 거리)
        over = 0.0
        if not self.demo and before is not None and before["dead"]:
            over = planned_move - (self.map.length.get(before["lane"], _INF) - before["pos"])

        self.assist.step()
        obs, _, terminated, truncated, info = super().step(a)   # 수업 코드의 보상은 버리고 아래에서 다시 계산
        after = self._snap                           # 한 스텝 뒤의 상황 (AV 가 사라졌으면 None)

        # 차로변경: lane_change      정책이 낸 값 (−1 / 0 / +1)
        #           lane_change_sent 그 명령이 환경의 확인을 통과해 SUMO 에 보내졌는가
        #           lane_changed     이번 스텝에 차로가 실제로 바뀐 방향 (같은 구간 안에서 차로 번호가 달라짐)
        #           lane_change_applied  수업 코드의 뜻 그대로 '실행됐는가'. 보낸 명령이 이번 스텝에 실행됐을 때만 True
        info["lane_change_sent"] = bool(info["lane_change"]) and bool(info["lane_change_applied"])
        info["lane_changed"] = 0
        info["off_route"] = False
        if before is not None and after is not None:
            lane, now = before["lane"], after["lane"]
            if not before["inside"] and not after["inside"] and lane.rsplit("_", 1)[0] == now.rsplit("_", 1)[0]:
                info["lane_changed"] = int(now.rsplit("_", 1)[1]) - int(lane.rsplit("_", 1)[1])
            # 경로 이탈: SUMO 는 이어지지 않는 차로의 끝에서 차를 더 보내지 않는다 (세워 둔다).
            # 정책이 세운 것이 아니므로, 넘어가려 한 거리가 OFF_ROUTE_SLACK 을 넘으면 경로를 벗어난 것으로 끝낸다.
            total = self._overrun[1] if self._overrun[0] == lane else 0.0
            self._overrun = (lane, total + max(over, 0.0))
            if self._overrun[1] > OFF_ROUTE_SLACK and not terminated:
                info["off_route"] = terminated = True
        if info["lane_change"]:
            info["lane_change_applied"] = info["lane_changed"] == info["lane_change"]

        if accel_raw is not None:
            info["accel_raw"] = accel_raw            # 정책이 낸 값 그대로 기록 (줄이기 전)
        info["speed_limit"] = limit
        info["sim_time"] = float(traci.simulation.getTime())

        # ---- 이번 스텝에 생긴 일 ----
        moved = (after["odo"] - before["odo"]) if (after and before) else planned_move
        red = cross = cut_in = False
        if before is not None:
            tls, cw, foe = before["tls"], before["cw"], before["foe"]
            # 판정 기준: 행동을 정할 때 관측에 보인 신호·보행자
            tls_state = tls[2] if tls else ""
            cw_wait, cw_touch = (cw[2], cw[3]) if cw else (False, False)
            if self.demo:
                # SUMO 는 스텝 맨 앞에서 신호와 보행자를 먼저 움직이고, 그 다음에 차들이 속도를 정한다.
                # 그래서 SUMO 운전자는 관측보다 한 스텝 새로운 신호·보행자를 보고 움직였다.
                # 시범에서는 그 값으로 판정하고, 정답과 짝이 맞도록 저장용 관측에도 그 값을 넣는다.
                if tls:
                    try:
                        state = traci.trafficlight.getRedYellowGreenState(tls[0])
                        tls_state = state[tls[3]] if tls[3] < len(state) else tls_state
                    except traci.TraCIException:
                        pass
                if cw and after and after["cw"] and after["cw"][0] == cw[0]:
                    cw_touch = after["cw"][3]
                    cw_wait = cw_wait or after["cw"][2]      # 멈춤 규칙은 이전 값을, SUMO 는 새 값을 보고 움직였다
                fixed = seen.copy()
                if tls:
                    fixed[self._i_red] = 1.0 if tls_state in RED_STATES else 0.0
                    fixed[self._i_yellow] = 1.0 if tls_state in "yY" else 0.0
                if cw:
                    fixed[self._i_ped] = 1.0 if cw_wait else 0.0
                fixed[self._i_need] = self._stop_need(before, tls_state, cw_wait)
                # 시범 운전자(SUMO)는 사이렌에 반응하지 않는다. 한 시간에 두 번뿐인 긴급차 장면이 엉뚱한 버릇으로
                # 배워지지 않도록, 시범용 관측에서는 '사이렌 없음'으로 둔다. (긴급차 양보는 PPO 가 보상으로 배운다.)
                if not DEMO_YIELD:
                    fixed[self._i_siren], fixed[self._i_siren_lane] = 1.0, 0.0
                info["demo_obs"] = fixed
                # 서 있을 이유가 관측에 보이는가 (아래에서 '이유 없이 서 있던 스텝'을 가려낼 때 쓴다)
                info["demo_reason"] = bool(
                    before["gap"] is not None                                      # 앞차가 있다
                    or foe is not None                                             # 양보해야 하는 차가 있다
                    or (tls and tls_state not in "Gg" and tls[1] <= DEMO_STOP_NEAR)   # 바로 앞 신호가 초록이 아니다
                    or (cw and cw_wait and cw[1] <= DEMO_STOP_NEAR)                # 바로 앞 횡단보도에 보행자가 있다
                    or before["reach"] < self.visibility)                          # 이 차로가 곧 경로에서 끊긴다 (차로변경 대기)
            # 빨간불인데 정지선을 넘었다
            red = bool(tls and tls_state in RED_STATES and moved > tls[1] + 1e-3)
            # 보행자가 있는 횡단보도에 들어섰다
            cross = bool(cw and cw_touch and cw[1] > 0.0 and moved > cw[1])
            # 양보해야 하는 차 바로 앞에서 상충 지점에 들어섰다
            cut_in = bool(foe and foe[3] and foe[0] > 0.0 and moved > foe[0])
            if cw and moved > cw[1]:                 # 횡단보도에 들어섰다(올라서 있다): 차체가 다 벗어날 지점
                self._cw_busy = before["odo"] + cw[1] + _CROSSING_WIDTH + _CAR_LENGTH + 1.0
        info["red_light"], info["crosswalk_violation"], info["yield_violation"] = red, cross, cut_in
        if self.demo:
            info.setdefault("demo_obs", seen)
            info.setdefault("demo_reason", True)
            info["demo_disturbed"] = disturbed
            # 정답이 없는 스텝 (저장하지 않는다):
            #   · 흔드는 스텝 — SUMO 의 운전이 아니다
            #   · 이유가 관측에 보이지 않는데 SUMO 가 서 있는 스텝 — SUMO 는 관측에 없는 내부 사정(교차로 진입 시각 계산 등)
            #     으로도 기다린다. 이런 장면을 그대로 배우면 '앞이 비었는데 서 있기'를 배우게 된다
            action = None if disturbed else self._demo_action(before, after)
            self._braking = bool(action is not None and after["v"] > 2.0 and after["v"] - before["v"] <= -0.5)
            info["demo_unexplained"] = bool(action is not None and not info["demo_reason"]
                                            and before["v"] < 0.05 and after["v"] < 0.05)
            info["demo_action"] = None if info["demo_unexplained"] else action

        info["collision_with"] = None
        if info["collided"]:
            for c in traci.simulation.getCollisions():
                if self.ego_id in (c.collider, c.victim):
                    info["collision_with"] = c.victim if c.collider == self.ego_id else c.collider
                    info["collision_type"] = c.type
                    break

        # 오래 서 있음 → 시간초과와 같은 방식으로 끝낸다.
        speed = info.get("speed")
        self._still = self._still + 1 if (speed is not None and speed < 0.1) else 0
        info["stuck"] = self._still * self.step_length >= STUCK_TIME
        if info["stuck"]:
            self._fresh = False                      # 오래 서서 뒤에 정체를 만들었다 → 다음 에피소드는 새 SUMO 에서
        # 시뮬레이션이 끝나는 시각(3600 s)을 넘겨 진행하지 않는다.
        if not terminated and (info["stuck"]
                               or info["sim_time"] >= self.sim_end - self.step_length):
            truncated = True

        reward, info["reward_terms"] = self._reward(info, after)
        return obs, reward, terminated, truncated, info

    def _demo_action(self, before, after):
        """시범 주행에서 SUMO 가 한 운전을 '정책이 냈어야 할 행동' [가감속, 차로변경] 으로 바꾼다.

        가감속: 이번 스텝의 속도 변화를 그대로 만들어 내는 값.
                이번 스텝이 끝났을 때 서 있으면 DEMO_HOLD 이하로 (더 세게 밟아도 결과는 똑같이 '정지'다).
        차로변경: 시범 운전자가 이번 스텝에 낸 값 = 차로 안내 쪽 (_demo_command). 옮길 때까지 계속 낸다.
        AV 가 사라진 스텝(도착·충돌)은 속도 변화를 알 수 없어 None.
        """
        if before is None or after is None:
            return None
        v0, v1 = before["v"], after["v"]
        acc = (v1 - v0) / self.step_length
        raw = float(np.clip(acc / self.max_accel if acc >= 0.0 else acc / self.max_decel, -1.0, 1.0))
        if v1 < 0.05:
            raw = min(raw, DEMO_HOLD)                # 멈춰 섰다 / 계속 서 있다 → 브레이크를 확실히
        return [raw, float(getattr(self, "_demo_lane_cmd", np.sign(before["guide"])))]

    # ==================================================================
    # 보상
    # ==================================================================
    def _reward(self, info, after):
        """REWARD 표대로 이번 스텝의 보상을 계산한다. Returns: (보상, {항목: 값})."""
        rw, t = self.drw, {}
        if self.penalty_scale != 1.0:
            rw = {**rw, **{k: rw[k] * self.penalty_scale for k in SCALED_PENALTIES}}
        if info["collided"]:
            who = info.get("collision_with") or ""
            on_foot = info.get("collision_type") in ("crossing", "walkingarea", "sharedLane") \
                or who.startswith("ped_")
            t["crash"] = -(rw["crash_pedestrian_penalty"] if on_foot else rw["crash_penalty"])
        elif info["arrived"]:
            t["goal"] = rw["goal_bonus"]
        elif info.get("off_route"):
            t["off_route"] = -rw["off_route_penalty"]
        elif after is not None:
            speed = info["speed"] or 0.0
            t["progress"] = rw["progress_weight"] * min(speed / max(after["limit"], 0.1), 1.0)
            t["time"] = -rw["time_penalty"]
            gap = info.get("gap")                    # 앞차와의 간격 (m). 0 미만이면 SUMO 가 충돌로 기록한다
            if gap is not None:
                need = rw["tailgate_gap"] + rw["tailgate_time"] * speed
                if gap < need:
                    t["tailgate"] = -rw["tailgate_penalty"] * (1.0 - max(gap, 0.0) / need)
            if after["siren"] and after["siren"][1]:
                t["siren_block"] = -rw["siren_block_penalty"]
            cost = (rw["lane_change_cost"] if info.get("lane_changed") else
                    rw["refused_lane_change_cost"] if info["lane_change"] != 0 and not info.get("lane_change_sent")
                    else 0.0)
            if cost:
                t["lane_change"] = -cost
            if rw["lane_guide_penalty"] and after["guide"]:
                t["lane_guide"] = -rw["lane_guide_penalty"] * abs(after["guide"])
            if info["stuck"]:
                t["stuck"] = -rw["stuck_penalty"]
        if not info["collided"]:
            if info["red_light"]:
                t["red_light"] = -rw["red_light_penalty"]
            if info["crosswalk_violation"]:
                t["crosswalk"] = -rw["crosswalk_penalty"]
            if info["yield_violation"]:
                t["yield"] = -rw["yield_penalty"]
        return float(sum(t.values())), t

    # ==================================================================
    def describe(self):
        """가장 최근 관측의 대학로 항목 [31..46] 을 말로 풀어 쓴다 (화면과 맞춰 보는 용도)."""
        s = self._snap
        if s is None:
            return "(AV 없음)"
        out = [f"제한 {s['limit'] * 3.6:.0f} km/h"]
        if s["tls"]:
            color = "빨강" if s["tls"][2] in RED_STATES else "노랑" if s["tls"][2] in "yY" else "초록"
            out.append(f"신호 {s['tls'][1]:.0f} m {color}")
        if s["cw"]:
            out.append(f"횡단보도 {s['cw'][1]:.0f} m" + (" 보행자 있음" if s["cw"][2] else ""))
        if s["foe"]:
            out.append(f"양보 대상: 나 {s['foe'][0]:.0f} m / 그 차 {s['foe'][1]:.0f} m, {s['foe'][2] * 3.6:.0f} km/h")
        if s["siren"]:
            out.append(f"사이렌 {s['siren'][0]:.0f} m" + (" (내 차로)" if s["siren"][1] else ""))
        if s["leader_stopped"]:
            out.append("앞차 정차 중")
        if s["guide"]:
            out.append(f"차로 안내: {'왼쪽' if s['guide'] > 0 else '오른쪽'}으로 {abs(s['guide'])}차로")
            if not s["inside"]:
                d = 1 if s["guide"] > 0 else -1
                say = []
                for name, hit in (("앞차", self._side_seen(d, True)), ("뒤차", self._side_seen(d, False))):
                    say.append(f"{name} 없음" if hit is None else f"{name} 나란히" if hit[0] <= 0.0
                               else f"{name} {hit[0]:.0f} m")
                out.append(f"그쪽 차로: {', '.join(say)}")
        if s["reach"] <= LANE_RANGE:
            out.append(f"이 차로는 {s['reach']:.0f} m 뒤 경로에서 끊김")
        stop, close = self._stop_need(s) * self.max_decel, self._close_need(s) * self.max_decel
        if stop >= 0.05:
            out.append(f"서려면 감속 {stop:.1f} m/s²")
        if close >= 0.05:
            out.append(f"앞차 때문에 감속 {close:.1f} m/s²")
        return " | ".join(out)


# ══════════════════════════════════════════════════════════════════
#  관측 눈으로 확인하기:  python -m env.daehakro_env --gui
#
#  시범 주행(demo=True)으로 달린다: SUMO 가 신호·보행자·앞차에 맞춰 속도를 정하고, 차로는 차로 안내대로
#  옮기므로 AV 가 끝까지 간다. 1초마다 관측의 대학로 항목을 말로 출력하니,
#  sumo-gui 화면과 맞는지 비교하면 된다. (학습과는 무관한 확인용 주행이다.)
# ══════════════════════════════════════════════════════════════════
def _watch():
    import argparse
    from env.mdp_config import SIMULATION, OBSERVATION, ACTION
    from env.mdp_config import REWARD as _HIGHWAY_REWARD

    ap = argparse.ArgumentParser(description="대학로 환경의 관측을 눈으로 확인")
    ap.add_argument("--gui", action="store_true", help="sumo-gui 로 보기")
    ap.add_argument("--episodes", type=int, default=1)
    ap.add_argument("--every", type=float, default=1.0, help="몇 초마다 출력할지")
    args = ap.parse_args()

    env = DaehakroEnv(build(), None, None, SIMULATION, OBSERVATION, ACTION, _HIGHWAY_REWARD, gui=args.gui,
                      demo=True)
    stride = max(1, round(args.every / env.step_length))
    try:
        for ep in range(args.episodes):
            env.reset()
            done, k, total, events = False, 0, 0.0, {}
            while not done:
                if k % stride == 0:
                    print(f"[{k * env.step_length:6.1f}s] {env.describe()}")
                _, r, terminated, truncated, info = env.step(np.asarray([1.0, 0.0], dtype=np.float32))
                done, k, total = terminated or truncated, k + 1, total + r
                for name in ("red_light", "crosswalk_violation", "yield_violation"):
                    events[name] = events.get(name, 0) + int(info[name])
            end = ("충돌" if info["collided"] else "도착" if info["arrived"]
                   else "경로 이탈" if info["off_route"] else "오래 멈춤" if info["stuck"] else "시간초과")
            print(f"에피소드 {ep + 1}: {k} 스텝, {end}, 보상 합 {total:.1f}, "
                  f"신호위반 {events['red_light']}, 횡단보도 위반 {events['crosswalk_violation']}, "
                  f"양보 위반 {events['yield_violation']}")
    finally:
        env.close()


if __name__ == "__main__":
    _watch()
