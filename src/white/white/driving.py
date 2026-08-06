#!/usr/bin/env python3
"""
driving.py ― GPS/IMU 기반 자율주행 제어 노드
[v6.7.3 - 0703 3차 로스백: 급코너(R≈2m 필렛) 과소평가 보정 → 슬라럼 밀림 해결]

── v6.7.3 변경 (0703 3차 로스백: max CTE 0.6~0.8m 스파이크 5회 정밀 분석) ────
  · [진단] 스파이크 5회를 진입 5초 전부터 추적: 조향 포화 아님(raw>20° 비율 1.4%,
    스파이크 시점엔 여유 13~18°), 대부분 '바깥밀림'(회전방향 정규화 CTE 양수).
    실측 궤적 최소반경 1.5~2.2m(필요조향 18~26°)인데 near_demand 최대 11.4°.
  · [근본원인 — 곡률 창(3m) 과소평가] scan_curve_demand의 3m 창은 heading 변화를
    3m 구간 평균으로 재기 때문에, route_remodeler가 만드는 R≈2.0~2.5m '짧은 Bezier
    필렛'(호길이 2~3m)을 직선 접근부까지 섞어 평균 → 같은 코너를 far 스캔은 16.4°,
    near는 11.1°로 읽음(실측). 감속(demand_to_speed)과 곡률캡(LFD) 둘 다 이 값을
    쓰므로 급코너에서 속도·조향 여유가 동시에 부족 → 진입 후 바깥으로 밀림.
    보조원인: 연속 게이트 사이 far_dist가 3→9m로 잠깐 벌어지며 GATE_HOLD(7m)가
    풀려 재가속(t=29.5s 실측).
  · [수정 ①] scan_curve_demand에 Menger 곡률(±1.2m 3점 외접원) 피크 병행 측정.
    짧은 호의 진짜 최대곡률을 포착(검증: 같은 경로에서 3m창 11.9° vs 피크 16.2°).
  · [수정 ② — 역할 분리로 '정상코너 감속 방지'] 피크의 사용처를 분리:
      - 곡률캡(LFD): 피크 완전 반영 → 급코너서 LFD 확실히 단축(코너컷 기하 차단)
      - 감속: PEAK_SPEED_BLEND(0.35)만 부분 반영 → 완만·보통 코너 순항속도 보존
      - ω_n 결속(OMEGA): '지속 곡률(3m창)' 기준만 사용 → 피크의 순간 단축이
        속도를 물고 내려가는 연쇄 차단(분리 전 OMEGA 36% → 분리 후 0%)
  · [수정 ③] GATE_HOLD를 피크 기준 + 거리 7→9.5m(게이트 사이 far_dist 튐 포섭).
  · [수정 ④] 디버그 23=near피크, 26=far피크 기록(다음 로스백 검증용).
  · [검증 — 실코스 재구성 경로(ego궤적→remodeler, 487pt, 급코너 34%) 폐루프]
      OFF(v6.7.2): max 0.40 rms 0.14 반전 8~10회/분 v 1.90
      ON (v6.7.3): max 0.32 rms 0.10 반전 4회/분   v 1.74  (|CTE|>0.35 14→0틱)
    [회귀 — 완만 코스(R8~14)] Δv -0.01(불변), max 소폭 개선 → 정상코너 무영향 확인.
  · [튜닝] 급코너서 아직 밀리면: PEAK_SPEED_BLEND 0.35→0.5, GATE_HOLD_SPEED 1.6→1.4.
    급코너 감속이 과하면: PEAK_SPEED_BLEND 0.35→0.2, CURVE_WINDOW_PEAK_M 1.2→1.5.
  · [전수 감사수정 — 배포 전 재검토에서 발견·수정 4건]
    ① 피크 스캔 시작점 win(3.0)→win_pk(1.2): 기존엔 전방 0~1.8m가 피크 사각지대
       → 코너 '진입 중' apex가 1.8m 아래로 내려오면 피크 조기소실 → LFD캡이 코너
       한복판에서 풀리던 문제 차단(진입~탈출 내내 캡 유지).
    ② 제동/홀드 거리 gate_dist=min(far_dist, far_peak_dist): windowed 최대점이
       다른(완만한) 코너를 가리켜도 '가까운 급피크' 기준으로 제동거리 계산.
    ③ is_straight를 far_peak 기준으로: windowed far 5°+14m 내 급피크 상황에서
       직선 오분류(잔떨림억제·큰 데드밴드)로 턴인 지연되던 것 방지.
    ④ 디버그 23/26 배선 검증(피크 18.6° vs windowed 10.0° 정상 기록 확인) +
       검증 하네스의 좌/우 게인 부호를 실코드 규약(steer<0=R)에 정렬 후 전량 재검증.
    감사 후 재검증: 급코너 5시드 max 0.31~0.33 / 반전 2.7회/분 / v 1.70,
    완만코스 Δv -0.02(무영향). 미검토 토픽 전수 스캔: loop_dt 스톨 0, WP 점프 0,
    i_term 0.29° 이내, GPS 이벤트 0, encoder 최대공백 112ms(1회, 무해) — 클린.

── v6.7.2 변경 (0703 2차 로스백 정량검증) ──────────────────────────────
  · [검증] v6.7.1 로스백을 직접 분석: 완주(wp482/485, 도착 d0.67m). 직선 간헐감속·
    직진 트림은 실제로 해소 확인 — OMEGA 사유 82%→1.4%, 직선 66.9%가 MAX(상한),
    직선 v_target 평균 2.65m/s, 직진 CTE 편향 -0.005m(트림보정 실작동). cmd_vel_raw
    첫값 1.6896 = (clamped -0.064 +1.5)/0.85 정확히 일치 → 발행식 검증 완료.
  · [잔여문제 ① — 슬라럼 게이트간 재가속 펄스가 max CTE 0.78m의 실제원인]
    실측 t=30.5 근방: 앞 게이트 통과 직후 near_demand가 11°→2.3°로 잠깐 풀리자
    속도가 near_demand 기준이라 v_target 1.75→2.53으로 재가속. 그러나 far_demand는
    계속 14.8°(바로 뒤 다음 게이트, far_dist 7→5.8m)라 재가속한 채 진입 → CTE
    +0.13→+0.78 오버슈트. 조향포화(0.6%)·진입과속이 아니라 '게이트 사이 펄스'였음.
    BRAKE 게이팅이 '지금 가속했다 나중에 제동가능'이라 판단해 펄스를 허용했으나,
    루프지연 ~0.55s 때문에 실제론 못 따라감.
    [수정] (b1.5) 슬라럼 게이트-홀드: 급한 원거리 게이트(far_demand > GATE_HOLD_
    DEMAND_DEG)가 가까이(far_dist < GATE_HOLD_DIST_M) 있으면, 근거리 곡률이 잠깐
    풀려도 v_target을 GATE_HOLD_SPEED 이상으로 올리지 않는다(펄스 차단). 로그
    재현검증: 발동 12%, 평균속도 영향 -0.04m/s(직선 far_demand<11°이라 무영향),
    펄스 2.53→1.6으로 억제. 튜닝: 게이트서 아직 벌어지면 GATE_HOLD_SPEED를
    1.3~1.5로 ↓(추종 우선), 슬라럼이 굼뜨면 1.8로 ↑(속도 우선).
  · [잔여문제 ② — 우회전 바깥밀림 -0.105m(좌회전은 +0.008로 양호)]
    강한 우조향(raw<-6°) 320샘플 CTE평균 -0.105m = 우회전서 경로 바깥(언더스티어).
    좌회전은 -0.016으로 대칭 양호. 단일게인 0.85가 우회전을 덜 꺾는 것으로 판단.
    [수정] STEER_PLANT_GAIN을 좌/우 분리(L=0.85 유지, R=0.82로 우회전 ~3.7% 더 꺾음).
    ※주의: 이 -0.1m가 제어 언더스티어인지 회전중 안테나→뒷차축 투영 측정편향인지
      단정 불가(테스트 필요). 보수적 분리이며 되돌리려면 R을 0.85로. 다음 로스백에서
      우조향 CTE가 0 근처로 오는지 확인 후 유지/조정.
  · gps_imu.py는 변경 없음: 이번 로스백에서 융합(헤딩·CTE)이 양호해 근거 없이 센서
    파이프라인을 건드리지 않음(직진 CTE ~0, 좌회전 추종 양호가 융합 정상의 증거).

── v6.7.1 변경 (0703 로스백: 직선 간헐감속 + 슬라럼 밀림 해결) ─────────────
  · [증상] v6.7.0로 CTE는 개선(rms 0.246, max 0.574)됐으나 ①긴 직선에서 속도가
    1.2~2.2를 오가며 평균 1.65m/s로 주저앉음 ②슬라럼에서 좌회전 위주로 밀림.
  · [실측 ① — OMEGA 하강 고착] 속도사유의 82%가 OMEGA. 그중 99%가 'LFD=속도
    테이블 설계점'에서 발동(캡이 누른 게 아님). 테이블 자체가 ω_n≈0.94~0.99로
    설계돼 있어 Ω=0.95 결속이 테이블과 충돌 → v↓→table LFD↓→v_wn↓→v↓ …
    고착 평형(직선 v≈1.56/LFD≈2.65). LFD는 4.0에 0% 도달.
    [수정] 결속을 'dynamic_lfd < lfd_speed−0.05'(캡/슬루가 테이블 아래로 누르는
    불일치 구간)에서만 발동 → 직선은 테이블 설계 속도(≤2.8) 회복, 코너 진입
    킥 보호는 유지. LFD 상승슬루 0.5→0.9(코너 탈출 후 회복 단축).
  · [실측 ② — 조향 트림] 직진 명령 중 요레이트 -3.64°/s인데 정지 시 자이로
    -0.004°/s → 물리적 우측 쏠림(트림 -1.5°). 직진 유지에 +2.2° 상시 조향이
    필요했고 그걸 만들 정상상태 CTE +0.14m가 상시 잔류(좌+0.18/우-0.07 비대칭).
    [수정] 발행식 δ_pub=(δ_ctrl−TRIM)/GAIN, STEER_TRIM_DEG=-1.5(실측).
  · [실측 ③ — 게인 재검증] 우회전 실현율 1.00(보정 적중), 좌 1.07 → 0.85 유지.
    조향지연 재실측 0.55s → PREDICT_HORIZON_S 0.40→0.45(상한 0.50 엄수).
  · [부가] 예측용 자이로 바이어스 자가학습(정지 중 LPF, 보험용).
    gps_imu v5.7.1과 페어: GPS 코스 0.23s 위상정렬로 코너 phantom drift 제거.

── v6.7.0 변경 (0702 로스백 정량 분석 · 자유곡선 코스 오실레이션 해결) ──────
  · [증상] v6.6.2에서 자유곡선 코스 주행 시 진동 횟수·최대오차가 오히려 악화:
    CTE 부호반전 20.5회/분, max|CTE| 0.77m, 진동주파수 0.156Hz(=ω 0.98rad/s).
  · [실측 1 — 지연] 조향 명령→IMU 요레이트 상호상관: 지연 0.45s(corr 0.971).
    센싱(GPS 5Hz+파이프라인) ~0.1s 추가 → 총 ~0.55s. 진동주파수가 설계
    ω_n(=v√2/LFD≈1.0)과 정확히 일치 → 지연이 그 주파수의 감쇠를 소거(한계진동).
  · [실측 2 — ω_n 구멍] 자유곡선 코스에선 곡률캡이 LFD를 지배(31%)하는데 속도는
    테이블 기준 그대로 → 코너 진입마다 LFD 급감+속도 잔존으로 ω_n 1.16~1.32
    스파이크(t=11.3/19.0/26.3/30.9s), 직후 |CTE| 0.62~0.77 재가진. v6.6.2의
    'LFD∝v' 설계가 곡률캡 구간에서 깨짐 → 매 코너가 진동을 다시 때림.
  · [실측 3 — 게인] 실제 요레이트 = 기대치(v·tanδ/L)의 0.85배(서보/링키지/슬립).
    회전 중 평균 0.25m 바깥쏠림(부족조향) → CTE회복 사이클 유발.
  · [실측 4 — 변조] 엔코더 양자화(1틱=0.283m/s)로 v가 떨려 LFD 변화율 p90
    1.28m/s → PP 게인(∝1/LFD²)이 진동대역에서 변조(파라메트릭 가진).
  · [수정] ① 상태 전방예측(τ=0.40s, IMU 요레이트+엔코더로 τ초 뒤 자세 계산 후
    그 자세로 WP탐색·CTE·PP) — 지연 위상 복원, 조향 全채널 적용(기존 lead는 P만).
    ② ω_n 결속: v ≤ Ω·LFD/√2 (Ω=0.95) + 원거리 제동게이팅에도 동일 정합 →
    코너 진입 전에 이미 ω_n-정합 속도로 도착, 진입 킥 제거. ③ 발행 직전 조향
    ÷0.85(실효게인 보정). ④ 테이블 조회용 저역 속도(τ0.3s)+LFD 비대칭 슬루.
    ⑤ 위치가 뒷차축으로 투영돼 들어옴(gps_imu v5.7) → PP 기하 정확.
  · [검증] 실코드 폐루프(실측 지연0.45s·게인0.85·센싱0.1s·완만곡선 83m):
    OFF(v6.6.2 상당) max 0.64m/반전 21.8회/분(실주행 재현) → ON max 0.16m/
    반전 0회, 평균속도 -11%(1.97→1.75). 과대예측(τ0.65)은 역효과 확인 → τ는
    0.30~0.50 범위 내 유지할 것.
  · [튜닝 노브] 속도↑ 원하면 OMEGA_N_MAX 0.95→1.05(진동 여유 감소),
    트래킹 타이트하게는 0.85. 잔여 왕복 시 PREDICT_HORIZON_S 0.40→0.50.
    STEER_PLANT_GAIN은 로스백의 cmd↔요레이트 상관 기울기로 재측정.

── v6.6.2 변경 (v6.6.1 로스백, 중속·고속 오실레이션 해결) ─────────────────
  · [증상] 순항~고속에서 조향이 ±16~18°로 반복 왕복(경로요구 near_demand는 3~9°뿐),
    CTE가 0을 지나도 조향이 큰 값을 유지하다 반대로 튐. 고속 직선에서 CTE 진폭이
    0.43→0.56m로 증가(발산성). 왕복 중 CTE 스파이크로 CTE_RECOVERY 감속→평균속도↓.
  · [로스백 분석] 조향의 94%가 Pure Pursuit, PID는 6%(평균 0.62°). 즉 v6.6.1이
    도입한 지연보상(lead)은 PID 채널에만 들어가 지배 채널(PP)의 감쇠에 기여 못함.
    CTE·조향 진동주기 = 정확히 5.0초. 직선 데이터를 ω_n=v·√2/LFD 로 정리하니
    ω_n 1.16(안정)·1.19(경미)·1.26(발산) — 임계 ω_n≈1.2 rad/s 확인.
  · [근본원인] 실측 루프지연(GPS+IMU+서보+필터) ≈0.5초. 이 큰 지연 탓에 ω_n이
    임계를 넘으면 감쇠가 음(-)이 되어 한계·발산 진동. v6.6이 "짧은 LFD=타이트"라며
    고속 LFD를 5.5→3.1로 과단축 → 고속에서 LFD가 속도를 못 따라가 ω_n이 임계 위로.
    (v6.5의 고속 S자는 LFD가 아니라 '시간상수 lead' 탓이었고 v6.6.1서 이미 제거됨.
     v6.6의 LFD 단축은 없는 병을 고치려다 진짜 진동을 만든 오진이었다.)
  · [수정] LFD_TABLE을 '속도에 ~비례'하도록 재설계하여 전 속도에서 ω_n≈1.0(임계
    1.2 대비 큰 여유) 유지. 고속 3.1→4.0m. max_lfd 3.3→4.0. 곡률캡(√1.5R)은 그대로
    두어 급코너는 캡이 지배(코너 성능 불변), 완만코너·직선만 LFD가 길어져 감쇠 회복.
  · [검증] 폐루프 시뮬(지연0.5s): 현재 테이블은 고속서 발산(성장比 3.4), 신규는 감쇠
    (성장比 0.10). 원호추종: 급코너(R≤3) 캡지배로 동일, 완만코너(R4.6~12)는 신규가
    추종오차 절반↓·조향진동 1/3(짧은 LFD는 코너에서도 링잉하며 오차를 키웠음).
  · [PID] 변경 없음. PID는 전체 조향의 6%로 진동원인이 아니며 게인은 이미 보수적.
    lead 보상은 유지(P항에 소량 위상앞섬 제공, 무해).

── v6.6.1 변경 (지연보상을 물리 기하로 재정식화) ───────────────────────
  · 기존 지연보상은 임의 시간상수(LEAD_TIME)로 예측: cte + LEAD_TIME·v·sin(θe).
  · [핵심] GPS가 앞차축 정중앙 → 앞차축 CTE는 뒷차축보다 정확히 L·sin(θe) 앞섬
    (L=휠베이스 0.73m, θe=경로접선−차량heading). 이 순수 기하로 재정식화:
        cte_ctrl = cte + L·sin(θe)   (= 한 휠베이스 앞 지점의 CTE)
    → preview 거리 = L, 시간 = L/v 로 속도에 자동 적응(임의 시간상수 불필요).
    → 식에서 v가 상쇄 → 속도추정(엔코더·바퀴둘레) 오차에 무관 → 강건·부드러움.
  · 바퀴둘레(0.8482m/300틱)는 속도 v 산출용 → v는 LFD·게인 스케줄 및 곡률식
    R=L/tan(δ)에 반영. 지연보상은 순수 기하라 v와 독립(위 상쇄).
  · 튜닝은 물리단위로: MULT=1.0(정확값). 잔여왕복시 1.2~1.6, 과민시 0.8(휠베이스 배수).

── v6.6 변경 (v6.5 로스백, 직선 S자 해결) ──────────────────────────────
  · CTE가 LFD(속도)와 연동: LFD 2.3~2.4→CTE 0.08~0.11, LFD 3.4~3.6→0.28~0.32(S자).
  · 고속 LFD 대폭 단축(2.8m/s 5.5→3.1 등). 긴 LFD의 지연진동 방지 역할은 이제
    (휠베이스)지연보상이 담당 → 짧은 LFD로 타이트 추종.
  · 적분: 부호반전시 완전리셋→×0.35 소프트(한쪽 쏠림 bias 누적 유지). 고속 Ki 소폭↑.
  · 출발 스파이크: 정지~저속 조향 권한 램프(정지중 큰 조향→출발 덜컥 방지).
  · 거리 길수록 전체 평균이 순주행속도(~2.0m/s)에 근접.

────────────────────────────────────────────────────────────────────────
설계 의도
────────────────────────────────────────────────────────────────────────
[v6.0 로스백(구코드) 분석]
  - 평균 1.27m/s, 속도제한 사유 59%가 SLALOM → 슬라럼 캡이 속도를 묶음.
  - |CTE| 평균 0.36m. 부호반전 3회뿐 → 진동이 아니라 코너 추종 실패 + 한쪽 쏠림.
  - 리모델러가 차 물리 최소반경(1.90m)보다 급한 코너를 만들어 조향 포화(별도 패치함).

[v6.1 로스백(v6.0 신코드) 분석 — 이번 수정의 근거]
  - 최대 CTE 0.95m, 코너에서 0.3~0.9m. 부호반전 0회(진동 아님).
  - 원인 확정: LFD가 코너 대비 너무 길어 Pure Pursuit가 코너를 안쪽으로 자름.
    코너컷 오차 ≈ LFD²/(8R). 관측 near_demand 최대 14.7°→R≈2.78m(차 한계 내, 코너 자체는
    돌 수 있음). LFD 5m·R2.78 → 예측 CTE 1.12m(관측 0.95), LFD 3.3 → 0.49m(관측 0.46). 일치.
  - "예측하다 오차나면 급수정": 긴 LFD로 조향을 조금만 주다(코너컷), CTE가 커지자
    뒤늦게 LFD가 줄며 조향을 +5°→−19°로 확 꺾어 회복 → 사용자가 본 급수정.

[v6.1 핵심 변경]
(1) LFD: 연속식(2.22v−0.22, 최대6m) 폐지 → 속도별 테이블(LFD_TABLE) + 곡률캡.
    실제 LFD = min(속도표 LFD, √(1.44·R_ahead)).  R_ahead=L/tan(near_demand).
    코너가 급할수록 LFD를 즉시 짧게(≈코너반경) → 코너컷을 기하학적으로 차단.
    v=1.0→2.0m(급코너) / v=2.8→5.5m(직선). 모든 코너 예측 CTE ≤ 0.18m.
(2) 선행 감속 강화: 요구조향 full-slowdown 17→13°. 코너에서 속도가 더 내려가
    LFD(속도연동)가 바닥까지 짧아져 타이트 추종. 제동게이팅 조기화(decel 2.5→2.0).
(3) 급수정 완화: 회복 조향 스텝 5.0→3.5°/틱. LFD를 애초에 짧게 해 CTE가 안 커지므로
    큰 회복 자체가 드물어짐.
(4) PID 게인테이블: 속도(=LFD) 구간별 (Kp,Ki,Kd) + 선형보간. 고속일수록 P·D 강화.
(5) 잔떨림 면역: 속도는 순간 조향이 아닌 전방 곡률에만 반응 + 직선 데드밴드 확대
    + 직선 미세조향 억제. → 직선 떨림이 속도·조향을 흔들지 않음.
(6) 정리: 죽은 변수(MAX_PUBLISH_TICKS/ONE_TICK_SPEED_MS/base_lfd/gps_sbas_only/
    straight_lfd_bonus/min_kd/terrain_speed_comp_enabled) 제거. 속도 캡 7종→3종.

[유지된 안전/구조 로직]
  - GPS 점프 감지·WP 재동기화, 루프 경로 자동 절단, DR(GPS두절) 폴백,
    종점 APPROACH 감속+헤딩정렬 정지, 엔코더/ego 속도 융합, 후진(direction=-1) 지원.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray, String, Int32, Bool
from sensor_msgs.msg import Imu   # [v6.7] 지연보상 상태예측용 요레이트
import os, math, csv, time

# ── [CAM-FUSION] 카메라 융합 애드온 (없어도 GPS 단독 주행은 그대로 동작) ──
try:
    from white.camera_fusion import CameraFusion
except Exception:
    try:
        from camera_fusion import CameraFusion
    except Exception:
        CameraFusion = None


# ── 속도 제한 사유 코드 (디버그/모니터 호환 유지) ───────────────────────────
SPD_REASON_CODE = {
    "MAX":         0,   # 상한
    "CURVE":       1,   # 전방 곡률 비례 감속(선행)
    "BRAKE":       2,   # 제동거리 게이팅(코너 선행 감속)
    "CTE_RECOVERY":3,   # 횡오차 회복 감속
    "STEER":       4,   # (예비)
    "DR":          6,   # GPS 두절 DR 감속
    "RAMP":        7,   # 출발 점진가속
    "SPEED_PI":    8,   # 속도 PI 하강
    "ACCEL_LIMIT": 9,
    "DECEL_LIMIT": 10,
    "APPROACH":    11,  # 종점 접근 감속
    "OMEGA":       12,  # [v6.7] ω_n 결속: v ≤ Ω·LFD/√2 (곡률캡으로 LFD 짧아지면 속도도 함께)
    "TL_BRAKE":    20,  # [CAM] 적신호 정지선 제동곡선 v=√(2a(d−margin))
    "TL_HOLD":     21,  # [CAM] 적신호 정지 유지(정지선 margin 내)
    "TL_SLOW":     22,  # [CAM] 적신호인데 정지선 미검출 → 서행 접근
    "DR_MID":      23,  # [CAM] GPS 두절 중속 상한(차선 정상)
    "DR_LANE_LOST":24,  # [CAM] GPS 두절 + 차선 상실 → 강감속
}


# ════════════════════════════════════════════════════════════════════════════
#  PID 게인 테이블 (속도축 = LFD축, 둘은 lfd=2.22v−0.22 로 1:1 결합)
#  각 구간 (속도, Kp, Ki, Kd) — 구간 사이는 선형보간.
#  튜닝법: 한 구간에서 P를 올려 진동이 생기면 그 구간 Kd를 올려 감쇠, 초기 진동이
#  과하면 그 구간 P를 다시 내린다. 구간별로 독립 조정 가능.
#  (로스백상 진동은 거의 없고 한쪽 쏠림이 문제였으므로, P는 보수적으로 두고 Ki로
#   정상상태 오차를 천천히 제거한다.)
# ════════════════════════════════════════════════════════════════════════════
GAIN_TABLE = [
    # v(m/s)  Kp    Ki     Kd     ← 대응 LFD(m, v6.6 단축판)
    # 저속대(LFD 2.0~2.4)는 v6.5서 CTE 0.08~0.11로 이미 우수 → 게인 유지.
    # 고속대는 LFD를 짧게 했으므로 PP 게인이 자연히 커짐 → Kp 유지(과게인 방지),
    #   잔여 쏠림 제거 위해 Ki만 소폭↑. Kd는 lead가 주감쇠라 백업(코드서 ×0.20).
    (1.0,     1.7,  0.12,  0.11),  # 2.0  : 급코너
    (1.4,     1.9,  0.14,  0.14),  # 2.2
    (1.8,     2.1,  0.16,  0.17),  # 2.4  : 코너 표준
    (2.2,     2.3,  0.20,  0.21),  # 2.7  : 완만(구3.4) — Ki 0.18→0.20
    (2.5,     2.5,  0.23,  0.25),  # 2.9  : 고속(구4.4) — Ki 0.20→0.23
    (2.8,     2.6,  0.25,  0.30),  # 3.1  : 최고속(구5.5)— Ki 0.22→0.25(쏠림 타이트)
]

# ════════════════════════════════════════════════════════════════════════════
#  LFD 게인 테이블 (속도 구간별 전방주시거리) — 구간 사이 선형보간
#  [v6.1] 연속식(2.22v−0.22, 최대6m)이 코너를 잘라(코너컷 CTE=LFD²/8R) 폐지.
#  [v6.2] 직선(고속) LFD를 4.2→5.5m로 상향(고속 직진 평활·안정). 곡선은 곡률캡이
#         따로 보호하므로 직선 LFD를 늘려도 코너컷 없음(min(속도표, √(1.44·R))).
#    실제 적용 LFD = min( 속도표 LFD, 곡률캡 √(1.44·R_ahead) ).  R_ahead=L/tan(near_demand).
#  튜닝법: 특정 속도 구간에서 코너 CTE가 크면 그 행 LFD를 줄인다(단 너무 줄이면 진동).
#          직선 잔떨림/뻣뻣함이 크면 고속행 LFD를 키운다(평활↑).
# ════════════════════════════════════════════════════════════════════════════
LFD_TABLE = [
    # v(m/s)  LFD(m)
    # [v6.6.2] 오실레이션 근본수정. 설계기준: PP 고유진동수 ω_n=v·√2/LFD 를 전 속도에서
    #   ω_n≈1.0 rad/s로 유지(로스백 실측 임계 ω_n≈1.2 대비 큰 감쇠여유). 즉 LFD를
    #   속도에 대략 비례(LFD≈1.4·v)시켜, 고속에서도 ω_n이 임계 위로 못 올라가게 한다.
    #   근거: 루프지연 ≈0.5초가 커서, ω_n이 임계를 넘으면 유효감쇠가 음(-)→발산 진동.
    #   v6.6의 과단축(고속 3.1)은 고속 ω_n을 1.26까지 밀어올려 발산의 원인이었다.
    #   곡률캡 min(표, √1.5R)은 유지 → 급코너는 캡이 지배(코너 성능 불변), 완만코너·
    #   직선만 LFD가 길어져 감쇠 회복. 튜닝: 고속서 아직 왕복이면 고속행을 더 키우고
    #   (ω_n↓), 코너진입이 굼뜨면 곡률캡K(1.5)를 살짝 올린다.
    (1.0,     2.3),   # ω_n 0.61 (급코너는 곡률캡√1.5R≈2.0~2.1이 지배)
    (1.4,     2.5),   # ω_n 0.79
    (1.8,     2.8),   # ω_n 0.91 (2.4→2.8)
    (2.2,     3.2),   # ω_n 0.97 (2.7→3.2)
    (2.5,     3.6),   # ω_n 0.98 (2.9→3.6)
    (2.8,     4.0),   # ω_n 0.99 (3.1→4.0) 고속 직선 발산 해소 핵심
]


class DrivingNode(Node):

    # ── 엔코더/속도 환산 상수 ──────────────────────────────────────────────
    WHEEL_CIRCUMFERENCE   = 0.27 * math.pi   # 0.8482 m (바퀴 둘레)
    TICKS_PER_REV         = 300.0
    DT_ENC                = 0.01             # 아두이노 속도 PID 주기 10ms
    ENCODER_SPEED_LPF_TAU = 0.05

    # ── 루프 경로 자동 절단 ────────────────────────────────────────────────
    LOOP_DETECT_THRESHOLD = 3.0
    LOOP_FAR_DIST         = 5.0
    LOOP_RETURN_DIST      = 1.5

    def ticks_to_speed_ms(self, ticks: float) -> float:
        return ticks * self.WHEEL_CIRCUMFERENCE / self.TICKS_PER_REV / self.DT_ENC

    def speed_ms_to_ticks_float(self, speed_ms: float) -> float:
        return speed_ms * self.DT_ENC * self.TICKS_PER_REV / self.WHEEL_CIRCUMFERENCE

    def __init__(self):
        super().__init__("driving_node")
        self.data_dir = os.path.expanduser("~/white_ws/gps_data")

        # ── 차량/속도 파라미터 ────────────────────────────────────────────
        self.vehicle_length = 0.73                 # 휠베이스 [m]
        self.STEER_MAX_DEG  = 20.0                 # ROS측 조향 클램프(아두이노는 21°)
        self.STEER_AUTHORITY_FULL_SPEED = 0.5      # [v6.6] 이 속도[m/s]↑서 조향 full 권한(저속 램프)
        # 물리 최소 회전반경 = L/tan(steer_max) = 0.73/tan(20°) ≈ 2.01m
        self.max_speed_ms   = 2.8                  # PWM205 상한 = 실측 약 2.8m/s
        self.min_speed_ms   = 1.0                  # 주행 중 최저 속도
        self.safe_stop_speed = 0.08                # 정지로 간주하는 미세속도

        # 출발 점진 가속
        self.RAMP_UP_DURATION = 3.0
        self.ramp_up_active   = False
        self.drive_start_time = 0.0
        self.dr_speed_factor  = 0.6                # GPS 두절(DR) 시 감속 계수

        # ── LFD (전방주시거리) — 속도 테이블(LFD_TABLE) + 곡률캡 ─────────────
        # [v6.1] 연속식(2.22v−0.22, 최대6m)이 코너를 잘라(코너컷 CTE=LFD²/8R) 폐지.
        #   실제 LFD = min( 속도표 LFD, 곡률캡 ),  곡률캡 = √(LFD_CURVE_CAP_K·R_ahead)
        #   R_ahead = L/tan(near_demand). 코너가 급할수록 LFD를 직접 짧게 → 컷 차단.
        self.min_lfd   = 2.3                       # [v6.4] 2.0→2.3 (진동유발 영역 탈출)
        self.max_lfd   = 4.0                       # [v6.6.2] 3.3→4.0 (고속 직선서 신규 LFD 4.0 적용 허용; 곡률캡이 코너를 계속 보호)
        self.LFD_CURVE_CAP_K = 1.5                 # [v6.6] 1.6→1.5 (코너 살짝 더 타이트) — v6.6.2 유지(코너 성능 불변)
        self._lfd_lpf  = self.min_lfd
        self.LFD_LPF_ALPHA = 0.40                  # LFD 급변 완화(시상수 ≈95ms@20Hz)
        self._near_demand_lpf = 0.0                # [v6.4] 요구조향 평활(속도·LFD 맥동→결합진동 억제)
        self._near_peak_lpf   = 0.0                # [v6.7.3] 급코너 피크(곡률캡 전용) 평활
        self.NEAR_DEMAND_LPF_ALPHA = 0.30

        # ── 선행 가감속(예측 제어) 파라미터 ───────────────────────────────
        # 전방 경로 곡률 → 요구 조향각(δ=atan(L·κ)) → 비례 감속.
        # [v6.4] full slowdown 13→15°: 완만한 커브에서 과감속(→저속→맥동)을 줄여 평균속도↑.
        #        급코너는 곡률캡(LFD 축소)+min_speed 하한이 계속 보호하므로 안전 유지.
        self.STEER_FULL_SLOWDOWN_DEG = 15.0        # 요구조향 15°↑ → min_speed
        self.CURVE_PREVIEW_NEAR_K    = 1.6         # 근거리 스캔 = LFD·1.6 (현재 코너 속도)
        self.CURVE_PREVIEW_FAR_MAX   = 14.0        # 원거리 스캔 상한[m] (코너 조기 인지)
        self.CURVE_WINDOW_M          = 3.0         # 곡률 계산 윈도우[m] (평활된 주 신호)
        # [v6.7.3] 급코너 피크 포착용 짧은 창. 3m창은 R≈2m·호길이 짧은 Bezier 필렛을
        #   직선접근까지 평균해 요구조향을 ~11°로 과소평가(실측: far 16.4° vs near 11.1°)
        #   → 감속·곡률캡 부족 → 코너 바깥밀림(실측 max CTE 0.6~0.8m 5회)의 근본원인.
        #   짧은 창(1.2m)으로 진짜 최대곡률을 함께 재 max로 합성. 노이즈는 주 신호(3m창)가
        #   담당하므로 피크는 감속·LFD캡에만 영향, 조향 채널 안정성은 불변.
        #   튜닝: 급코너서 아직 밀리면 1.0으로 ↓(더 민감), 과잉감속이면 1.5로 ↑.
        self.CURVE_WINDOW_PEAK_M     = 1.2
        # [v6.7.3] 급코너 피크를 '속도'에 반영하는 비율(곡률캡=LFD엔 항상 완전 반영).
        #   곡률캡이 코너컷을 이미 막으므로 속도는 부분만 낮춰 순항속도 보존.
        #   0.35: 급코너 피크 5° 초과분의 35%만 감속에 반영(정상코너 속도 거의 불변,
        #   급코너만 소폭 추가 감속). 급코너서 아직 밀리면 0.5로 ↑, 과잉감속이면 0.2로 ↓.
        self.PEAK_SPEED_BLEND        = 0.35
        self.BRAKE_GATE_DECEL        = 2.0         # 제동거리 가정 감속도[m/s²] (조기 감속 위해 보수적)
        self.BRAKE_GATE_MARGIN       = 1.5         # 코너 1.5m 전엔 목표속도 도달(조기)

        # ── [v6.7.2] 슬라럼 게이트-홀드(게이트간 재가속 펄스 억제) ──────────
        # 실측(0703 2차): 두 급게이트 사이 near_demand가 잠깐 풀리면 v_target이
        # 재가속(1.75→2.53)했다가 다음 게이트서 오버슈트(CTE +0.78m). 급한 원거리
        # 게이트가 가까이 있으면 근거리가 풀려도 속도를 올리지 않는다.
        # BRAKE와 독립(BRAKE는 '제동거리상 가능'이면 펄스를 허용하지만, 여기선
        # '가까운 급게이트 앞에선 재가속 금지' — 지연 때문에 펄스가 오버슈트로 직결).
        self.GATE_HOLD_DEMAND_DEG = 12.0   # 이 이상 far_demand면 급게이트로 간주
        self.GATE_HOLD_DIST_M     = 9.5    # 이 이내 far_dist면 홀드 발동 [v6.7.3] 7→9.5:
                                           #   연속 게이트 사이 far_dist가 잠깐 9m로 벌어져도 유지
        self.GATE_HOLD_SPEED      = 1.6    # 홀드 시 v_target 상한[m/s] (추종↔속도 균형)
        #   튜닝: 게이트서 아직 벌어지면 1.3~1.5로↓(추종우선), 슬라럼 굼뜨면 1.8로↑

        # ── 속도 변화율 제한(슬루) ────────────────────────────────────────
        self.MAX_ACCEL_PER_S = 1.7                 # 가속은 부드럽게(코너 탈출 시 서서히 빌드)
        self.MAX_DECEL_PER_S = 3.5                 # 감속은 빠르게(코너 진입 제때)
        self.CORNER_DECEL_PER_S = 4.5              # 코너 직전 강한 감속 허용

        # ── CTE 회복 감속(안전) ───────────────────────────────────────────
        # [v6.1] LFD를 짧게 만들어 CTE가 애초에 안 커지므로, 회복캡은 진짜 큰 오차에서만.
        # [v6.4] 0.30→0.38: 0.3m 잔여 쏠림이 CTErec를 발동시켜 불필요 감속하던 것 방지(속도↑).
        self.CTE_RECOVER_START = 0.38              # 이 이상 CTE면 속도 캡 시작
        #   <0.38: 제한없음 / 0.48→2.2 / 0.62→1.7 / 0.80↑→1.2
        self.CTE_RECOVER_CAPS = [(0.48, 2.2), (0.62, 1.7), (0.80, 1.2)]

        # ── [v6.7] ① 루프 지연 보상: 상태 전방 예측 ──────────────────────
        # 로스백 실측(0702 주행): 조향 명령→실제 요레이트 지연 0.45s(상관 0.971),
        # GPS 5Hz+파이프라인 센싱 지연 ~0.1s. 총 ~0.55s 순수지연이 ω_n≈1.0에서
        # 감쇠를 소거 → 한계진동(측정 진동주파수 0.98rad/s = 설계 ω_n과 일치).
        # 대책: τ초 뒤 자세(위치+헤딩)를 IMU 요레이트·엔코더 속도로 예측하고
        # '그 자세'로 WP탐색·CTE·PP를 계산 → 잃어버린 위상을 조향 전 채널에서 복원.
        # 폐루프 시뮬 검증: 과대예측(0.65s)은 역효과·불안정, 0.35~0.50이 안전역
        # → 기본 0.40s(보수). 왕복이 남으면 0.50까지 ↑, 예민해지면 0.30으로 ↓.
        self.PREDICT_HORIZON_S   = 0.45    # [s] 0=예측 끔(비권장) [v6.7.1] 재실측 조향지연
                                           #   0.55s(0703) 반영 0.40→0.45. 상한 0.50 엄수.
        self.PREDICT_MAX_YAW_RAD = 0.60    # 예측 헤딩 전진 상한(±34°) — 스핀 시 폭주 방지
        self.imu_gyro_z    = 0.0           # [rad/s] /imu/data 요레이트(부호: CCW+)
        self.imu_last_time = None
        self.ctrl_lat = None               # 예측 자세(제어 전용) — 도착판정은 원시좌표 사용
        self.ctrl_lon = None

        # ── [v6.7] ② ω_n 결속 속도상한: v ≤ Ω·LFD/√2 ─────────────────────
        # 실측: 자유곡선 코스에서 곡률캡이 LFD를 지배(31%)하는데 속도는 테이블 기준
        # 그대로 → 코너 진입마다 ω_n이 1.16~1.32로 스파이크(t=11.3/19.0/26.3/30.9s)
        # → 직후 |CTE| 0.62~0.77m 재가진. v6.6.2의 'LFD∝v' 설계가 곡률캡 구간에서
        # 깨지는 구멍을 봉합: LFD가 어떤 이유로든 짧아지면 속도가 자동으로 따라온다.
        self.OMEGA_N_MAX        = 0.95     # 예측 활성 시. 트래킹 우선이면 0.85(평균속도↓)
        self.OMEGA_N_MAX_NO_IMU = 0.80     # IMU 미수신(예측 저하) 시 자동 보수화

        # ── [v6.7] ③ 조향 실효게인 보정 ──────────────────────────────────
        # 실측: 실제 요레이트 = 기대치(v·tanδ/L)의 0.85배 (서보 스케일/링키지/슬립).
        # → 코너에서 15% 부족조향 → 회전 중 평균 0.25m 바깥쏠림 → CTE회복 사이클 유발.
        # 발행 직전에만 ÷0.85 (제어기 내부 수치는 그대로 → 튜닝값 의미 보존).
        # 재측정법: 로스백에서 cmd 조향↔IMU 요레이트 상호상관 기울기.
        # [v6.7.1 재실측 0703] 우회전 게인 0.847(보정 후 실현율 1.00 ✓),
        #   좌회전 0.911(7% 과실현, 허용범위) → 0.85 유지.
        self.STEER_PLANT_GAIN = 0.85       # (하위호환/기본값) 좌우 분리는 아래 L/R 사용

        # ── [v6.7.2] ③-1b 조향 실효게인 좌/우 분리 ───────────────────────
        # 실측(0703 2차): 강한 우조향(raw<-6°) 구간 CTE 평균 -0.105m(바깥 언더스티어),
        # 좌조향은 -0.016m(양호). 우회전이 덜 꺾이는 것으로 판단 → 우회전 게인만 소폭↓
        # (÷게인이므로 게인↓ = 발행 조향↑). 좌는 양호하므로 0.85 유지.
        # 되돌리기: STEER_PLANT_GAIN_R을 0.85로 두면 v6.7.1과 동일(단일 게인).
        # 재측정: 다음 로스백에서 좌/우 조향별 CTE 평균이 둘 다 0 근처면 성공.
        self.STEER_PLANT_GAIN_L = 0.85     # 좌회전(δ_ctrl>0) 실효게인 (실측 0.911)
        self.STEER_PLANT_GAIN_R = 0.82     # 우회전(δ_ctrl<0) 실효게인 (실측 0.847) — 바깥밀림 보정

        # ── [v6.7.1] ③-2 조향 트림(직진 쏠림) 보정 ───────────────────────
        # 0703 로스백 실측: 직진 명령(|δ_pub|<1°) 중 요레이트 -3.64°/s, 그런데 정지 시
        # 자이로는 -0.004°/s → 센서 바이어스가 아니라 '물리적 우측 쏠림'. 조향각 환산 -1.5°.
        # 이 트림 탓에 직진 유지에 +2.2°(제어량) 상시 조향이 필요했고, 그 조향을 만들
        # 정상상태 CTE +0.14m가 직선에 상시 잔류(회전 밀림 좌+0.18/우-0.07 비대칭의 주범).
        # 보정: 발행 직전 δ_pub = (δ_ctrl − TRIM)/GAIN.
        # 재측정법: 직진 로스백에서 |δ_pub|<1° 구간 gz평균 × L/v = 물리 트림각(부호 그대로).
        self.STEER_TRIM_DEG = -1.5

        # ── [v6.7.1] ③-3 예측용 자이로 바이어스 자가학습 ─────────────────
        # 예측이 raw gz를 쓰므로 IMU 바이어스가 있으면 헤딩예측이 상시 기울어짐.
        # 정지(|v|<0.05) 중 gz를 느린 LPF로 학습해 예측에서 차감(이번 IMU는 ~0, 보험용).
        self.gyro_bias_z = 0.0             # [rad/s]

        # ── [v6.7] ④ 스케줄링 안정화(게인 변조 억제) ─────────────────────
        # 엔코더 양자화(1틱=0.283m/s)로 v_now가 떨려 LFD가 p90 1.28m/s로 출렁
        # → PP 게인(∝1/LFD²)이 진동주파수 대역에서 변조(파라메트릭 가진).
        # 테이블 조회 전용 저역 속도 + LFD 비대칭 슬루(감소는 빠르게=코너 보호 유지).
        self.V_SCHED_TAU   = 0.30          # [s] 테이블 조회용 속도 LPF
        self.v_sched       = 0.0
        self.LFD_RATE_UP   = 0.9           # [m/s] LFD 증가 상한 [v6.7.1] 0.5→0.9: 코너 탈출 후
                                           #   회복이 느려 직선 초입까지 저LFD·저속이 끌리던 것 단축
        self.LFD_RATE_DOWN = 2.0           # [m/s] LFD 감소 상한(코너 보호는 즉각)
        self._lfd_out      = self.min_lfd

        # ── PID(조향) 상태 ────────────────────────────────────────────────
        self.Ki_CLAMP        = 6.0                 # [v6.3] 1.0→6.0: 적분이 실제 쏠림 보정 가능하게
                                                   #   (기존 상한 1.0+Ki 0.05 → 최대 i항 0.05°로 사실상 무력했음)
        self.CTE_DEADBAND          = 0.02          # 코너 데드밴드[m]
        self.CTE_DEADBAND_STRAIGHT = 0.06          # 직선 데드밴드[m] (잔떨림 무시)
        self.CTE_LPF_ALPHA   = 0.80
        self.D_TERM_LPF_ALPHA = 0.16
        # ── [v6.6.1] 지연 보상 — '시간상수' 폐지, '휠베이스 기하'로 정확 계산 ──────
        # 원리: GPS가 앞차축 정중앙 → 앞차축 CTE는 뒷차축 CTE보다 정확히 L·sin(θe) 앞섬
        #   (θe=경로접선−차량heading, L=휠베이스 0.73m). 이 기하로 '한 휠베이스 앞'의
        #   CTE를 예측: cte_ctrl = cte + L·sin(θe).
        #   → preview 거리 = L, preview 시간 = L/v (속도에 자동 적응, 임의 시간상수 불필요).
        #   → 식에서 속도 v가 상쇄 → 속도추정(엔코더·바퀴둘레) 오차에 무관 → 강건·부드러움.
        #   (바퀴둘레 0.8482m/300틱은 속도 v 산출에 쓰이며, v는 LFD·게인 스케줄과
        #    곡률식 R=L/tan(δ)에 반영됨. 지연보상은 위처럼 순수 기하라 v와 독립.)
        self.LEAD_WHEELBASE_MULT = 0.0             # [v6.7] 상태예측(PREDICT_HORIZON_S)이 지연보상을 전 채널에서
                                                   #   대체 → P채널 이중보상 방지 위해 0. (예측을 끄면 1.0 권장)
        self.LEAD_WHEELBASE = self.LEAD_WHEELBASE_MULT * self.vehicle_length   # [m]
        self.LEAD_CLAMP = 0.45                     # lead 기여 상한[m] (안전)
        self.LEAD_RATE_LPF_ALPHA = 0.40            # heading오차 기반 lead 평활(세그먼트 전환 노이즈↓)
        self._cte_rate_lpf = 0.0
        # 주의: 잔여 지연이 남아 왕복이 있으면 MULT를 1.2~1.6(=1.2~1.6 휠베이스 preview)로,
        #       과보상으로 예민하면 0.8로. (여전히 물리단위=휠베이스 배수, 임의 시간 아님)
        self.PID_STEER_LIMIT = 9.0                 # PID 단독 조향 상한(PP 보조)
        self.cte_integral = 0.0
        self.prev_cte     = 0.0
        self.cte_lpf      = 0.0
        self.cte_lpf_ready = False
        self.prev_cte_seg_idx = 0
        self.d_term_lpf   = 0.0
        self.integral_decay_interval = 1.0
        self.last_integral_decay = time.time()

        # ── 직선 미세조향(잔떨림) 억제 ────────────────────────────────────
        self.JITTER_DEMAND_DEG = 6.0               # 전방요구조향 이 이하 = 직선으로 간주
        self.JITTER_CTE_M      = 0.08              # 이 이하 CTE면
        self.JITTER_STEER_DEG  = 3.0               # 이 이하 조향명령은
        self.JITTER_ZERO_DEG   = 0.7               # 이 이하면 0으로, 아니면 수축

        # ── 속도 PI(목표속도 미세보정) ────────────────────────────────────
        self.spd_kp = 0.08
        self.spd_ki = 0.0005
        self.spd_integral = 0.0
        self.MAX_SPD_INTEG = 0.08

        # ── 조향 슬루레이트 ───────────────────────────────────────────────
        self.STEER_SMOOTH_ALPHA = 0.5
        self.STEER_STEP_LOW_DEG  = 2.5             # 저속(코너) 빠른 응답 허용
        self.STEER_STEP_HIGH_DEG = 3.2             # 고속 급변 완화
        self.STEER_STEP_RECOVER_DEG = 3.5          # [v6.1] 큰 CTE 회복 허용(5.0→3.5, 급수정 완화)
        self.prev_steer = 0.0

        # ── 종점 정지/접근(APPROACH) ──────────────────────────────────────
        self.APPROACH_START_DIST = 9.0             # [v6.4] 14→9: 접근 크롤 구간 단축(전체 평균속도↑)
        self.APPROACH_END_SPEED  = 0.10
        self.FINAL_APPROACH_WP_WINDOW = 90
        self.FINAL_APPROACH_ENTRY_SPEED = 2.6      # [v6.4] 2.4→2.6 접근 진입속도(덜 굼뜨게)
        self.FINAL_HEADING_ALIGN_DIST = 2.0        # [v6.3] 8.0→2.0: 종점 2m 전까지 실제 경로 추종
                                                   #   (8m부터 직선타깃을 쫓아 곡선구간에서 CTE 1.2m 폭발하던 문제 수정)
        self.FINAL_HEADING_TOL_DEG = 22.0
        self.FINAL_CREEP_DIST  = 2.0               # 종점 2m 이내 크리프
        self.FINAL_CREEP_SPEED = 0.85
        self.FINAL_STOP_SPEED  = 0.45
        self.FINAL_FORCE_STOP_DIST = 0.30
        self.STOP_CONFIRM_DIST  = 0.80
        self.STOP_CONFIRM_SPEED = 0.55
        self.HARD_ARRIVAL_DIST  = 0.55
        self.FINAL_ARRIVAL_DIST = 0.70
        # APPROACH LFD 캡(붕괴 방지): cap=d2dest·RATIO+OFFSET, 단 MIN 아래로 안 줄임
        self.APPROACH_LFD_MIN    = 2.20
        self.APPROACH_LFD_RATIO  = 0.45
        self.APPROACH_LFD_OFFSET = 1.30
        self.approach_reset_done = False
        # 종점 지나침 감지
        self.PASS_DETECT_MIN_DIST = 3.00
        self.PASS_HYSTERESIS      = 0.10
        self.min_d2dest_seen      = float('inf')

        # ── 상태 변수 ─────────────────────────────────────────────────────
        self.active        = False
        self.current_lat   = None
        self.current_lon   = None
        self.heading       = 0.0
        self.current_steer = 0.0
        self.current_speed_ms = 0.0
        self.ego_speed_ms  = 0.0
        self.current_encoder_ticks_lpf = 0.0
        self.encoder_last_time = None

        self.prev_lat_for_jump = None
        self.prev_lon_for_jump = None
        self.gps_jump_recover_cycles = 0

        self.terrain_code = 0.0
        self.imu_pitch    = 0.0
        self.dr_active    = False
        self.gps_blend_on = False
        self.prev_gps_blend_on = False
        self.last_gps_blend_event_time = 0.0

        self.waypoints = []
        self.wp_idx    = 0

        self.target_speed_lpf  = self.min_speed_ms
        self.last_published_speed = 0.0
        self.last_control_time = None

        # WP 탐색
        self.WP_SEARCH_WINDOW      = 45
        self.START_ALIGN_DURATION  = 2.0
        self.MAX_WP_ADVANCE_START  = 35
        self.MAX_WP_ADVANCE_NORMAL = 5

        # 상태 토픽 발행 타이밍
        self.last_status_publish = 0.0
        self.STATUS_PUBLISH_INTERVAL = 1.0

        # ── 구독 / 발행 ───────────────────────────────────────────────────
        self.create_subscription(Float64MultiArray, "/ego_state",     self.cb_ego,        10)
        self.create_subscription(String,            "/drive_cmd",     self.cb_drive_cmd,  10)
        self.create_subscription(Int32,             "/encoder",       self.cb_encoder,    10)
        self.create_subscription(Imu,               "/imu/data",      self.cb_imu,        20)  # [v6.7] 예측용 요레이트
        self.create_subscription(Float64MultiArray, "/terrain_state", self.cb_terrain,    10)  # [v6.3.1] 원본과 동일하게 복원
        self.create_subscription(String,            "/gps_status",    self.cb_gps_status, 10)

        self.drive_pub  = self.create_publisher(Twist,             "/cmd_vel_raw",   10)
        self.state_pub  = self.create_publisher(Bool,              "/control_state", 10)
        self.status_pub = self.create_publisher(String,            "/drive_status",  10)
        self.event_pub  = self.create_publisher(String,            "/drive_event",   10)
        self.debug_pub  = self.create_publisher(Float64MultiArray, "/driving_debug", 10)

        # ── [CAM-FUSION] 카메라 융합 애드온 장착 (실패해도 GPS 단독 주행 유지) ──
        self.camfu = None
        if CameraFusion is not None:
            try:
                self.camfu = CameraFusion(self)
            except Exception as e:
                self.get_logger().error(f"⚠️ CameraFusion 초기화 실패(GPS 단독으로 진행): {e}")

        self.create_timer(0.05, self.control_loop)   # 20Hz

        self.get_logger().info(
            f"🚀 자율주행 v6.7.3 [급코너피크 Menger±{self.CURVE_WINDOW_PEAK_M}m·blend{self.PEAK_SPEED_BLEND} "
            f"+ 지연예측 τ={self.PREDICT_HORIZON_S}s + ω_n결속 Ω={self.OMEGA_N_MAX} "
            f"+ 조향게인 ÷L{self.STEER_PLANT_GAIN_L}/R{self.STEER_PLANT_GAIN_R} "
            f"+ 게이트홀드 {self.GATE_HOLD_SPEED}m/s@{self.GATE_HOLD_DIST_M}m] | "
            f"v={self.min_speed_ms}~{self.max_speed_ms}m/s "
            f"LFD=table[{self.min_lfd}~{self.max_lfd}]∩곡률캡√{self.LFD_CURVE_CAP_K}R | "
            f"선행감속 full@{self.STEER_FULL_SLOWDOWN_DEG}° 제동게이팅 decel={self.BRAKE_GATE_DECEL}")

    # ══════════════════════════════════════════════════════════════════════
    #  이벤트 헬퍼
    # ══════════════════════════════════════════════════════════════════════
    def publish_event(self, msg: str, level: str = "info"):
        self.event_pub.publish(String(data=msg))
        if level == "warn":
            self.get_logger().warning(msg)
        elif level == "error":
            self.get_logger().error(msg)
        else:
            self.get_logger().info(msg)

    # ══════════════════════════════════════════════════════════════════════
    #  콜백
    # ══════════════════════════════════════════════════════════════════════
    def cb_encoder(self, msg: Int32):
        raw = float(msg.data)
        speed_raw = self.ticks_to_speed_ms(raw)
        now = time.time()
        if self.encoder_last_time is None:
            self.current_speed_ms = speed_raw
        else:
            dt = max(0.001, min(0.1, now - self.encoder_last_time))
            alpha = dt / (self.ENCODER_SPEED_LPF_TAU + dt)
            self.current_speed_ms += alpha * (speed_raw - self.current_speed_ms)
            if abs(raw) < 0.5 and abs(self.current_speed_ms) < self.min_speed_ms * 0.5:
                self.current_speed_ms = 0.0
        self.current_encoder_ticks_lpf = self.speed_ms_to_ticks_float(self.current_speed_ms)
        self.encoder_last_time = now

    def cb_imu(self, msg: Imu):
        # [v6.7] 지연보상 예측용 요레이트(rad/s, CCW+). 가벼운 LPF로 잡음만 억제.
        gz = float(msg.angular_velocity.z)
        self.imu_gyro_z += 0.5 * (gz - self.imu_gyro_z)
        # [v6.7.1] 정지 중 자이로 바이어스 자가학습(느린 LPF, τ≈4s@20Hz)
        if abs(self.current_speed_ms) < 0.05:
            self.gyro_bias_z += 0.012 * (gz - self.gyro_bias_z)
        self.imu_last_time = time.time()

    def cb_ego(self, msg: Float64MultiArray):
        d = msg.data
        if len(d) >= 7:
            new_lat, new_lon = d[0], d[1]
            # GPS 점프 감지 → WP 재동기화 윈도우 확대
            if self.prev_lat_for_jump is not None and self.current_lat is not None:
                R = 6378137.0
                dx = math.radians(new_lon - self.prev_lon_for_jump) * math.cos(math.radians(new_lat)) * R
                dy = math.radians(new_lat - self.prev_lat_for_jump) * R
                jump = math.hypot(dx, dy)
                if jump > 1.0:
                    self.gps_jump_recover_cycles = 4
                    self.publish_event(f"📍 위치 점프 {jump:.2f}m → WP 재동기화")
            self.prev_lat_for_jump = new_lat
            self.prev_lon_for_jump = new_lon

            self.current_lat   = new_lat
            self.current_lon   = new_lon
            self.heading       = d[4]
            self.current_steer = d[6]

            now = time.time()
            self.ego_speed_ms = float(d[5])
            # 엔코더가 끊기면 ego 속도로 대체
            if self.encoder_last_time is None or (now - self.encoder_last_time) > 0.50:
                self.current_speed_ms = self.ego_speed_ms
                self.current_encoder_ticks_lpf = self.speed_ms_to_ticks_float(self.current_speed_ms)
        if len(d) >= 9:
            self.imu_pitch    = d[7]
            self.terrain_code = d[8]

    def cb_terrain(self, msg: Float64MultiArray):
        # [v6.3.1] 원본과 동일하게 /terrain_state 구독 복원.
        # terrain은 /ego_state[8]에도 실려오지만, 별도 토픽도 병행 반영(안전).
        d = msg.data
        if len(d) >= 1:
            self.terrain_code = d[0]

    def cb_gps_status(self, msg: String):
        s = msg.data
        self.dr_active = "[DR중]" in s
        new_blend = "[GPS복구중]" in s
        if self.prev_gps_blend_on and not new_blend:
            now = time.time()
            if now - self.last_gps_blend_event_time > 1.0:
                self.gps_jump_recover_cycles = 6
                self.last_gps_blend_event_time = now
                self.publish_event("📍 GPS 블렌딩 종료 → WP 재동기화")
        self.prev_gps_blend_on = new_blend
        self.gps_blend_on = new_blend

    def cb_drive_cmd(self, msg: String):
        cmd = msg.data.strip()
        self.get_logger().info(f"📩 /drive_cmd 수신: '{cmd}'")   # [v6.3.1] 명령 하달 확인용
        if cmd == "STOP":
            self.request_stop()
        else:
            self.load_waypoints(cmd)

    # ══════════════════════════════════════════════════════════════════════
    #  경로 로드 (루프 자동 절단 + 시작 WP 정렬)
    # ══════════════════════════════════════════════════════════════════════
    def load_waypoints(self, filename):
        path = os.path.join(self.data_dir, filename)
        if not os.path.exists(path):
            self.publish_event(f"❌ 경로 파일 없음: {path}", level="error")
            return

        raw_wps = []
        with open(path, 'r') as f:
            for row in csv.DictReader(f):
                raw_wps.append({
                    'lat': float(row['latitude']),
                    'lon': float(row['longitude']),
                    'direction': int(float(row.get('direction', '1'))),
                })
        if not raw_wps:
            self.publish_event("❌ WP가 없습니다!", level="error")
            return

        R = 6378137.0
        def _dist(a, b):
            dx = math.radians(b['lon'] - a['lon']) * math.cos(math.radians(a['lat'])) * R
            dy = math.radians(b['lat'] - a['lat']) * R
            return math.hypot(dx, dy)

        # 루프 경로 자동 절단
        wp0 = raw_wps[0]
        loop_dist = _dist(wp0, raw_wps[-1])
        cut_idx = len(raw_wps)
        if loop_dist < self.LOOP_DETECT_THRESHOLD and len(raw_wps) > 20:
            far_reached = False
            for i in range(1, len(raw_wps)):
                d_from_start = _dist(wp0, raw_wps[i])
                if not far_reached:
                    if d_from_start > self.LOOP_FAR_DIST:
                        far_reached = True
                else:
                    if d_from_start < self.LOOP_RETURN_DIST:
                        cut_idx = i
                        break
            if cut_idx < len(raw_wps):
                self.publish_event(f"🔁 루프 자동 절단: {len(raw_wps)}→{cut_idx}개")

        self.waypoints = raw_wps[:cut_idx]

        # 상태 초기화
        self.wp_idx = 0
        self.cte_integral = 0.0
        self.prev_cte = 0.0
        self.cte_lpf = 0.0
        self.cte_lpf_ready = False
        self.spd_integral = 0.0
        self.current_speed_ms = 0.0
        self.ego_speed_ms = 0.0
        self.encoder_last_time = None
        self.last_published_speed = 0.0
        self.prev_cte_seg_idx = 0
        self.target_speed_lpf = self.min_speed_ms
        self.active = True
        self._dispatch_logged = False   # [v6.3.1] 새 주행마다 첫 하달 로그 재활성화
        self.d_term_lpf = 0.0
        self._lfd_lpf = self.min_lfd
        self._near_demand_lpf = 0.0   # [v6.4] 요구조향 평활 리셋
        self._near_peak_lpf   = 0.0   # [v6.7.3] 피크 평활 리셋
        self._cte_rate_lpf = 0.0      # [v6.5] 지연보상 rate 리셋
        self.prev_lat_for_jump = None
        self.prev_lon_for_jump = None
        self.gps_jump_recover_cycles = 0
        self.approach_reset_done = False
        self.min_d2dest_seen = float('inf')
        self.ramp_up_active = True
        self.drive_start_time = time.time()
        self.prev_steer = 0.0

        # 시작 WP 정렬 (현재 위치/헤딩에서 전방 가장 가까운 WP)
        if self.current_lat is not None and len(self.waypoints) > 0:
            psi = math.radians(self.heading)
            search_limit = max(20, len(self.waypoints) // 2)
            min_d, best = float('inf'), 0
            for i in range(min(search_limit, len(self.waypoints))):
                wp = self.waypoints[i]
                dx = math.radians(wp['lon'] - self.current_lon) * math.cos(math.radians(self.current_lat)) * R
                dy = math.radians(wp['lat'] - self.current_lat) * R
                lx = dx * math.cos(psi) + dy * math.sin(psi)
                d = math.hypot(dx, dy)
                if lx > -0.5 and d < min_d:
                    min_d, best = d, i
            if min_d == float('inf'):
                for i in range(min(search_limit, len(self.waypoints))):
                    wp = self.waypoints[i]
                    dx = math.radians(wp['lon'] - self.current_lon) * math.cos(math.radians(self.current_lat)) * R
                    dy = math.radians(wp['lat'] - self.current_lat) * R
                    d = math.hypot(dx, dy)
                    if d < min_d:
                        min_d, best = d, i
            self.wp_idx = best
            self.publish_event(f"🔍 시작 WP={best}/{len(self.waypoints)} (거리 {min_d:.2f}m)")
        else:
            self.wp_idx = 0
            self.publish_event("⚠ GPS 미수신 → wp_idx=0")

        self.state_pub.publish(Bool(data=True))
        rev = sum(1 for w in self.waypoints if w['direction'] == -1)
        self.publish_event(f"📂 경로 로드: {filename} | WP={len(self.waypoints)} (후진 {rev}) | 모터 활성화")

    # ══════════════════════════════════════════════════════════════════════
    #  정지
    # ══════════════════════════════════════════════════════════════════════
    def request_stop(self):
        self.publish_event("🛑 정지 요청 → 즉시 정지", level="warn")
        self.instant_stop()

    def publish_cmd(self, speed_ms: float, steer_deg: float):
        speed_ms = max(-self.max_speed_ms, min(self.max_speed_ms, float(speed_ms)))
        # [v6.7] 조향 실효게인 보정 + [v6.7.1] 트림 보정(실측 -1.5°)
        #   + [v6.7.2] 좌/우 게인 분리(우회전 바깥밀림 -0.1m 보정):
        #   물리 조향 ≈ GAIN·δ_pub + TRIM  →  δ_pub = (δ_ctrl − TRIM)/GAIN
        #   (제어기 내부 수치·디버그 로그는 보정 전 값 유지 → 튜닝 의미 보존)
        #   방향은 트림 보정 전 제어 조향(δ_ctrl) 부호로 판정(+좌/−우).
        plant_gain = self.STEER_PLANT_GAIN_R if float(steer_deg) < 0.0 else self.STEER_PLANT_GAIN_L
        steer_out = (float(steer_deg) - self.STEER_TRIM_DEG) / max(0.5, plant_gain)
        steer_out = max(-self.STEER_MAX_DEG, min(self.STEER_MAX_DEG, steer_out))
        msg = Twist()
        msg.linear.x  = speed_ms
        msg.angular.z = steer_out
        self.drive_pub.publish(msg)
        self.last_published_speed = speed_ms

    def instant_stop(self):
        self.state_pub.publish(Bool(data=False))
        msg = Twist()
        msg.linear.x = msg.angular.z = 0.0
        self.drive_pub.publish(msg)
        self.last_published_speed = 0.0
        self.spd_integral = 0.0
        self.active = False
        self.publish_event("🛑 모터 즉시 정지 (control_state=False)")

    # ══════════════════════════════════════════════════════════════════════
    #  인지(1): 전방 곡률 → 요구 조향각 스캔  [선행 가감속의 핵심 신호]
    #  경로 기하에서 직접 계산하므로 순간 조향 잡음과 무관(잔떨림 면역).
    #  반환: (요구조향각[deg], 그 지점까지 거리[m])
    # ══════════════════════════════════════════════════════════════════════
    def scan_curve_demand(self, from_idx: int, preview_dist: float):
        wps = self.waypoints
        if not wps or len(wps) < 3 or self.current_lat is None:
            return 0.0, float('inf'), 0.0, float('inf')
        R = 6378137.0
        start = max(0, min(from_idx, len(wps) - 3))
        ref = wps[start]
        ref_lat, ref_lon = ref['lat'], ref['lon']

        # 로컬 좌표 누적 (preview_dist까지)
        pts = []
        s = [0.0]
        for i in range(start, len(wps)):
            w = wps[i]
            x = math.radians(w['lon'] - ref_lon) * math.cos(math.radians(ref_lat)) * R
            y = math.radians(w['lat'] - ref_lat) * R
            if pts:
                s.append(s[-1] + math.hypot(x - pts[-1][0], y - pts[-1][1]))
                if s[-1] > preview_dist:
                    pts.append((x, y)); break
            pts.append((x, y))
        if len(pts) < 3:
            return 0.0, float('inf'), 0.0, float('inf')

        def idx_at(dist_m):
            for k in range(1, len(s)):
                if s[k] >= dist_m:
                    return k
            return len(s) - 1

        def heading_between(i0, i1):
            return math.atan2(pts[i1][1] - pts[i0][1], pts[i1][0] - pts[i0][0])

        win = self.CURVE_WINDOW_M
        win_pk = self.CURVE_WINDOW_PEAK_M          # [v6.7.3] 짧은 창(급코너 피크 포착)
        max_steer_deg = 0.0
        dist_at_max = float('inf')
        peak_steer_deg = 0.0                       # [v6.7.3] 짧은 창 기준 최대(과소평가 보정)
        peak_dist = float('inf')
        d = win_pk                                 # [v6.7.3 감사수정] 피크는 win_pk부터 스캔
                                                   #   (기존 d=win 시작 → 전방 0~1.8m 사각 →
                                                   #    코너 '진입 중' 피크 조기소실로 LFD캡이
                                                   #    코너 한복판에서 풀리는 문제 차단)
        scan_end = min(preview_dist, s[-1])
        while d <= scan_end:
            if d >= win:                           # 평활(3m창) 신호는 기존과 동일 지점부터
                i_p0 = idx_at(max(0.0, d - win))
                i_p1 = idx_at(d)
                i_c1 = idx_at(min(d + win, s[-1]))
                if i_p1 > i_p0 and i_c1 > i_p1:
                    h_prev = heading_between(i_p0, i_p1)
                    h_curr = heading_between(i_p1, i_c1)
                    diff = h_curr - h_prev
                    while diff >  math.pi: diff -= 2*math.pi
                    while diff < -math.pi: diff += 2*math.pi
                    # 곡률 κ = |Δθ|/Δs, 요구 조향 δ = atan(L·κ)
                    curvature = abs(diff) / win
                    req_steer = math.degrees(math.atan(self.vehicle_length * curvature))
                    if req_steer > max_steer_deg:
                        max_steer_deg = req_steer
                        dist_at_max = d
            # [v6.7.3] 짧은 창 Menger 곡률(3점 외접원): R≈2m 급코너를 3m 창이 ~11°로
            #   과소평가하는 것을 보정. 짧은 호에서도 정확한 최대곡률을 포착.
            #   (실측 근거: 같은 코너를 far 스캔은 16.4°로, near 3m창은 11.1°로 읽었음)
            i_pm = idx_at(d)
            j0 = idx_at(max(0.0, d - win_pk))
            j2 = idx_at(min(d + win_pk, s[-1]))
            if j0 < i_pm < j2:
                ax, ay = pts[j0]; bx, by = pts[i_pm]; cx, cy = pts[j2]
                la = math.hypot(bx-ax, by-ay)
                lb = math.hypot(cx-bx, cy-by)
                lc = math.hypot(cx-ax, cy-ay)
                area2 = abs((bx-ax)*(cy-ay) - (cx-ax)*(by-ay))   # 2×삼각형 넓이
                if area2 > 1e-6 and la*lb*lc > 1e-9:
                    R_menger = (la*lb*lc) / (2.0*area2)           # 외접원 반경
                    req_pk = math.degrees(math.atan(self.vehicle_length / R_menger))
                    if req_pk > peak_steer_deg:
                        peak_steer_deg = req_pk
                        peak_dist = d
            d += 0.4
        return max_steer_deg, dist_at_max, peak_steer_deg, peak_dist

    # ══════════════════════════════════════════════════════════════════════
    #  인지(2): 세그먼트 CTE (현재 위치의 경로 횡오차)
    # ══════════════════════════════════════════════════════════════════════
    def compute_segment_cte(self, from_idx: int, psi_eff: float, is_rev: bool, R: float):
        wps = self.waypoints
        if not wps or self.current_lat is None or len(wps) < 2:
            return 0.0, from_idx, 0.0
        # [v6.7] 예측좌표 기준(지연보상). 예측 미설정 시 원시좌표 폴백.
        clat = self.ctrl_lat if self.ctrl_lat is not None else self.current_lat
        clon = self.ctrl_lon if self.ctrl_lon is not None else self.current_lon
        start = max(0, from_idx - 3)
        end   = min(len(wps) - 2, from_idx + 25)
        best_dist = float('inf'); best_x = best_y = 0.0; best_i = from_idx
        for i in range(start, end + 1):
            wp0, wp1 = wps[i], wps[i + 1]
            x0 = math.radians(wp0['lon'] - clon) * math.cos(math.radians(clat)) * R
            y0 = math.radians(wp0['lat'] - clat) * R
            x1 = math.radians(wp1['lon'] - clon) * math.cos(math.radians(clat)) * R
            y1 = math.radians(wp1['lat'] - clat) * R
            vx, vy = x1 - x0, y1 - y0
            denom = vx*vx + vy*vy
            if denom < 1e-6:
                continue
            t = max(0.0, min(1.0, -((x0*vx) + (y0*vy)) / denom))
            px, py = x0 + t*vx, y0 + t*vy
            dist = math.hypot(px, py)
            if dist < best_dist:
                best_dist, best_x, best_y, best_i = dist, px, py, i
        cte = -best_x * math.sin(psi_eff) + best_y * math.cos(psi_eff)
        if is_rev:
            cte = -cte
        return cte, best_i, best_dist

    # ══════════════════════════════════════════════════════════════════════
    #  종점 헤딩/가상타깃 헬퍼 (APPROACH 정지용)
    # ══════════════════════════════════════════════════════════════════════
    def _norm_deg(self, deg):
        while deg > 180.0: deg -= 360.0
        while deg < -180.0: deg += 360.0
        return deg

    def get_final_path_heading_deg(self):
        wps = self.waypoints
        if not wps or len(wps) < 2:
            return self.heading
        R = 6378137.0
        i0 = max(0, len(wps) - 9); i1 = len(wps) - 1
        a, b = wps[i0], wps[i1]
        dx = math.radians(b['lon'] - a['lon']) * math.cos(math.radians(a['lat'])) * R
        dy = math.radians(b['lat'] - a['lat']) * R
        if math.hypot(dx, dy) < 1e-6:
            a = wps[max(0, i1 - 1)]
            dx = math.radians(b['lon'] - a['lon']) * math.cos(math.radians(a['lat'])) * R
            dy = math.radians(b['lat'] - a['lat']) * R
        h = math.degrees(math.atan2(dy, dx))
        if wps[-1].get('direction', 1) == -1:
            h = self._norm_deg(h + 180.0)
        return self._norm_deg(h)

    def get_final_heading_error_deg(self):
        return self._norm_deg(self.get_final_path_heading_deg() - self.heading)

    def final_line_target_latlon(self, lookahead_m: float, overshoot_max_m: float = 1.0):
        dest = self.waypoints[-1]
        R = 6378137.0
        h = math.radians(self.get_final_path_heading_deg())
        ux, uy = math.cos(h), math.sin(h)
        px = math.radians(self.current_lon - dest['lon']) * math.cos(math.radians(dest['lat'])) * R
        py = math.radians(self.current_lat - dest['lat']) * R
        t = px*ux + py*uy
        t_tgt = min(overshoot_max_m, t + lookahead_m)
        lat = dest['lat'] + math.degrees((t_tgt*uy) / R)
        lon = dest['lon'] + math.degrees((t_tgt*ux) / (R * math.cos(math.radians(dest['lat']))))
        return lat, lon

    # ══════════════════════════════════════════════════════════════════════
    #  판단: PID 게인 테이블 조회 (속도 → Kp,Ki,Kd 선형보간)
    # ══════════════════════════════════════════════════════════════════════
    def lookup_gains(self, v: float):
        T = GAIN_TABLE
        if v <= T[0][0]:
            return T[0][1], T[0][2], T[0][3]
        if v >= T[-1][0]:
            return T[-1][1], T[-1][2], T[-1][3]
        for k in range(len(T) - 1):
            v0, kp0, ki0, kd0 = T[k]
            v1, kp1, ki1, kd1 = T[k + 1]
            if v0 <= v <= v1:
                r = (v - v0) / (v1 - v0)
                return (kp0 + (kp1 - kp0)*r,
                        ki0 + (ki1 - ki0)*r,
                        kd0 + (kd1 - kd0)*r)
        return T[-1][1], T[-1][2], T[-1][3]

    def lookup_lfd(self, v: float) -> float:
        """속도 → 전방주시거리(LFD) 선형보간 (LFD_TABLE)."""
        T = LFD_TABLE
        if v <= T[0][0]:
            return T[0][1]
        if v >= T[-1][0]:
            return T[-1][1]
        for k in range(len(T) - 1):
            v0, l0 = T[k]
            v1, l1 = T[k + 1]
            if v0 <= v <= v1:
                r = (v - v0) / (v1 - v0)
                return l0 + (l1 - l0) * r
        return T[-1][1]

    # ══════════════════════════════════════════════════════════════════════
    #  디버그/상태 토픽
    # ══════════════════════════════════════════════════════════════════════
    def _publish_debug(self, fields):
        msg = Float64MultiArray()
        msg.data = [float(x) for x in fields]
        self.debug_pub.publish(msg)

    def _publish_status_if_due(self, wp_total, d2dest, signed_spd, reason, cte, is_rev, demand_deg, dist_curve):
        now = time.time()
        if now - self.last_status_publish < self.STATUS_PUBLISH_INTERVAL:
            return
        self.last_status_publish = now
        progress = self.wp_idx / max(wp_total - 1, 1) * 100.0
        gps_mode = "DR" if self.dr_active else ("BLEND" if self.gps_blend_on else "RTK")
        rev_m = "REV" if is_rev else "FWD"
        if dist_curve == float('inf'):
            curve_str = "curve=none"
        else:
            curve_str = f"curve={demand_deg:.0f}°@{dist_curve:.1f}m"
        self.status_pub.publish(String(data=(
            f"[{gps_mode}] WP={self.wp_idx}/{wp_total} ({progress:.0f}%) {rev_m} | "
            f"tgt={signed_spd:+.2f} real={self.current_speed_ms:+.2f}m/s | CTE={cte:+.2f}m | "
            f"{reason} | {curve_str} | d2dest={d2dest:.1f}m")))

    # ══════════════════════════════════════════════════════════════════════
    #  메인 제어 루프 (20Hz):  인지 → 판단 → 제어
    # ══════════════════════════════════════════════════════════════════════
    def control_loop(self):
        now_ctl = time.time()
        loop_dt = 0.05 if self.last_control_time is None else max(0.01, min(0.2, now_ctl - self.last_control_time))
        self.last_control_time = now_ctl

        if not self.active or not self.waypoints or self.current_lat is None:
            # [v6.3.1] 진단: 활성인데 못 나가는 이유를 2초마다 알림(하달 안 될 때 원인 노출)
            if self.active:
                now_w = time.time()
                if now_w - getattr(self, '_last_wait_log', 0.0) > 2.0:
                    self._last_wait_log = now_w
                    if not self.waypoints:
                        self.get_logger().warning("⏳ 주행 대기: 경로(waypoints) 비어있음 — /drive_cmd 파일 로드 실패 확인")
                    elif self.current_lat is None:
                        self.get_logger().warning("⏳ 주행 대기: /ego_state(GPS) 미수신 — gps_imu 노드/GPS fix 확인 (이 경우 /cmd_vel_raw 미발행→아두이노 워치독이 모터 정지)")
            return

        R = 6378137.0
        psi = math.radians(self.heading)

        # ─────────────────────────────────────────────────────────────────
        # [v6.7] 지연 보상: τ초 뒤 자세 예측 → 이 자세로 WP탐색·CTE·PP 계산.
        #   (도착판정·d2dest·APPROACH는 원시좌표 사용 — 예측으로 일찍 서지 않게)
        #   실측 지연: 조향 0.45s + 센싱 ~0.1s. 예측은 보수적으로 0.40s(기본).
        # ─────────────────────────────────────────────────────────────────
        tau = self.PREDICT_HORIZON_S
        imu_fresh_pred = (self.imu_last_time is not None
                          and (now_ctl - self.imu_last_time) < 0.4)
        if tau > 1e-3:
            if imu_fresh_pred:
                omega_z = max(-2.5, min(2.5, self.imu_gyro_z - self.gyro_bias_z))  # 실측 요레이트(바이어스 차감)
            else:
                # IMU 미수신 폴백: 직전 조향명령 기반 기대 요레이트
                # (게인보정계에선 실현조향≈제어량이므로 tan(prev_steer) 그대로)
                omega_z = (self.current_speed_ms
                           * math.tan(math.radians(self.prev_steer)) / self.vehicle_length)
            dpsi = max(-self.PREDICT_MAX_YAW_RAD,
                       min(self.PREDICT_MAX_YAW_RAD, omega_z * tau))
            psi_mid = psi + 0.5 * dpsi
            adv = self.current_speed_ms * tau                           # 부호 포함(후진 지원)
            self.ctrl_lat = self.current_lat + math.degrees(adv * math.sin(psi_mid) / R)
            self.ctrl_lon = self.current_lon + math.degrees(
                adv * math.cos(psi_mid) / (R * math.cos(math.radians(self.current_lat))))
            psi = psi + dpsi                                            # 예측 헤딩으로 제어
        else:
            imu_fresh_pred = False
            self.ctrl_lat, self.ctrl_lon = self.current_lat, self.current_lon

        cur_dir = self.waypoints[self.wp_idx]['direction']
        is_rev  = (cur_dir == -1)
        psi_eff = (psi + math.pi) if is_rev else psi
        elapsed_drive = max(0.0, time.time() - self.drive_start_time) if self.drive_start_time else 0.0

        # ─────────────────────────────────────────────────────────────────
        # 인지 1. WP 탐색
        # ─────────────────────────────────────────────────────────────────
        search_window = self.WP_SEARCH_WINDOW
        max_advance   = self.MAX_WP_ADVANCE_NORMAL
        if self.wp_idx < 5 and elapsed_drive < self.START_ALIGN_DURATION:
            search_window = max(search_window, 120)
            max_advance   = self.MAX_WP_ADVANCE_START
        elif self.gps_jump_recover_cycles > 0:
            search_window = max(search_window, 70)
            max_advance   = max(max_advance, 12)
            self.gps_jump_recover_cycles -= 1

        search_end = min(self.wp_idx + search_window, len(self.waypoints))
        min_ahead, best_ahead = float('inf'), None
        min_any,   best_any   = float('inf'), self.wp_idx
        for i in range(self.wp_idx, search_end):
            wp = self.waypoints[i]
            dx = math.radians(wp['lon']-self.ctrl_lon)*math.cos(math.radians(self.ctrl_lat))*R   # [v6.7] 예측좌표
            dy = math.radians(wp['lat']-self.ctrl_lat)*R
            lx = dx*math.cos(psi_eff) + dy*math.sin(psi_eff)
            d  = math.hypot(dx, dy)
            if d < min_any:
                min_any, best_any = d, i
            if lx > -2.0 and d < min_ahead:
                min_ahead, best_ahead = d, i
        best = best_any if (best_ahead is None or min_ahead > min_any + 1.2) else best_ahead
        best = max(self.wp_idx, min(best, self.wp_idx + max_advance))
        self.wp_idx = best

        # ─────────────────────────────────────────────────────────────────
        # 인지 2. 도착 판정 / 종점 거리
        # ─────────────────────────────────────────────────────────────────
        dest = self.waypoints[-1]
        dx_d = math.radians(dest['lon']-self.current_lon)*math.cos(math.radians(self.current_lat))*R
        dy_d = math.radians(dest['lat']-self.current_lat)*R
        d2dest = math.hypot(dx_d, dy_d)
        wp_total = len(self.waypoints)

        near_final_wp = self.wp_idx >= max(0, wp_total - 10)
        if near_final_wp and d2dest < self.min_d2dest_seen:
            self.min_d2dest_seen = d2dest
        passed_goal = (near_final_wp and self.min_d2dest_seen <= self.PASS_DETECT_MIN_DIST and
                       d2dest > self.min_d2dest_seen + self.PASS_HYSTERESIS)

        final_zone_idx = max(0, wp_total - self.FINAL_APPROACH_WP_WINDOW)
        approach_active = (self.wp_idx >= final_zone_idx and d2dest <= self.APPROACH_START_DIST)
        approach_speed_cap = self.max_speed_ms
        final_heading_error = self.get_final_heading_error_deg()
        final_heading_ok = abs(final_heading_error) <= self.FINAL_HEADING_TOL_DEG

        if approach_active:
            ratio = max(0.0, min(1.0, d2dest / self.APPROACH_START_DIST))
            approach_speed_cap = self.APPROACH_END_SPEED + \
                (self.FINAL_APPROACH_ENTRY_SPEED - self.APPROACH_END_SPEED) * (ratio ** 1.3)
            if d2dest <= self.FINAL_CREEP_DIST:
                creep_cap = self.APPROACH_END_SPEED + \
                    (self.FINAL_CREEP_SPEED - self.APPROACH_END_SPEED) * (d2dest / self.FINAL_CREEP_DIST)
                approach_speed_cap = min(approach_speed_cap, creep_cap)
            if abs(final_heading_error) > self.FINAL_HEADING_TOL_DEG:
                approach_speed_cap = min(approach_speed_cap, max(0.35, 0.18 + 0.18 * d2dest))
            approach_speed_cap = max(self.APPROACH_END_SPEED, min(self.FINAL_APPROACH_ENTRY_SPEED, approach_speed_cap))
            if not self.approach_reset_done and d2dest < self.APPROACH_START_DIST * 0.6:
                self.cte_integral = 0.0
                self.d_term_lpf = 0.0
                self.approach_reset_done = True
                self.publish_event(f"🎯 APPROACH 진입 reset (d2dest={d2dest:.2f}m)")
        else:
            self.approach_reset_done = False

        slow_enough = abs(self.current_speed_ms) < self.STOP_CONFIRM_SPEED
        final_stop_slow = abs(self.current_speed_ms) < self.FINAL_STOP_SPEED
        final_force_close = d2dest < self.FINAL_FORCE_STOP_DIST and slow_enough
        arrived = (
            (self.wp_idx >= wp_total - 3 and d2dest < self.STOP_CONFIRM_DIST and slow_enough and
             (final_heading_ok or d2dest < self.FINAL_FORCE_STOP_DIST)) or
            (d2dest < self.HARD_ARRIVAL_DIST and final_stop_slow and
             (final_heading_ok or d2dest < self.FINAL_FORCE_STOP_DIST)) or
            (self.wp_idx >= wp_total - 2 and d2dest < self.FINAL_ARRIVAL_DIST and final_stop_slow and
             (final_heading_ok or final_force_close)) or
            passed_goal or final_force_close
        )
        if arrived:
            self.publish_event(
                f"🎯 도착 — 정지 d={d2dest:.2f}m v={self.current_speed_ms:.2f}m/s "
                f"head_err={final_heading_error:+.1f}° wp={self.wp_idx}/{wp_total}")
            self.instant_stop()
            return

        # ─────────────────────────────────────────────────────────────────
        # 인지 3. 전방 곡률 → 요구 조향각 (선행 가감속 신호)
        # ─────────────────────────────────────────────────────────────────
        v_now = abs(self.current_speed_ms)
        # [v6.7] 테이블 조회 전용 저역 속도(엔코더 양자화 → LFD/게인 변조 차단)
        alpha_s = loop_dt / (self.V_SCHED_TAU + loop_dt)
        self.v_sched += alpha_s * (v_now - self.v_sched)
        # 근거리(현재 코너) 스캔 + 평활(속도·LFD 맥동→결합진동 억제)
        near_preview = max(self.min_lfd * self.CURVE_PREVIEW_NEAR_K, self._lfd_lpf * self.CURVE_PREVIEW_NEAR_K)
        near_demand_raw, _, near_peak_raw, _ = self.scan_curve_demand(self.wp_idx, near_preview)
        self._near_demand_lpf += self.NEAR_DEMAND_LPF_ALPHA * (near_demand_raw - self._near_demand_lpf)
        near_demand_deg = self._near_demand_lpf
        # [v6.7.3] 급코너 피크(Menger)를 '곡률캡 전용'으로 별도 평활 유지.
        #   3m창은 R≈2m 필렛을 ~11°로 과소평가 → LFD가 안 짧아져 코너컷/바깥밀림.
        #   피크를 곡률캡에만 강하게 반영(LFD↓로 컷 차단), 속도엔 아래서 '부분만' 반영
        #   → 완만·보통 코너의 순항속도는 보존(정상코너 감속 방지), 급코너만 교정.
        self._near_peak_lpf += self.NEAR_DEMAND_LPF_ALPHA * (
            max(near_demand_raw, near_peak_raw) - self._near_peak_lpf)
        near_peak_deg = self._near_peak_lpf
        # 원거리(다가오는 코너) 스캔
        far_demand_deg, far_dist, far_peak, far_peak_dist = self.scan_curve_demand(
            self.wp_idx, self.CURVE_PREVIEW_FAR_MAX)
        # [v6.7.3] 원거리 급코너 피크: 다가오는 급코너를 일찍 감지(제동 게이팅용).
        far_peak_deg = max(far_demand_deg, far_peak)

        # ─────────────────────────────────────────────────────────────────
        # 인지 4. CTE
        # ─────────────────────────────────────────────────────────────────
        cte_raw, cte_seg_idx, _ = self.compute_segment_cte(self.wp_idx, psi_eff, is_rev, R)
        if (not self.cte_lpf_ready or abs(cte_seg_idx - self.prev_cte_seg_idx) > 20
                or abs(cte_raw - self.cte_lpf) > 6.0):
            self.cte_lpf = cte_raw
            self.cte_integral = 0.0
            self.d_term_lpf = 0.0
            self.cte_lpf_ready = True
        else:
            self.cte_lpf += self.CTE_LPF_ALPHA * (cte_raw - self.cte_lpf)
        self.prev_cte_seg_idx = cte_seg_idx
        cte = max(-4.0, min(4.0, self.cte_lpf))
        self.wp_idx = max(self.wp_idx, cte_seg_idx)
        cte_abs = abs(cte)

        # ── [CAM] 카메라 차선 CTE 가중융합 — PID 채널 입력 확정 ──────────
        #   cte ← w·cam_cte_rear + (1−w)·gps_cte
        #   · 섀도 모드(cam_enable=False): w=0 고정, /fusion_debug 로깅만
        #   · DR: w→cam_w_dr(0.9) = 차선이 횡방향 전담 / 평상시 w≤cam_w_max(0.4)
        #   · 후진·신뢰도미달·계측stale·GPS와 과대괴리 시 w 목표 0 (슬루로 완만 전환)
        cam_w = 0.0
        if self.camfu is not None:
            cte, cam_w = self.camfu.fuse_cte(cte, is_rev=is_rev, dr_active=self.dr_active)
            cte_abs = abs(cte)

        # ── [v6.6.1] 지연 보상: 휠베이스 기하로 '한 휠베이스 앞' CTE 예측 ──────────
        #   앞차축 CTE(cte) + L·sin(θe) = 앞차축보다 L 앞선 지점의 CTE (θe=경로접선−heading).
        #   시간상수 없이 preview 거리=L(휠베이스), 시간=L/v 로 속도 자동적응. v 상쇄→강건.
        si = max(0, min(cte_seg_idx, len(self.waypoints) - 2))
        w0 = self.waypoints[si]; w1 = self.waypoints[si + 1]
        seg_dx = math.radians(w1['lon'] - w0['lon']) * math.cos(math.radians(w0['lat'])) * R
        seg_dy = math.radians(w1['lat'] - w0['lat']) * R
        if math.hypot(seg_dx, seg_dy) > 1e-6:
            path_psi = math.atan2(seg_dy, seg_dx)
            he = path_psi - psi_eff                    # 헤딩오차 θe [rad]
            while he >  math.pi: he -= 2 * math.pi
            while he < -math.pi: he += 2 * math.pi
            lead_raw = self.LEAD_WHEELBASE * math.sin(he)   # 기하 lead[m] = L·sin(θe)
            if is_rev: lead_raw = -lead_raw                 # cte 후진 부호반전과 일치
        else:
            lead_raw = 0.0
            path_psi = psi_eff   # [CAM] 세그먼트 퇴화 시 접선 폴백(헤딩참조 발행용)
        self._cte_rate_lpf += self.LEAD_RATE_LPF_ALPHA * (lead_raw - self._cte_rate_lpf)
        cte_lead = max(-self.LEAD_CLAMP, min(self.LEAD_CLAMP, self._cte_rate_lpf))
        cte_ctrl = max(-4.0, min(4.0, cte + cte_lead))   # 지연보상된 제어용 CTE (앞차축 기하 예측)

        is_straight = (far_peak_deg < self.JITTER_DEMAND_DEG)   # [v6.7.3 감사수정] 피크 기준:
        #   windowed far가 5°여도 14m 내 급피크가 있으면 '직선'이 아님 → 급코너 접근 중
        #   잔떨림 억제·큰 데드밴드가 초기 턴인을 지연시키는 것 방지.

        # ─────────────────────────────────────────────────────────────────
        # ── [CAM] DR용 절대헤딩 참조 발행: ψ추정 = 지도접선 − θ_lane ──────
        #   게이트(카메라 신선·conf, |θ|≤8°, 지도 직선 near_peak≤3°, is_straight)
        #   통과 시에만 /lane_heading_ref 발행 → gps_imu 가 DR 중 자이로 drift 억제.
        if self.camfu is not None:
            self.camfu.update_route_ref(math.degrees(path_psi), near_peak_deg, is_straight)

        # 판단 1. LFD — 속도 테이블(LFD_TABLE) ∩ 곡률캡 (코너컷 직접 차단)
        #   코너컷 오차 ≈ LFD²/(8R). R_ahead = L/tan(near_demand).
        #   곡률캡 = √(LFD_CURVE_CAP_K·R_ahead) → CTE ≤ 약 0.18m 보장.
        #   최종 LFD = min(속도표, 곡률캡). 코너가 급할수록 LFD가 즉시 짧아진다.
        # ─────────────────────────────────────────────────────────────────
        lfd_speed = self.lookup_lfd(self.v_sched)   # [v6.7] 저역 속도로 조회(플러터 차단)
        if near_peak_deg > 2.5:   # [v6.7.3] 곡률캡은 피크 기준 → 급코너서 LFD 확실히 단축
            R_ahead = self.vehicle_length / math.tan(math.radians(min(near_peak_deg, 25.0)))
            lfd_curve_cap = math.sqrt(self.LFD_CURVE_CAP_K * R_ahead)
        else:
            lfd_curve_cap = self.max_lfd
        # [v6.7.3] OMEGA 결속 기준용: '평활 near'만으로 정한 LFD(피크 단축분 제외).
        #   피크는 짧은 기하 특징이라 LFD를 순간 단축하지만, 그것까지 ω_n 속도결속에
        #   넣으면 매 필렛마다 과잉 감속(OMEGA 36%). 지속 곡률(windowed)만 결속에 사용.
        if near_demand_deg > 2.5:
            R_win = self.vehicle_length / math.tan(math.radians(min(near_demand_deg, 25.0)))
            lfd_cap_win = math.sqrt(self.LFD_CURVE_CAP_K * R_win)
        else:
            lfd_cap_win = self.max_lfd
        lfd_win_only = min(self.max_lfd, max(self.min_lfd, min(lfd_speed, lfd_cap_win)))
        lfd_target = min(self.max_lfd, max(self.min_lfd, min(lfd_speed, lfd_curve_cap)))
        self._lfd_lpf += self.LFD_LPF_ALPHA * (lfd_target - self._lfd_lpf)
        # [v6.7] LFD 비대칭 슬루: 감소는 신속(코너 보호 유지), 증가는 완만(탈출 후 게인 점프 방지)
        self._lfd_out = min(self._lfd_out + self.LFD_RATE_UP * loop_dt,
                            max(self._lfd_out - self.LFD_RATE_DOWN * loop_dt, self._lfd_lpf))
        dynamic_lfd = max(self.min_lfd, min(self.max_lfd, self._lfd_out))
        if approach_active:
            lfd_cap = max(self.APPROACH_LFD_MIN, d2dest * self.APPROACH_LFD_RATIO + self.APPROACH_LFD_OFFSET)
            dynamic_lfd = min(self._lfd_lpf, lfd_cap)

        # ─────────────────────────────────────────────────────────────────
        # 판단 2. 목표 속도 — 선행 가감속(예측 제어)
        #   (a) 근거리 곡률 비례 감속: 현재 코너 속도
        #   (b) 원거리 제동거리 게이팅: 다가오는 코너에 "딱 필요한 만큼만" 미리 감속
        #   (c) CTE 회복 감속(안전)
        # ─────────────────────────────────────────────────────────────────
        def demand_to_speed(demand_deg):
            r = max(0.0, min(1.0, demand_deg / self.STEER_FULL_SLOWDOWN_DEG))
            return self.max_speed_ms - (self.max_speed_ms - self.min_speed_ms) * r

        # (a) 현재 코너 속도 (곡률 비례)
        #   [v6.7.3] 급코너 피크를 '부분만' 반영: 완만·보통 코너(peak≈near)는 순항속도
        #   보존, 급코너(peak≫near)에서만 추가 감속. 곡률캡(LFD)이 이미 컷을 막으므로
        #   속도는 과하지 않게 blend(PEAK_SPEED_BLEND). 0=속도에 피크 무시, 1=완전 반영.
        near_for_speed = near_demand_deg + self.PEAK_SPEED_BLEND * max(0.0, near_peak_deg - near_demand_deg)
        v_curve = demand_to_speed(near_for_speed)
        reason = "MAX" if v_curve >= self.max_speed_ms - 0.02 else "CURVE"

        # [v6.7] ω_n 상한 선택(예측 활성 여부에 따라 자동 보수화)
        omega_n_max = self.OMEGA_N_MAX if (tau > 1e-3 and imu_fresh_pred) else self.OMEGA_N_MAX_NO_IMU

        # (b) 다가오는 코너 제동거리 게이팅
        #   [v6.7.3] 원거리도 급코너 피크를 부분 반영(far_for_gate) → 다가오는 R≈2m
        #   코너를 3m창이 과소평가해 '늦게·덜' 감속하던 것 교정. 곡률캡이 컷을 막으므로
        #   속도는 부분만(PEAK_SPEED_BLEND) 낮춰 정상코너 순항 보존.
        far_for_gate = far_demand_deg + self.PEAK_SPEED_BLEND * max(0.0, far_peak_deg - far_demand_deg)
        # [v6.7.3 감사수정] 게이트 거리: 급피크가 windowed 최대점과 다른(더 가까운) 코너면
        #   그 거리 기준으로 제동 — far_dist가 완만한 다른 코너를 가리켜 제동이 늦는 것 방지.
        gate_dist = far_dist
        if far_peak_dist != float('inf') and far_peak > far_demand_deg + 1.0:
            gate_dist = min(gate_dist, far_peak_dist)
        v_target = v_curve
        if gate_dist != float('inf') and far_for_gate > near_demand_deg + 1.0:
            v_corner = demand_to_speed(far_for_gate)
            if far_peak_deg > 2.5:
                # [v6.7] 코너 도착 시점의 LFD(곡률캡, 피크기준)와 ω_n 정합 속도로 '미리' 감속
                #   → 진입 순간 LFD 급감·속도 잔존으로 ω_n 튀는 것(실측 1.16~1.32) 예방
                R_far = self.vehicle_length / math.tan(math.radians(min(far_peak_deg, 25.0)))
                cap_far = min(self.max_lfd, max(self.min_lfd,
                                                math.sqrt(self.LFD_CURVE_CAP_K * R_far)))
                v_corner = min(v_corner, omega_n_max * cap_far / 1.41421356)
            brake_d = max(0.0, gate_dist - self.BRAKE_GATE_MARGIN)
            v_brake = math.sqrt(max(0.0, v_corner*v_corner + 2.0 * self.BRAKE_GATE_DECEL * brake_d))
            if v_brake < v_target:
                v_target = v_brake
                reason = "BRAKE"

        # (b1.5) [v6.7.2] 슬라럼 게이트-홀드: 급한 원거리 게이트가 가까우면 근거리
        #   곡률이 잠깐 풀려도 재가속 금지(게이트간 펄스 → 다음 게이트 오버슈트 차단).
        #   [v6.7.3] 피크 기준 + 거리 확대(연속 게이트 사이 far_dist가 잠깐 벌어지는
        #   것까지 포섭). 실측 t=29.5s far_dist 3→9m 튐으로 홀드가 풀려 재가속했던 것 차단.
        if (gate_dist != float('inf') and far_peak_deg > self.GATE_HOLD_DEMAND_DEG
                and gate_dist < self.GATE_HOLD_DIST_M):
            v_hold = max(self.GATE_HOLD_SPEED, demand_to_speed(far_peak_deg))
            if v_hold < v_target:
                v_target = v_hold
                reason = "BRAKE"

        # (b2) [v6.7] ω_n 결속 — [v6.7.1 조건화] '캡/슬루가 LFD를 테이블 설계값 아래로
        #   누르고 있을 때만' 발동. [v6.7.3] 기준을 '지속 곡률(windowed) LFD'로 변경:
        #   피크(짧은 필렛)로 인한 순간 단축까지 결속하면 매 코너 과잉감속(OMEGA 36%)
        #   → 피크 통과는 예측+곡률캡이 감당, 결속은 지속 코너에서만.
        if not approach_active and lfd_win_only < (lfd_speed - 0.05):
            v_wn = omega_n_max * lfd_win_only / 1.41421356
            if v_wn < v_target:
                v_target = v_wn
                reason = "OMEGA"

        # (c) CTE 회복 감속
        if cte_abs > self.CTE_RECOVER_START:
            cte_cap = self.max_speed_ms
            for thr, cap in self.CTE_RECOVER_CAPS:
                if cte_abs >= thr:
                    cte_cap = cap
            if cte_cap < v_target:
                v_target = cte_cap
                reason = "CTE_RECOVERY"

        v_target = max(self.min_speed_ms, min(self.max_speed_ms, v_target))

        # DR 감속
        if self.dr_active:
            v_target = max(self.safe_stop_speed, v_target * self.dr_speed_factor)
            reason = "DR"

        # ── [CAM] 6·7번째 속도캡: 적신호 정지선 제동 + DR 중속 절대상한 ──
        #   cam_hold=True(적신호 정지 유지) 면 아래 normal_floor(최저속 하한)를 해제.
        cam_hold = False
        if self.camfu is not None:
            v_target, reason, cam_hold = self.camfu.apply_speed_caps(
                v_target, reason, self.dr_active)
            if cam_hold:
                self.spd_integral = 0.0   # 정지 유지 중 속도적분 windup 방지

        # 속도 PI(미세보정)
        spd_err = v_target - v_now
        if v_now / max(v_target, 0.01) < 0.30:
            self.spd_integral += spd_err * 0.1
        else:
            self.spd_integral += spd_err
        self.spd_integral = max(-self.MAX_SPD_INTEG, min(self.MAX_SPD_INTEG, self.spd_integral))
        spd_corr = self.spd_kp * spd_err + self.spd_ki * self.spd_integral
        pre_pi = v_target
        v_target = v_target + (spd_corr * 0.22 if spd_corr >= 0 else spd_corr * 0.06)
        v_target = max(self.safe_stop_speed, v_target)
        if v_target < pre_pi - 0.12:
            reason = "SPEED_PI"

        # 출발 점진 가속
        if self.ramp_up_active:
            elapsed = time.time() - self.drive_start_time
            ramp_ratio = min(1.0, elapsed / self.RAMP_UP_DURATION)
            ramp_max = self.max_speed_ms * ramp_ratio
            if ramp_max < v_target:
                reason = "RAMP"
            v_target = min(v_target, max(self.safe_stop_speed, ramp_max))
            if ramp_ratio >= 1.0:
                self.ramp_up_active = False
                self.publish_event("✅ 점진 가속 완료 → 정상 속도")

        # 정상주행 하한(접근/DR/램프 제외)
        normal_floor = (not self.ramp_up_active and not self.dr_active
                        and not approach_active and not cam_hold)   # [CAM] 적신호 정지 시 하한 해제
        if normal_floor:
            v_target = max(self.min_speed_ms, v_target)

        # 속도 슬루레이트 (가속 부드럽게 / 감속 빠르게)
        max_accel = self.MAX_ACCEL_PER_S * loop_dt
        decel_rate = self.CORNER_DECEL_PER_S if (reason in ("BRAKE", "CTE_RECOVERY")) else self.MAX_DECEL_PER_S
        if approach_active:
            decel_rate = max(decel_rate, 5.0)
        max_decel = decel_rate * loop_dt
        prev_v = abs(self.last_published_speed)
        if v_target > prev_v + max_accel:
            v_target = prev_v + max_accel
            if reason == "MAX":
                reason = "ACCEL_LIMIT"
        elif prev_v - v_target > max_decel:
            v_target = max(self.safe_stop_speed, prev_v - max_decel)
            reason = "DECEL_LIMIT"

        if normal_floor:
            v_target = max(self.min_speed_ms, v_target)
        v_target = max(self.safe_stop_speed, v_target)

        # APPROACH 캡 최종 적용
        if approach_active and approach_speed_cap < v_target:
            v_target = max(self.safe_stop_speed, approach_speed_cap)
            reason = "APPROACH"

        signed_spd = v_target * cur_dir

        # ─────────────────────────────────────────────────────────────────
        # 제어 1. Pure Pursuit (lookahead 타깃 추종)
        # ─────────────────────────────────────────────────────────────────
        pp_idx = min(self.wp_idx + 1, len(self.waypoints) - 1)
        for i in range(self.wp_idx, len(self.waypoints)):
            wp = self.waypoints[i]
            dx = math.radians(wp['lon']-self.ctrl_lon)*math.cos(math.radians(self.ctrl_lat))*R   # [v6.7] 예측좌표
            dy = math.radians(wp['lat']-self.ctrl_lat)*R
            if is_rev: dx, dy = -dx, -dy
            lx = dx*math.cos(psi_eff) + dy*math.sin(psi_eff)
            if math.hypot(dx, dy) >= dynamic_lfd and lx > 0:
                pp_idx = i
                break

        # 종점 8m 이내: 최종 진행선 위 가상타깃으로 직진 정렬
        if approach_active and d2dest <= self.FINAL_HEADING_ALIGN_DIST and not is_rev:
            tgt_lat, tgt_lon = self.final_line_target_latlon(lookahead_m=dynamic_lfd, overshoot_max_m=1.0)
        else:
            pp_wp = self.waypoints[pp_idx]
            tgt_lat, tgt_lon = pp_wp['lat'], pp_wp['lon']

        dx = math.radians(tgt_lon-self.ctrl_lon)*math.cos(math.radians(self.ctrl_lat))*R   # [v6.7] 예측좌표
        dy = math.radians(tgt_lat-self.ctrl_lat)*R
        if is_rev: dx, dy = -dx, -dy
        lx = dx*math.cos(psi_eff) + dy*math.sin(psi_eff)
        ly = -dx*math.sin(psi_eff) + dy*math.cos(psi_eff)

        if lx <= 0.1:
            # 타깃이 뒤/옆 → fallback (급변 방지: 최소 분모 + 클램프)
            fb_idx = min(self.wp_idx + 1, len(self.waypoints) - 1)
            fb = self.waypoints[fb_idx]
            dxf = math.radians(fb['lon']-self.ctrl_lon)*math.cos(math.radians(self.ctrl_lat))*R   # [v6.7]
            dyf = math.radians(fb['lat']-self.ctrl_lat)*R
            if is_rev: dxf, dyf = -dxf, -dyf
            lxf = dxf*math.cos(psi_eff) + dyf*math.sin(psi_eff)
            lyf = -dxf*math.sin(psi_eff) + dyf*math.cos(psi_eff)
            if lxf > 0.05:
                alpha = math.atan2(lyf, lxf)
                fb_dist = max(dynamic_lfd * 0.5, math.hypot(dxf, dyf))
                pp_steer = math.degrees(math.atan2(2.0*self.vehicle_length*math.sin(alpha), fb_dist))
                pp_steer = max(-self.STEER_MAX_DEG, min(self.STEER_MAX_DEG, pp_steer))
            else:
                pp_steer = 0.0
        else:
            alpha = math.atan2(ly, lx)
            pp_steer = math.degrees(math.atan2(2.0*self.vehicle_length*math.sin(alpha), dynamic_lfd))

        # ─────────────────────────────────────────────────────────────────
        # 제어 2. PID (게인 테이블, PP 보조 + 정상상태 오차 제거)
        # ─────────────────────────────────────────────────────────────────
        curr_kp, curr_ki, curr_kd = self.lookup_gains(self.v_sched)   # [v6.7] 저역 속도(게인 변조 차단)
        active_deadband = self.CTE_DEADBAND_STRAIGHT if is_straight else self.CTE_DEADBAND

        # [v6.6] 부호반전 시 '완전 리셋' → '소프트 감쇠(×0.35)'.
        #   근거: 완전0으로 지우면 좌우 왕복마다 적분이 리셋돼 실제 쏠림(bias)을 못 쌓음.
        #   ×0.35로 줄이면 대칭 왕복은 0 근처로 수렴하되 한쪽 쏠림은 누적 유지 → 쏠림 타이트.
        #   (windup은 아래 클램프+과대시 감쇠로 계속 방지)
        if (cte * self.prev_cte) < 0 and cte_abs > active_deadband and abs(self.prev_cte) > active_deadband:
            self.cte_integral *= 0.35

        if cte_abs < active_deadband:
            self.cte_integral *= 0.80
        else:
            self.cte_integral += cte * loop_dt
        self.cte_integral = max(-self.Ki_CLAMP, min(self.Ki_CLAMP, self.cte_integral))

        now = time.time()
        if now - self.last_integral_decay > self.integral_decay_interval:
            if abs(self.cte_integral) > self.Ki_CLAMP * 0.90:
                self.cte_integral *= 0.85
            self.last_integral_decay = now

        d_cte = cte - self.prev_cte
        self.prev_cte = cte
        d_cte_per_sec = d_cte / max(1e-3, loop_dt)
        self.d_term_lpf += self.D_TERM_LPF_ALPHA * (d_cte_per_sec - self.d_term_lpf)

        p_term = curr_kp * cte_ctrl                 # [v6.5] 지연보상된 CTE로 P 제어(위상 앞섬→왕복↓)
        i_term = curr_ki * self.cte_integral
        d_term = curr_kd * self.d_term_lpf * 0.20   # [v6.5] 수치 D는 백업으로 축소(lead가 주 감쇠)
        pid_steer = max(-self.PID_STEER_LIMIT, min(self.PID_STEER_LIMIT, p_term + i_term + d_term))

        raw_steer = pp_steer + pid_steer

        # ─────────────────────────────────────────────────────────────────
        # 제어 3. 직선 잔떨림 억제 (속도는 이미 곡률에만 반응 → 여기선 조향만)
        # ─────────────────────────────────────────────────────────────────
        if (is_straight and not approach_active and cte_abs < self.JITTER_CTE_M
                and abs(raw_steer) < self.JITTER_STEER_DEG):
            if abs(raw_steer) < self.JITTER_ZERO_DEG:
                raw_steer = 0.0
            else:
                raw_steer = math.copysign((abs(raw_steer) - 0.35) * 0.72, raw_steer)

        raw_steer = max(-self.STEER_MAX_DEG - 4.0, min(self.STEER_MAX_DEG + 4.0, raw_steer))

        # ─────────────────────────────────────────────────────────────────
        # 제어 4. 조향 슬루레이트 + 클램프
        # ─────────────────────────────────────────────────────────────────
        speed_ratio = max(0.0, min(1.0, (v_now - self.min_speed_ms) / max(1e-6, self.max_speed_ms - self.min_speed_ms)))
        smoothed = raw_steer * self.STEER_SMOOTH_ALPHA + self.prev_steer * (1.0 - self.STEER_SMOOTH_ALPHA)
        step_limit = self.STEER_STEP_LOW_DEG + (self.STEER_STEP_HIGH_DEG - self.STEER_STEP_LOW_DEG) * speed_ratio
        if cte_abs > 0.55 or abs(raw_steer) > 12.0:
            step_limit = max(step_limit, self.STEER_STEP_RECOVER_DEG)
        delta = smoothed - self.prev_steer
        if delta > step_limit:
            smoothed = self.prev_steer + step_limit
        elif delta < -step_limit:
            smoothed = self.prev_steer - step_limit
        final_steer = smoothed
        self.prev_steer = final_steer
        clamped_steer = max(-self.STEER_MAX_DEG, min(self.STEER_MAX_DEG, final_steer))

        # [v6.6] 저속 조향 권한 램프: 정지~저속에선 조향이 물리적 효과가 없고, 정지 중
        #   큰 조향을 유지하면 출발 순간 덜컥거림(로스백 startup서 정지중 -20° 유지 확인).
        #   속도로 부드럽게 권한 부여 → 출발 시 조향이 서서히 실려 오프셋을 매끄럽게 복귀.
        #   0.5m/s↑ 정상주행은 full(주행 중 min_speed=1.0이라 무영향). 출발/최종크립만 완화.
        steer_authority = min(1.0, max(0.15, v_now / self.STEER_AUTHORITY_FULL_SPEED))
        clamped_steer *= steer_authority

        # ─────────────────────────────────────────────────────────────────
        # 명령 발행
        # ─────────────────────────────────────────────────────────────────
        self.publish_cmd(signed_spd, clamped_steer)
        if not getattr(self, '_dispatch_logged', False):   # [v6.3.1] 첫 하달 확인
            self._dispatch_logged = True
            self.get_logger().info(
                f"✅ 주행 하달 개시: /cmd_vel_raw 발행 시작 "
                f"(속도 {signed_spd:+.2f}m/s, 조향 {clamped_steer:+.1f}°, WP {self.wp_idx}/{wp_total})")

        # 디버그(20Hz) — 필드 위치는 기존 모니터/분석 호환 유지
        self._publish_debug([
            self.wp_idx, wp_total, d2dest, cte, cte_raw,
            self.cte_integral, dynamic_lfd, speed_ratio, raw_steer, clamped_steer,
            step_limit, v_target, signed_spd, self.current_speed_ms, near_demand_deg,
            (far_dist if far_dist != float('inf') else 99.0), far_demand_deg, p_term, i_term, d_term,
            self.d_term_lpf, loop_dt, 1.0 if is_rev else 0.0, near_peak_deg, 1.0 if approach_active else 0.0,
            SPD_REASON_CODE.get(reason, -1), far_peak_deg,   # [v6.7.3] 23=near피크, 26=far피크
        ])

        # 상태(1Hz)
        self._publish_status_if_due(wp_total, d2dest, signed_spd, reason, cte, is_rev, far_demand_deg, far_dist)

        # ── [CAM] 융합 디버그 20Hz 발행 (/fusion_debug — 로스백 검증 1차 소스) ──
        if self.camfu is not None:
            self.camfu.publish_debug(loop_dt)


def main(args=None):
    rclpy.init(args=args)
    node = DrivingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.instant_stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()