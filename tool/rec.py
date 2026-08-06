#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
rosbag_extractor.py ― rosbag2 추출기 [카메라 융합 검증판, rec.py 후속]
─────────────────────────────────────────────────────────────────
rec.py 대비 변경:
  1. [버그수정] /ego_state 경로: 'x','yaw' 등 존재하지 않는 필드 → data[2..5] 로 교정
  2. [지뢰제거] Image/CompressedImage 토픽은 디코딩 자체를 건너뜀
     (/lane/debug 가 bag 에 섞여 있어도 JSON 폭발 방지)
  3. [신규] /fusion_debug 18필드 · /lane_metrics 10필드 이름 매핑 CSV 생성
  4. [확장] merged_timeline 에 카메라·융합·헤딩 신호 추가

생성물:  extracted_<bag>/
  ├─ <topic>.jsonl / <topic>.csv
  ├─ driving_debug_named.csv   (27필드)
  ├─ fusion_debug_named.csv    (18필드)  ← 융합 검증 1차 소스
  ├─ lane_metrics_named.csv    (10필드)
  ├─ merged_timeline.csv
  └─ _summary.txt              (토픽별 Hz — /lane/state 실측 Hz 는 여기서 확인)

실행:  python3 rosbag_extractor.py
"""

import os
import glob
import json
import csv
from collections import OrderedDict, defaultdict

from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from rosidl_runtime_py import message_to_ordereddict


# ═══════════════════════════════════════════════════════════════
#  이름 매핑 (각 노드의 발행 순서와 반드시 일치)
# ═══════════════════════════════════════════════════════════════
DRIVING_DEBUG_FIELDS = [
    'wp_idx', 'wp_total', 'd2dest', 'cte', 'cte_raw', 'cte_integral',
    'dynamic_lfd', 'speed_ratio', 'raw_steer', 'clamped_steer',
    'steer_step_limit', 'target_spd', 'signed_spd', 'current_speed',
    'route_turn_deg', 'dist_to_curve', 'ahead_steer', 'p_term', 'i_term',
    'd_term', 'd_term_lpf', 'loop_dt', 'is_rev', 'hard_corner',
    'finish_progress', 'spd_reason_code', 'finish_mode_code',
]

# camera_fusion.py 의 FD_* 인덱스와 1:1
FUSION_DEBUG_FIELDS = [
    'cam_ok',          # 0  이번 루프 카메라 계측 사용가능(신선+게이트)
    'cam_cte_rear',    # 1  뒷차축 투영 카메라 CTE [m, 좌+]
    'cam_cte_near',    # 2  근점 CTE [m]
    'theta_lane_deg',  # 3  차선 대비 헤딩오차 [°, CCW+]
    'cam_conf',        # 4  bridge 유효 conf
    'cam_age_s',       # 5  계측 나이 [s]
    'gps_cte',         # 6  융합 전 GPS CTE [m]
    'cte_fused',       # 7  융합 후 CTE [m] (PID 실입력)
    'w_applied',       # 8  적용 가중치
    'w_target',        # 9  목표 가중치
    'dr_active',       # 10
    'tl_code',         # 11 -1 UNK / 0 GREEN / 1 RED_FAR / 2 RED
    'stop_line_d',     # 12 [m], 미검출 -1
    'cap_code',        # 13 0없음/1 TL_BRAKE/2 TL_HOLD/3 DR_MID/4 DR_LANE_LOST
    'v_cap',           # 14 캡 후 목표속도
    'head_ref_deg',    # 15 발행한 절대헤딩 참조(미발행 999)
    'head_ref_qual',   # 16
    'loop_dt',         # 17
]

# lane_camera_bridge.py 발행 순서와 1:1
LANE_METRICS_FIELDS = [
    'cte_rear_m', 'cte_near_m', 'theta_lane_deg', 'curvature_1pm',
    'conf_eff', 'lane_width_m', 'flags', 'd_near_m', 'conf_raw', 'seq',
]

# 통합 타임라인 (topic, [(열이름, 경로)])
MERGED_TIMELINE_SPEC = [
    ('/cmd_vel_raw', [('cmd_v_ms', 'linear.x'), ('cmd_steer', 'angular.z')]),
    ('/encoder',     [('enc_tick', 'data')]),
    ('/ego_state',   [                       # Float64MultiArray → data[i]
        ('ego_x',   'data[2]'), ('ego_y',   'data[3]'),
        ('ego_yaw', 'data[4]'), ('ego_v',   'data[5]'),
    ]),
    ('/heading',     [('head_locked', 'data[0]'),
                      ('head_now',    'data[1]'),
                      ('head_drift',  'data[2]')]),
    ('/driving_debug', [
        ('cte',           'data[3]'),  ('clamped_steer', 'data[9]'),
        ('target_spd',    'data[11]'), ('current_speed', 'data[13]'),
        ('p_term',        'data[17]'), ('d_term_lpf',    'data[20]'),
    ]),
    ('/fusion_debug', [
        ('fz_cam_cte',   'data[1]'),  ('fz_gps_cte',  'data[6]'),
        ('fz_cte_fused', 'data[7]'),  ('fz_w',        'data[8]'),
        ('fz_dr',        'data[10]'), ('fz_cap_code', 'data[13]'),
        ('fz_head_ref',  'data[15]'),
    ]),
    ('/lane_metrics', [('lm_theta_deg', 'data[2]'),
                       ('lm_conf',      'data[4]'),
                       ('lm_width_m',   'data[5]'),
                       ('lm_flags',     'data[6]')]),
    ('/tl/state',       [('tl_state', 'data')]),
    ('/stop_line_dist', [('stop_d',   'data')]),
    ('/gps_status',     [('gps_status', 'data')]),
]

# 디코딩 자체를 건너뛸 타입(용량 폭발 방지)
SKIP_TYPE_SUBSTR = ('sensor_msgs/msg/Image', 'sensor_msgs/msg/CompressedImage')


# ═══════════════════════════════════════════════════════════════
#  유틸 (rec.py 와 동일)
# ═══════════════════════════════════════════════════════════════
def get_latest_bag_dir(base_path='.'):
    bag_dirs = [d for d in glob.glob(os.path.join(base_path, 'rosbag2_*'))
                if os.path.isdir(d)]
    if not bag_dirs:
        return None
    bag_dirs.sort(key=os.path.getmtime, reverse=True)
    return bag_dirs[0]


def flatten_dict(d, parent_key='', sep='.'):
    items = []
    if isinstance(d, dict):
        for k, v in d.items():
            new_key = f'{parent_key}{sep}{k}' if parent_key else k
            items.extend(flatten_dict(v, new_key, sep).items())
    elif isinstance(d, (list, tuple)):
        if len(d) > 40:
            items.append((f'{parent_key}_len', len(d)))
        else:
            for i, v in enumerate(d):
                items.extend(flatten_dict(v, f'{parent_key}[{i}]', sep).items())
    else:
        items.append((parent_key, d))
    return dict(items)


def get_by_path(msg_dict, path):
    cur = msg_dict
    try:
        tokens, buf, i = [], '', 0
        while i < len(path):
            c = path[i]
            if c == '.':
                if buf:
                    tokens.append(buf); buf = ''
            elif c == '[':
                if buf:
                    tokens.append(buf); buf = ''
                j = path.index(']', i)
                tokens.append(int(path[i + 1:j]))
                i = j
            else:
                buf += c
            i += 1
        if buf:
            tokens.append(buf)
        for t in tokens:
            cur = cur[t]
        return cur
    except (KeyError, IndexError, TypeError):
        return None


def write_named_csv(out_dir, fname, fields, rows):
    if not rows:
        return
    path = os.path.join(out_dir, fname)
    with open(path, 'w', encoding='utf-8', newline='') as f:
        w = csv.DictWriter(f, fieldnames=['t_sec', 't_rel'] + fields)
        w.writeheader()
        w.writerows(rows)
    print(f'📊 {fname} ({len(rows)}행)')


# ═══════════════════════════════════════════════════════════════
#  메인
# ═══════════════════════════════════════════════════════════════
def extract_latest_bag():
    latest_bag = get_latest_bag_dir()
    if not latest_bag:
        print("❌ 현재 폴더에 'rosbag2_'로 시작하는 기록 폴더가 없습니다.")
        return
    print(f"✅ 최근 기록 발견: {latest_bag}")

    output_dir = f"extracted_{os.path.basename(latest_bag)}"
    os.makedirs(output_dir, exist_ok=True)

    reader = SequentialReader()
    reader.open(StorageOptions(uri=latest_bag, storage_id='sqlite3'),
                ConverterOptions(input_serialization_format='cdr',
                                 output_serialization_format='cdr'))
    type_map = {t.name: t.type for t in reader.get_all_topics_and_types()}

    jsonl_handles  = {}
    rows_per_topic = defaultdict(list)
    msg_count      = defaultdict(int)
    t_first        = defaultdict(lambda: None)
    t_last         = defaultdict(lambda: None)
    skipped_img    = defaultdict(int)

    merged_rows = []
    merged_target_topics = {t for t, _ in MERGED_TIMELINE_SPEC}
    named_rows = {'/driving_debug': [], '/fusion_debug': [], '/lane_metrics': []}
    NAMED_SPEC = {'/driving_debug': ('driving_debug_named.csv', DRIVING_DEBUG_FIELDS),
                  '/fusion_debug':  ('fusion_debug_named.csv',  FUSION_DEBUG_FIELDS),
                  '/lane_metrics':  ('lane_metrics_named.csv',  LANE_METRICS_FIELDS)}

    bag_t0, total = None, 0
    print(f"📂 '{output_dir}' 폴더에 추출 중...")

    while reader.has_next():
        topic, data, timestamp = reader.read_next()
        if bag_t0 is None:
            bag_t0 = timestamp
        t_sec = timestamp * 1e-9
        t_rel = (timestamp - bag_t0) * 1e-9

        ttype = type_map.get(topic, '')
        if any(s in ttype for s in SKIP_TYPE_SUBSTR):
            skipped_img[topic] += 1        # 이미지 계열: 통계만, 디코딩 생략
            msg_count[topic] += 1
            if t_first[topic] is None:
                t_first[topic] = t_sec
            t_last[topic] = t_sec
            total += 1
            continue

        try:
            msg = deserialize_message(data, get_message(ttype))
            msg_dict = message_to_ordereddict(msg)
        except Exception as e:
            print(f"⚠️ 디코딩 실패: {topic} ({e})")
            continue

        if topic not in jsonl_handles:
            safe = topic.replace('/', '_').strip('_')
            jsonl_handles[topic] = open(
                os.path.join(output_dir, f'{safe}.jsonl'), 'w', encoding='utf-8')
        jsonl_handles[topic].write(json.dumps(
            {'timestamp': timestamp, 't_sec': t_sec, 't_rel': t_rel,
             'data': msg_dict}, ensure_ascii=False) + '\n')

        row = OrderedDict(t_sec=t_sec, t_rel=t_rel)
        row.update(flatten_dict(msg_dict))
        rows_per_topic[topic].append(row)

        if topic in NAMED_SPEC:
            arr = msg_dict.get('data', [])
            fields = NAMED_SPEC[topic][1]
            named = OrderedDict(t_sec=t_sec, t_rel=t_rel)
            for i, name in enumerate(fields):
                named[name] = arr[i] if i < len(arr) else None
            named_rows[topic].append(named)

        if topic in merged_target_topics:
            mrow = OrderedDict(t_sec=t_sec, t_rel=t_rel, source_topic=topic)
            for ttopic, fields in MERGED_TIMELINE_SPEC:
                if ttopic != topic:
                    continue
                for col, path in fields:
                    mrow[col] = get_by_path(msg_dict, path)
            merged_rows.append(mrow)

        msg_count[topic] += 1
        if t_first[topic] is None:
            t_first[topic] = t_sec
        t_last[topic] = t_sec
        total += 1
        if total % 2000 == 0:
            print(f'... {total}개 처리 중 ...')

    for fh in jsonl_handles.values():
        fh.close()

    # 토픽별 CSV
    for topic, rows in rows_per_topic.items():
        if not rows:
            continue
        safe = topic.replace('/', '_').strip('_')
        all_keys, seen = ['t_sec', 't_rel'], {'t_sec', 't_rel'}
        for r in rows:
            for k in r.keys():
                if k not in seen:
                    all_keys.append(k); seen.add(k)
        with open(os.path.join(output_dir, f'{safe}.csv'),
                  'w', encoding='utf-8', newline='') as f:
            w = csv.DictWriter(f, fieldnames=all_keys)
            w.writeheader()
            for r in rows:
                w.writerow({k: r.get(k, '') for k in all_keys})

    for topic, (fname, fields) in NAMED_SPEC.items():
        write_named_csv(output_dir, fname, fields, named_rows[topic])

    if merged_rows:
        all_keys, seen = ['t_sec', 't_rel', 'source_topic'], {'t_sec', 't_rel', 'source_topic'}
        for r in merged_rows:
            for k in r.keys():
                if k not in seen:
                    all_keys.append(k); seen.add(k)
        merged_rows.sort(key=lambda x: x['t_sec'])
        with open(os.path.join(output_dir, 'merged_timeline.csv'),
                  'w', encoding='utf-8', newline='') as f:
            w = csv.DictWriter(f, fieldnames=all_keys)
            w.writeheader()
            for r in merged_rows:
                w.writerow({k: r.get(k, '') for k in all_keys})
        print(f'📈 merged_timeline.csv ({len(merged_rows)}행)')

    with open(os.path.join(output_dir, '_summary.txt'), 'w', encoding='utf-8') as f:
        f.write(f'rosbag: {latest_bag}\n총 메시지: {total}\n\n토픽별 통계:\n')
        f.write(f'{"topic":40s} {"count":>8s} {"dur_s":>8s} {"hz":>7s} {"type":s}\n')
        f.write('-' * 100 + '\n')
        for topic in sorted(msg_count.keys()):
            cnt = msg_count[topic]
            dur = (t_last[topic] - t_first[topic]) if cnt > 1 else 0.0
            hz = (cnt - 1) / dur if dur > 0 else 0.0
            note = '  [image: 디코딩 생략]' if topic in skipped_img else ''
            f.write(f'{topic:40s} {cnt:>8d} {dur:>8.2f} {hz:>7.2f} '
                    f'{type_map.get(topic, "?")}{note}\n')
    print('📝 _summary.txt 작성 완료')
    print(f'\n🎉 완료! 총 {total}개 메시지 → {output_dir}/')


if __name__ == '__main__':
    extract_latest_bag()