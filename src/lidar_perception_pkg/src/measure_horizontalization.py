#!/usr/bin/env python3
"""Read-only live diagnostic. Writes paired CSV/JSON/HTML, never control or TF."""
import os
os.environ['OPENBLAS_NUM_THREADS'] = '1'
import argparse
import csv
import html
import json
import threading
import time
from collections import Counter
from pathlib import Path

import numpy as np
import yaml
from lidar_perception_pkg.horizontalization_metrics import paired_metrics, distribution


def render_report(output,summary,rows,example):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig,axes=plt.subplots(2,1,figsize=(10,7),sharex=True)
    for label,color in [('before','tab:orange'),('after','tab:blue')]:
        axes[0].plot([r['time_s'] for r in rows],[r[label+'_tilt_deg'] for r in rows],label=label,color=color)
        axes[1].plot([r['time_s'] for r in rows],[r[label+'_horizontal_height_rmse_m']*100 for r in rows],label=label,color=color)
    axes[0].set_ylabel('Ground-candidate tilt (deg)')
    axes[1].set_ylabel('Height spread RMS (cm)');axes[1].set_xlabel('Scan time (s)')
    for ax in axes:
        ax.grid(alpha=.25);ax.legend()
    fig.suptitle('Same raw-selected points before / after the recorded rotation')
    fig.tight_layout(rect=[0,0,1,.94]);fig.savefig(output/'metrics.png',dpi=150);plt.close(fig)
    if example is not None:
        before,after=example
        fig,axes=plt.subplots(1,2,figsize=(10,4),sharey=True)
        for i,axis in enumerate(('X','Y')):
            axes[i].scatter(before[:,i],before[:,2],s=2,label='before',color='tab:orange',alpha=.4)
            axes[i].scatter(after[:,i],after[:,2],s=2,label='after',color='tab:blue',alpha=.4)
            axes[i].set_xlabel(axis+' (m)');axes[i].grid(alpha=.25);axes[i].legend()
        axes[0].set_ylabel('Z (m)');fig.suptitle('First accepted ground patch (same returns)')
        fig.tight_layout(rect=[0,0,1,.92]);fig.savefig(output/'ground_patch.png',dpi=150);plt.close(fig)
    metrics=summary['metrics']
    def value(key,scale=1):
        v=metrics.get(key)
        return 'N/A' if v is None else '{:.4f}'.format(v['median']*scale)
    table=''.join('<tr><td>{}</td><td>{}</td><td>{}</td></tr>'.format(label,value('before_'+key,scale),value('after_'+key,scale)) for label,key,scale in [
        ('수평 기준 기울기 (°)','tilt_deg',1),('전방 방향 경사 (°)','forward_slope_deg',1),
        ('좌우 방향 경사 (°)','lateral_slope_deg',1),('수평 높이 편차 RMS (cm)','horizontal_height_rmse_m',100),
        ('최적 평면 잔차 RMS (cm)','orthogonal_plane_rmse_m',100)])
    coverage=summary['coverage']
    coverage_text='수집 스캔: {}개\n유효 관측: {}개\n실제 보정행렬 대응: {}개\n지면 지표 산출: {}개\n관측 누락: {}개 / 무효 관측: {}개'.format(
        coverage['raw_scans'],coverage['valid_observations'],coverage['matched_applied_rotations'],coverage['ground_metric_scans'],coverage['missing_observations'],coverage['invalid_observations'])
    latency=summary['all_valid_observation_latency_ms']
    if latency:coverage_text+='\n관측 수신 지연: 중앙값 {:.1f} ms / p95 {:.1f} ms'.format(latency['median'],latency['p95'])
    report='''<!doctype html><html lang="ko"><meta charset="utf-8"><title>LiDAR 수평화 측정</title>
<style>body{font-family:sans-serif;max-width:1000px;margin:36px auto;padding:0 20px;color:#192838;background:#f7f9fc}table{border-collapse:collapse;width:100%;background:white}td,th{padding:12px;border-bottom:1px solid #ddd;text-align:left}img{max-width:100%}pre{white-space:pre-wrap;background:#edf1f6;padding:15px}a{color:#185bb0}</style>
<h1>LiDAR 수평화 측정 결과</h1><p>검출기가 실제 사용한 회전행렬과 같은 시각의 원본 점군을 대응해 측정했습니다. 아래 전후 값은 유효한 지면 후보 스캔별 지표의 중앙값입니다.</p>
<p><b>독립적인 평지 기준이 없습니다.</b> 잔류 기울기는 실제 도로 경사와 센서 정렬 오차를 포함합니다. 보정 정확도나 객체 인식 성공률로 해석하지 않습니다.</p>
<table><tr><th>지표</th><th>보정 전</th><th>보정 후</th></tr>'''+table+'''</table>
<h2>수신·처리 품질</h2><pre>'''+html.escape(coverage_text)+'''</pre>
<p>최적 평면 잔차는 강체 회전으로 줄어들지 않는 것이 정상입니다. 수평 높이 편차는 같은 점들의 z 표준편차이며 거리 노이즈 지표가 아닙니다. 형상 보존 오차는 기록된 행렬의 수학적 검증으로, 실제 도로 기준 정확도를 증명하지 않습니다.</p>
<img src="metrics.png" alt="시간별 기울기와 높이 편차">'''
    if example is not None:report+='<img src="ground_patch.png" alt="첫 유효 지면 후보 점군 비교">'
    report+='<p><a href="summary.json">전체 지표·제외 사유·설정 JSON</a> · <a href="frames.csv">스캔별 CSV</a></p></html>'
    (output/'report.html').write_text(report,encoding='utf-8')


def main():
    import rospy
    import rospkg
    from sensor_msgs.msg import PointCloud2
    from sensor_msgs.point_cloud2 import read_points
    from std_msgs.msg import String
    from common_msgs_pkg.msg import LidarObservationArray
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--duration',type=float,default=20.)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--config',type=Path)
    args=parser.parse_args(rospy.myargv()[1:])
    if not 1<=args.duration<=120:parser.error('duration must be 1..120 seconds')
    output=args.output.expanduser().resolve()
    if output.exists() and any(output.iterdir()):parser.error('output must be a new or empty directory')
    output.mkdir(parents=True,exist_ok=True)
    config_path=args.config or Path(rospkg.RosPack().get_path('lidar_perception_pkg'))/'config/horizontalization_metrics.yaml'
    config=yaml.safe_load(config_path.read_text())
    from lidar_perception_pkg.horizontalization_metrics import validate_config
    validate_config(config)
    rospy.init_node('horizontalization_metrics',argv=['horizontalization_metrics','__ns:=/molit/internal/lidar_perception'],disable_signals=True)
    if rospy.get_param('/use_sim_time',False):raise RuntimeError('This collector requires live ROS wall time, not rosbag replay')
    lock=threading.Lock();raw={};stamps=set();audits={};observations={};stored_bytes=0
    capture=[False]; issues=Counter()
    def cloud(m):
        nonlocal stored_bytes
        with lock:
            if not capture[0]:return
            stamp=m.header.stamp.to_nsec()
            if not stamp or m.header.frame_id!='lidar_link':issues['invalid_raw_header']+=1;return
            if stamp in stamps:issues['duplicate_raw_stamp']+=1;return
            stamps.add(stamp)
            if stored_bytes+len(m.data)>128*1024*1024:issues['raw_storage_limit']+=1;return
            raw[stamp]=m;stored_bytes+=len(m.data)
    def audit(m):
        try:
            data=json.loads(m.data);stamp=int(data['stamp_ns'])
            with lock:
                if capture[0] or stamp in stamps:audits[stamp]=data
        except (ValueError,KeyError,TypeError):
            with lock:issues['invalid_audit_json']+=1
    def observation(m):
        with lock:
            stamp=m.header.stamp.to_nsec()
            if capture[0] or stamp in stamps:
                observations[stamp]=(bool(m.objects_valid),(time.time()-m.header.stamp.to_sec())*1000)
    subs=[rospy.Subscriber('/molit/sensors/lidar/points',PointCloud2,cloud,queue_size=30),
          rospy.Subscriber('/lidar_perception_node/horizontalization_audit',String,audit,queue_size=100),
          rospy.Subscriber('/molit/perception/lidar/observations',LidarObservationArray,observation,queue_size=100)]
    time.sleep(.5)
    with lock:capture[0]=True
    print('Collecting {:.1f} seconds of live paired diagnostics...'.format(args.duration),flush=True)
    time.sleep(args.duration)
    with lock:capture[0]=False
    time.sleep(1.)  # Drain observations/audit for scans at the end of the capture.
    for sub in subs:sub.unregister()
    with lock:
        raw=dict(raw);audits=dict(audits);observations=dict(observations)
    rospy.signal_shutdown('capture complete')
    rows=[];example=None;start=min(stamps) if stamps else 0
    print('Analyzing {} stored scans...'.format(len(raw)),flush=True)
    for stamp,m in sorted(raw.items()):
        if stamp not in observations:issues['no_observation']+=1;continue
        if not observations[stamp][0]:issues['invalid_observation']+=1;continue
        if stamp not in audits:issues['missing_audit']+=1;continue
        data=audits[stamp]
        if data.get('leveling_enabled') is not True:issues['leveling_disabled']+=1;continue
        try:
            xyz=np.asarray(list(read_points(m,field_names=('x','y','z'),skip_nans=True)))
            rotation=np.asarray(data['rotation'],dtype=float).reshape(3,3)
            result,before,after=paired_metrics(xyz,rotation,config)
        except (ValueError,KeyError) as error:issues[str(error)]+=1;continue
        result.update(stamp_ns=str(stamp),time_s=(stamp-start)*1e-9,
                      latency_ms=observations[stamp][1],processing_ms=data['processing_ms'])
        rows.append(result)
        if example is None:
            example=(before,after)
            np.savez_compressed(output/'example_patch.npz',raw=before,corrected=after,rotation=rotation,stamp_ns=np.uint64(stamp))
    valid=sum(observations.get(s,(False,))[0] for s in stamps)
    coverage={'raw_scans':len(stamps),'valid_observations':valid,
              'valid_observation_ratio':valid/len(stamps) if stamps else None,
              'missing_observations':sum(s not in observations for s in stamps),
              'invalid_observations':sum(s in observations and not observations[s][0] for s in stamps),
              'matched_applied_rotations':sum(s in audits and audits[s].get('leveling_enabled') is True for s in stamps),
              'ground_metric_scans':len(rows),'ground_metric_coverage':len(rows)/len(stamps) if stamps else None}
    metric_keys=[k for k in rows[0] if k not in ('stamp_ns','time_s')] if rows else []
    summary={'schema_version':1,'reference':'unknown_terrain_not_ground_truth',
             'source':'actual C++ rotation audit + same-stamp raw returns',
             'duration_s':args.duration,'coverage':coverage,'exclusions':dict(issues),'config':config,
             'metrics':{key:distribution([r[key] for r in rows]) for key in metric_keys},
             'all_valid_observation_latency_ms':distribution([observations[s][1] for s in stamps if s in observations and observations[s][0]]),
             'limitations':['No independent flat-ground reference; no accuracy percentage.',
                            'Patch selection is fixed in raw coordinates; identical returns before and after.',
                            'Plane residual and point ranges are rigid-rotation invariants, not denoising scores.',
                            'Coverage and latency are collector-observed, not UDP packet-loss measurements.']}
    (output/'summary.json').write_text(json.dumps(summary,indent=2,allow_nan=False),encoding='utf-8')
    with (output/'frames.csv').open('w',newline='') as stream:
        writer=csv.DictWriter(stream,fieldnames=list(rows[0]) if rows else ['stamp_ns'])
        writer.writeheader();writer.writerows(rows)
    render_report(output,summary,rows,example)
    print(json.dumps(summary,indent=2,allow_nan=False),flush=True)
    print('Report: '+str(output/'report.html'),flush=True)


if __name__=='__main__':main()
