"""Metrics from independently timestamped simulated truth and controller output."""
from pathlib import Path
import re
import json
import sys
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

HERE = Path(__file__).resolve().parents[2]/'datalog/acados_closed_loop'


def load(name):
    folder = HERE/name
    raw = np.load(folder/'raw.npz')
    s, d, m = (raw[k] for k in ['state','debug','motors'])
    text = (folder/'outer.log').read_text()
    line = next(l for l in text.splitlines() if 'Takeoff + figure-eight activated:' in l)
    start = float(d[d[:,1]==3][0,0])
    duration = float(re.search(r'samples, ([\d.]+) s',line).group(1))
    # Avoid using manual-mode zero timestamps / reference values.
    d = d[d[:,0]>s[0,0]]
    t = s[:,0]-start
    td = d[:,0]-start
    window = (t>=3)&(t<=duration)
    interp = lambda arr, ts, q: np.column_stack([np.interp(q,ts,arr[:,i]) for i in range(arr.shape[1])])
    ref = interp(d[:,2:5], td, t)
    err = s[:,3:6]-ref
    rates = interp(d[:,8:11],td,t)
    norms = np.linalg.norm(err[window],axis=1)
    lag = []
    for axis in range(2):
        scores=[]
        shifts=np.arange(-.30,.401,.0025)
        for shift in shifts:
            r = np.interp(t[window]-shift,td,d[:,2+axis])
            scores.append(np.mean((s[window,3+axis]-r)**2))
        lag.append(float(shifts[np.argmin(scores)]*1000))
    dw=(td>=3)&(td<=duration)
    mw=(m[:,0]>=start+3)&(m[:,0]<=start+duration)
    dt=np.diff(d[dw,0])
    cmd_rate=d[dw,8:11]
    flight = t>=0
    touch = np.where(flight & (s[:,5]<.12))[0]
    tilt = np.degrees(np.arccos(np.clip(1-2*(s[:,13]**2+s[:,14]**2),-1,1)))
    airborne = len(touch)==0
    settled = (t>=duration+3)&(t<=duration+10)
    if not airborne:
        lag = [None,None]  # A crashed trajectory has no meaningful tracking lag.
    else:
        lag = [v if -299<v<399 else None for v in lag]
    metrics=dict(name=name, seed=int(raw['seed']), trajectory_duration_s=duration,
        evaluation_s=[3,duration], position_rmse_m=float(np.sqrt(np.mean(norms**2))),
        xyz_rmse_m=np.sqrt(np.mean(err[window]**2,axis=0)).tolist(),
        position_p95_m=float(np.quantile(norms,.95)),position_max_m=float(norms.max()),
        xy_effective_lag_ms=lag,
        rate_rmse_rad_s=np.sqrt(np.mean((s[window,9:12]-rates[window])**2,axis=0)).tolist(),
        rate_command_rms_rad_s=np.sqrt(np.mean(cmd_rate**2,axis=0)).tolist(),
        rate_command_slew_rms_rad_s2=np.sqrt(np.mean((np.diff(cmd_rate,axis=0)/dt[:,None])**2,axis=0)).tolist(),
        throttle_min=float(m[mw,1:].min()),throttle_max=float(m[mw,1:].max()),
        throttle_rail_fraction=float(np.mean((m[mw,1:]<=0)|(m[mw,1:]>=.9999))),
        outer_interval_ms_percentiles=np.quantile(dt*1000,[.5,.95,.99,1]).tolist(),
        outer_intervals_over_15ms_fraction=float(np.mean(dt>.015)),
        physics_wall_ratio=float((s[-1,1]-s[0,1])/(s[-1,20]-s[0,20])),
        physics_interval_ms_percentiles=np.quantile(s[:,2]*1000,[.5,.95,.99,1]).tolist(),
        min_altitude_m=float(s[flight,5].min()),
        max_tilt_deg=float(tilt[flight].max()),
        first_low_altitude_s=float(t[touch[0]]) if len(touch) else None,
        remained_airborne=airborne,
        post_hover_position_rmse_m=float(np.sqrt(np.mean(np.sum(err[settled]**2,axis=1)))) if settled.any() else None,
        simulation_finished=bool(t[-1]>duration+2), finite=bool(np.isfinite(s).all() and np.isfinite(d).all() and np.isfinite(m).all()))
    if 'solver' in raw and raw['solver'].size:
        q=raw['solver']; qw=(q[:,0]>=start+3)&(q[:,0]<=start+duration)
        metrics['solver_fallback_fraction']=float(np.mean(q[qw,5]))
        metrics['solver_status_counts']={str(int(v)):int(n) for v,n in zip(*np.unique(q[qw,1],return_counts=True))}
        metrics['solver_time_ms_percentiles']=np.quantile(q[qw,3],[.5,.95,.99,1]).tolist()
    (folder/'metrics.json').write_text(json.dumps(metrics,indent=2)+'\n')
    return metrics, (t,s,ref,err,window)


def main():
    import argparse
    parser=argparse.ArgumentParser()
    parser.add_argument('names',nargs='+')
    parser.add_argument('--output-root',type=Path,default=HERE)
    args=parser.parse_args()
    globals()['HERE']=args.output_root
    names=args.names
    results=[]
    fig,ax=plt.subplots(2,2,figsize=(12,8))
    for name in names:
        metrics,(t,s,ref,err,w)=load(name)
        results.append(metrics)
        ax[0,0].plot(s[w,3],s[w,4],label=name,alpha=.8)
        ax[0,1].plot(t[w],np.linalg.norm(err[w],axis=1),label=name)
        ax[1,0].plot(t[w],err[w,0],label=name)
        ax[1,1].plot(t[w],err[w,1],label=name)
        print(json.dumps(metrics))
    ax[0,0].plot(ref[w,0],ref[w,1],'k--',label='reference')
    for a,title in zip(ax.flat,['XY trajectory [m]','3D position error [m]','X error [m]','Y error [m]']):
        a.set_title(title);a.grid();a.legend(fontsize=8)
    ax[0,0].axis('equal')
    fig.tight_layout()
    fig.savefig(HERE/'comparison.png',dpi=160)
    (HERE/'comparison.json').write_text(json.dumps(results,indent=2)+'\n')


if __name__=='__main__':
    main()
