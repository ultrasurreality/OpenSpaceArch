"""pymoo NSGA-III sweep for OpenSpaceArch.
Called from C# via Process.Start("python", "PymooSweep.py <n_gen> <pop_size> <output.json>").
Prints progress to stderr, writes Pareto JSON to stdout or file.
"""
import sys, json, time
import numpy as np
from pymoo.core.problem import Problem
from pymoo.algorithms.moo.nsga3 import NSGA3
from pymoo.util.ref_dirs import get_reference_directions
from pymoo.optimize import minimize

class RocketProblem(Problem):
    def __init__(self):
        super().__init__(
            n_var=6, n_obj=3, n_constr=2,
            xl=np.array([50e5, 2.5, 3.0, 0.3, 1.3, 1.0]),
            xu=np.array([150e5, 4.0, 6.0, 1.2, 2.0, 4.0]),
        )

    def _evaluate(self, X, out, *args, **kwargs):
        Pc=X[:,0]; OF=X[:,1]; CR=X[:,2]; Lstar=X[:,3]; SF=X[:,4]; Twist=X[:,5]
        Tc = 3492 - 50*np.abs(OF-3.2)
        gamma=1.131; cStar=1850-20*np.abs(OF-3.2); g0=9.81; Pa=101325; F=5000
        Cf = np.sqrt(2*gamma**2/(gamma-1)*(2/(gamma+1))**((gamma+1)/(gamma-1))*(1-(Pa/Pc)**((gamma-1)/gamma)))
        Isp = cStar*Cf/g0
        At = F/(Cf*Pc)
        rShroud = np.sqrt(At/(np.pi*0.36))*1000
        wallT = Pc*rShroud/1000/(250e6/SF)*1000
        Dt = 2*np.sqrt(At/np.pi)
        hg = 0.026/Dt**0.2*(8.5e-5)**0.2*2200/0.55**0.6*(Pc/cStar)**0.8
        q = hg*(0.82*Tc-800)
        deltaT = q*(wallT/1000)/320
        sigma_th = 120e9*17e-6*deltaT/(1-0.33)
        Lc = Lstar*At/(CR*At)*1000
        vol = np.pi*(rShroud+5)**2*(Lc+30)*0.35
        mass = vol*1e-9*8900
        out["F"] = np.column_stack([-Isp, mass, sigma_th/1e6])
        out["G"] = np.column_stack([wallT-3.0, 0.5-wallT])

n_gen = int(sys.argv[1]) if len(sys.argv) > 1 else 200
pop_size = int(sys.argv[2]) if len(sys.argv) > 2 else 500
out_path = sys.argv[3] if len(sys.argv) > 3 else None

t0 = time.time()
ref_dirs = get_reference_directions("das-dennis", 3, n_partitions=12)
algo = NSGA3(pop_size=pop_size, ref_dirs=ref_dirs)
res = minimize(RocketProblem(), algo, ("n_gen", n_gen), seed=42, verbose=False)
elapsed = time.time() - t0

results = []
for i in range(len(res.F)):
    Pc,OF,CR,Lstar,SF,Twist = res.X[i]
    results.append(dict(
        Isp=float(-res.F[i,0]), mass=float(res.F[i,1]), stress_MPa=float(res.F[i,2]),
        Pc=float(Pc), OF=float(OF), CR=float(CR),
        Lstar=float(Lstar), SF=float(SF), Twist=float(Twist)))
results.sort(key=lambda r: -r["Isp"])

output = json.dumps(dict(
    n_eval=int(res.algorithm.evaluator.n_eval),
    elapsed_s=round(elapsed, 2),
    pareto_size=len(results),
    solutions=results
), indent=2)

if out_path:
    with open(out_path, "w") as f: f.write(output)
    print(f"pymoo: {len(results)} Pareto solutions in {elapsed:.1f}s -> {out_path}", file=sys.stderr)
else:
    print(output)
